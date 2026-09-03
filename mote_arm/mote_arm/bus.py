"""Thin Feetech STS bus wrapper around the ``scservo_sdk`` Python SDK.

All servo I/O is confined to this module so the rest of ``mote_arm`` (config
maths, jog logic, node wiring) can be exercised without hardware. The register
map matches the vendored C++ SDK used by the drive wheels (see
``mote_hardware`` / ``SMS_STS.h``): the arm servos are the same STS class.

``scservo_sdk`` (PyPI ``feetech-servo-sdk``) is imported lazily inside
``open`` so importing this module — and everything above it — never requires the
SDK to be installed. That keeps the build/lint/test gates hardware-free; the SDK
is only needed at runtime on the robot.
"""

from __future__ import annotations

import os
import struct
import time
from dataclasses import dataclass

# STS/SMS register addresses (from SMS_STS.h).
_TORQUE_ENABLE = 40
_ACC = 41
_GOAL_POSITION = 42
_GOAL_SPEED = 46
_LOCK = 55
_MODE = 33
_KP = 21
_KD = 22
_KI = 23
# SMS_STS_OFS_L/H: the position-correction offset, in EEPROM. The servo reports
# `present = actual - offset`, so writing it moves where the joint's zero sits
# in the 0-4095 encoder frame. This is what stops a joint's travel straddling
# the 0/4095 wrap; see mote_arm/calibrate.py.
_HOMING_OFFSET = 31
# SMS_STS_MIN_ANGLE_LIMIT_L / _MAX_ANGLE_LIMIT_L, both EEPROM. In position mode
# the servo refuses a goal outside this band, silently and in one direction —
# which looks exactly like a joint that has run out of torque. Nothing else in
# this repo reads or writes them, so a servo that arrived with a restricted
# range, or was configured with one, is invisible to every tool we have.
_MIN_ANGLE_LIMIT = 9
_MAX_ANGLE_LIMIT = 11
_PRESENT_POSITION = 56
_PRESENT_LOAD = 60
_PRESENT_VOLTAGE = 62
_PRESENT_TEMPERATURE = 63

# STS/SMS is little-endian: protocol_end 0 in scservo_sdk terms.
_PROTOCOL_END = 0
_MODE_POSITION = 0

# One encoder turn, in counts. Duplicated from config.COUNTS_PER_REV rather than
# imported, to keep this module's dependencies to the SDK alone.
COUNTS_PER_TURN = 4096

# The offset register is sign-magnitude, not two's complement: bit 11 is the
# sign and bits 0-10 the magnitude, so it spans +-2047.
OFFSET_SIGN_BIT = 0x800
OFFSET_MAX = 0x7FF


def encode_sign_magnitude(value: int) -> int:
    """Encode a signed offset for the servo's sign-magnitude register."""
    if abs(value) > OFFSET_MAX:
        raise ValueError(f"offset {value} outside the register's +-{OFFSET_MAX} range")
    return (abs(value) | OFFSET_SIGN_BIT) if value < 0 else value


def decode_sign_magnitude(raw: int) -> int:
    """Decode a sign-magnitude register value back to a signed offset."""
    magnitude = raw & OFFSET_MAX
    return -magnitude if raw & OFFSET_SIGN_BIT else magnitude


@dataclass
class ServoHealth:
    id: int
    position: int  # raw counts 0-4095
    voltage: float  # volts
    temperature: int  # degrees C
    load: int  # signed, +-1000 = +-100%


class BusError(RuntimeError):
    pass


def _decode_load(raw: int) -> int:
    """Signed load from PRESENT_LOAD: bit 10 is the sign, low 10 bits the size."""
    return -(raw & 0x3FF) if raw & 0x400 else (raw & 0x3FF)


def port_holders(path: str) -> list[tuple[int, str]]:
    """Return (pid, cmdline) for every *other* process holding ``path`` open.

    The arm shares its serial bus with the drive wheels, so a second opener is
    not merely a conflict — it interleaves packets on the bus that moves the
    robot. Serial ports carry no kernel-level exclusion, so we scan /proc for
    the real device behind the symlink. Processes we cannot inspect (other
    users) are skipped: this is a footgun guard, not a security boundary.
    """
    real = os.path.realpath(path)
    self_pid = os.getpid()
    holders: list[tuple[int, str]] = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid == self_pid:
            continue
        fd_dir = f"/proc/{entry}/fd"
        try:
            fds = os.listdir(fd_dir)
        except OSError:
            continue
        for fd in fds:
            try:
                if os.readlink(f"{fd_dir}/{fd}") != real:
                    continue
            except OSError:
                continue
            try:
                with open(f"/proc/{entry}/cmdline", "rb") as f:
                    cmd = f.read().replace(b"\0", b" ").decode(errors="replace").strip()
            except OSError:
                cmd = "?"
            holders.append((pid, cmd or "?"))
            break
    return holders


def open_bus(cfg) -> "FeetechBus":
    """Open the arm's bus for a setup tool, refusing to share it.

    The arm shares the drive-wheel port, so a second opener interleaves packets
    with the traffic that moves the robot. Every tool that talks to the servos
    directly comes through here: this was copied into four of them, byte for
    byte, which is three chances for one of them to grow a different idea of
    what "the base is running" means.
    """
    holders = port_holders(cfg.port)
    if holders:
        for pid, cmd in holders:
            print(f"  port held by pid {pid}: {cmd}")
        raise SystemExit(
            f"refusing to share {cfg.port} — stop the arm driver / robot base "
            "first (`pixi run kill`)."
        )
    bus = FeetechBus(cfg.port, cfg.baud_rate)
    try:
        bus.open()
    except BusError as exc:
        raise SystemExit(f"cannot open bus: {exc}")
    return bus


class FeetechBus:
    """Position-mode control of Feetech STS servos over one serial bus."""

    def __init__(self, port: str, baud_rate: int):
        self._port_name = port
        self._baud = baud_rate
        self._port = None
        self._packet = None
        self._comm_success = 0
        # Last (speed, acc) confirmed written per servo; see write_goal.
        self._rates: dict[int, tuple[int, int]] = {}

    def open(self, allow_shared: bool = False) -> None:
        """Open the bus, refusing if another process already holds the port.

        On Mote the arm shares the wheel bus, so a concurrent opener (the
        ros2_control node from `pixi run launch`/`mapping`/`robot`) would
        interleave packets with drive-wheel traffic. Refusing is the safe
        default; pass allow_shared=True only to override deliberately.
        """
        if not allow_shared:
            holders = port_holders(self._port_name)
            if holders:
                listed = "; ".join(f"pid {pid}: {cmd}" for pid, cmd in holders)
                raise BusError(
                    f"{self._port_name} is already open by another process "
                    f"({listed}). The arm shares the drive-wheel bus, so two "
                    "openers would corrupt wheel traffic. Stop the robot base "
                    "(e.g. `pixi run kill`) before running the arm."
                )
        try:
            from scservo_sdk import COMM_SUCCESS, PacketHandler, PortHandler
        except ImportError as exc:  # pragma: no cover - runtime-only dependency
            raise BusError(
                "scservo_sdk not available — install the 'feetech-servo-sdk' "
                "PyPI dependency (it is declared in pixi.toml)"
            ) from exc

        self._comm_success = COMM_SUCCESS
        self._port = PortHandler(self._port_name)
        self._packet = PacketHandler(_PROTOCOL_END)
        if not self._port.openPort():
            raise BusError(
                f"failed to open {self._port_name} — check the device exists and "
                "the user is in the 'dialout' group"
            )
        if not self._port.setBaudRate(self._baud):
            raise BusError(f"failed to set baud {self._baud} on {self._port_name}")
        self._rates.clear()

    def close(self) -> None:
        if self._port is not None:
            self._port.closePort()
            self._port = None

    def __enter__(self) -> "FeetechBus":
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def _ok(self, comm: int, err: int) -> bool:
        return comm == self._comm_success and err == 0

    def _read(self, width: int, servo_id: int, address: int) -> int | None:
        """Read a 1- or 2-byte register, or None if the read did not come back.

        Every SDK read goes through here because ``read2ByteTxRx`` indexes the
        received buffer *before* checking it is long enough, so a short reply
        raises IndexError rather than reporting a failure. On a busy shared bus
        that turned a dropped packet into a crash — mid-EEPROM-write, on real
        hardware. A read that does not come back is a None, never an exception.
        """
        reader = (
            self._packet.read1ByteTxRx if width == 1 else self._packet.read2ByteTxRx
        )
        # Drop anything still sitting in the input buffer first. A reply that
        # arrived late — after an EEPROM write, say — would otherwise be
        # consumed as the answer to *this* request, and a register read that
        # silently returns the previous register's value is worse than a failure.
        if self._port is not None and hasattr(self._port, "clearPort"):
            try:
                self._port.clearPort()
            except OSError:
                pass
        try:
            value, comm, err = reader(self._port, servo_id, address)
        except (IndexError, TypeError, struct.error):
            return None
        return value if self._ok(comm, err) else None

    def read_position_settled(
        self, servo_id: int, tolerance: int = 3, attempts: int = 5
    ) -> int | None:
        """A position confirmed by two agreeing reads, for use after a write.

        The single read this replaces was observed returning the *offset*
        register's value right after that register had been written — 3902,
        which is exactly -1854 in the servo's sign-magnitude encoding. Agreement
        within ``tolerance`` counts rejects a stale reply while still allowing
        the count or two of drift a limp arm shows between two reads.
        """
        for _ in range(attempts):
            first = self._read(2, servo_id, _PRESENT_POSITION)
            time.sleep(0.05)
            second = self._read(2, servo_id, _PRESENT_POSITION)
            if first is not None and second is not None:
                if min(abs(first - second), COUNTS_PER_TURN - abs(first - second)) <= (
                    tolerance
                ):
                    return second
            time.sleep(0.1)
        return None

    def ping(self, servo_id: int) -> bool:
        try:
            _model, comm, err = self._packet.ping(self._port, servo_id)
        except (IndexError, TypeError, struct.error):
            return False
        return self._ok(comm, err)

    def read_position(self, servo_id: int) -> int | None:
        """Return raw encoder counts (0-4095), or None on comms failure."""
        return self._read(2, servo_id, _PRESENT_POSITION)

    def read_health(self, servo_id: int) -> ServoHealth | None:
        pos = self.read_position(servo_id)
        if pos is None:
            return None
        volt = self._read(1, servo_id, _PRESENT_VOLTAGE)
        temp = self._read(1, servo_id, _PRESENT_TEMPERATURE)
        raw_load = self._read(2, servo_id, _PRESENT_LOAD)
        if volt is None or temp is None or raw_load is None:
            return None
        return ServoHealth(
            id=servo_id,
            position=pos,
            voltage=volt / 10.0,
            temperature=temp,
            load=_decode_load(raw_load),
        )

    def read_torque(self, servo_id: int) -> bool | None:
        """True if the servo is currently holding, None if unreadable."""
        value = self._read(1, servo_id, _TORQUE_ENABLE)
        return None if value is None else bool(value)

    def read_position_load(self, servo_id: int) -> tuple[int, int] | None:
        """Return (raw counts, signed load), or None on comms failure.

        The two registers a step response is scored on, and only those: a
        sampling loop that also read voltage and temperature would double its
        traffic on the bus the drive wheels share, for values that do not change
        within a step.
        """
        pos = self.read_position(servo_id)
        if pos is None:
            return None
        raw_load, comm, err = self._packet.read2ByteTxRx(
            self._port, servo_id, _PRESENT_LOAD
        )
        if not self._ok(comm, err):
            return None
        return pos, _decode_load(raw_load)

    def set_torque(self, servo_id: int, enable: bool) -> None:
        comm, err = self._packet.write1ByteTxRx(
            self._port, servo_id, _TORQUE_ENABLE, 1 if enable else 0
        )
        if not self._ok(comm, err):
            raise BusError(f"servo {servo_id}: torque write failed")

    def ensure_position_mode(self, servo_id: int) -> bool:
        """Ensure the servo is in position (servo) mode; True once confirmed.

        A servo whose mode cannot be read is left untouched — this bus has been
        observed returning garbled bytes (see ``read_gains``), and a blind
        EEPROM write on a glitch could mis-configure a servo that was fine. A
        servo confirmed in another mode is rewritten and then verified by
        read-back: wheel mode obeys GOAL_SPEED, so position goals sent to an
        unverified servo would spin it continuously.
        """
        mode = self._read_mode(servo_id)
        if mode == _MODE_POSITION:
            return True
        if mode is None:
            return False
        # EEPROM writes: unlock, set mode, re-lock, with brief settle delays
        # between (mirrors the vendored C++ SDK usage in mote_hardware).
        self._packet.write1ByteTxRx(self._port, servo_id, _LOCK, 0)
        time.sleep(0.01)
        self._packet.write1ByteTxRx(self._port, servo_id, _MODE, _MODE_POSITION)
        time.sleep(0.01)
        self._packet.write1ByteTxRx(self._port, servo_id, _LOCK, 1)
        time.sleep(0.01)
        return self._read_mode(servo_id) == _MODE_POSITION

    def _read_mode(self, servo_id: int, attempts: int = 3) -> int | None:
        for attempt in range(attempts):
            mode = self._read(1, servo_id, _MODE)
            if mode is not None:
                return mode
            if attempt + 1 < attempts:
                time.sleep(0.05)
        return None

    def read_gains(self, servo_id: int) -> tuple[int, int, int] | None:
        """Return (kp, kd, ki) from EEPROM, or None if the reads disagree.

        Reads twice and trusts the value only when both agree: a single read
        taken soon after an EEPROM write or a torque cycle has been observed
        returning a garbled byte on this bus, which makes a successful write
        look like a failure.
        """
        for _ in range(5):
            first = [self._read_gain_reg(servo_id, r) for r in (_KP, _KD, _KI)]
            time.sleep(0.05)
            second = [self._read_gain_reg(servo_id, r) for r in (_KP, _KD, _KI)]
            if None not in first and first == second:
                return tuple(first)  # type: ignore[return-value]
            time.sleep(0.1)
        return None

    def read_angle_limits(self, servo_id: int) -> tuple[int, int] | None:
        """Return (min, max) goal counts the servo will accept, or None.

        Read twice and trusted only when both agree, for the same reason
        ``read_gains`` does: these live in EEPROM and a single read on this bus
        has been seen to come back garbled.
        """
        for _ in range(5):
            first = (
                self._read(2, servo_id, _MIN_ANGLE_LIMIT),
                self._read(2, servo_id, _MAX_ANGLE_LIMIT),
            )
            time.sleep(0.05)
            second = (
                self._read(2, servo_id, _MIN_ANGLE_LIMIT),
                self._read(2, servo_id, _MAX_ANGLE_LIMIT),
            )
            if None not in first and first == second:
                return first  # type: ignore[return-value]
            time.sleep(0.1)
        return None

    def write_angle_limits(self, servo_id: int, low: int, high: int) -> bool:
        """Write the goal-range registers to EEPROM and verify they took.

        ``low``/``high`` are raw counts in the same frame as a goal position, so
        0 and 4095 hand the joint its whole single-turn range back. Returns True
        only once a confirmed read-back matches, for the reason
        ``write_homing_offset`` does: this is persistent servo config with no
        copy anywhere else, and reporting an unverified write would leave the
        arm silently capped.
        """
        if not 0 <= low <= high <= COUNTS_PER_TURN - 1:
            raise ValueError(
                f"angle limits {low}..{high} outside 0..{COUNTS_PER_TURN - 1}"
            )
        for _ in range(4):
            self._packet.write1ByteTxRx(self._port, servo_id, _LOCK, 0)
            time.sleep(0.05)
            self._packet.write2ByteTxRx(self._port, servo_id, _MIN_ANGLE_LIMIT, low)
            time.sleep(0.05)
            self._packet.write2ByteTxRx(self._port, servo_id, _MAX_ANGLE_LIMIT, high)
            time.sleep(0.05)
            self._packet.write1ByteTxRx(self._port, servo_id, _LOCK, 1)
            # The read-back races the relock; give the servo time to settle.
            time.sleep(0.15)
            if self.read_angle_limits(servo_id) == (low, high):
                return True
        return False

    def _read_gain_reg(self, servo_id: int, addr: int) -> int | None:
        return self._read(1, servo_id, addr)

    def write_gains(self, servo_id: int, kp: int, kd: int, ki: int) -> bool:
        """Write the position-loop gains to EEPROM and verify they took.

        Retries because the write does not always land. Returns True only once
        a confirmed read-back matches all three values, so callers never report
        success on an unverified change to persistent servo config.
        """
        want = (kp, kd, ki)
        for _ in range(4):
            self._packet.write1ByteTxRx(self._port, servo_id, _LOCK, 0)
            time.sleep(0.05)
            for addr, value in zip((_KP, _KD, _KI), want):
                self._packet.write1ByteTxRx(self._port, servo_id, addr, value)
                time.sleep(0.05)
            self._packet.write1ByteTxRx(self._port, servo_id, _LOCK, 1)
            # The read-back races the relock; give the servo time to settle.
            time.sleep(0.15)
            if self.read_gains(servo_id) == want:
                return True
        return False

    def read_homing_offset(self, servo_id: int) -> int | None:
        """Return the servo's position-correction offset, or None if unreadable.

        Read twice and trusted only when both agree, for the same reason as
        ``read_gains``: this bus returns the occasional garbled byte, and a
        wrong offset here would silently move every angle the arm reports.
        """
        for _ in range(5):
            first = self._read_offset_once(servo_id)
            time.sleep(0.05)
            second = self._read_offset_once(servo_id)
            if first is not None and first == second:
                return decode_sign_magnitude(first)
            time.sleep(0.1)
        return None

    def _read_offset_once(self, servo_id: int) -> int | None:
        return self._read(2, servo_id, _HOMING_OFFSET)

    def read_homing_offset_raw(self, servo_id: int) -> int | None:
        """The offset register's undecoded value, for diagnosing the decoding.

        ``read_homing_offset`` applies the sign-magnitude interpretation. When
        that interpretation is what is in doubt, this is the number to look at.
        """
        for _ in range(5):
            first = self._read_offset_once(servo_id)
            time.sleep(0.05)
            if first is not None and first == self._read_offset_once(servo_id):
                return first
            time.sleep(0.1)
        return None

    def write_homing_offset(self, servo_id: int, offset: int) -> bool:
        """Write the position-correction offset to EEPROM and verify it took.

        Returns True only once a confirmed read-back matches, so a caller never
        reports success on an unverified change to persistent servo config.
        Torque must be off: the offset moves the position frame, and a torqued
        servo would chase its old goal into the new frame.
        """
        encoded = encode_sign_magnitude(offset)
        for _ in range(4):
            self._packet.write1ByteTxRx(self._port, servo_id, _LOCK, 0)
            time.sleep(0.05)
            self._packet.write2ByteTxRx(self._port, servo_id, _HOMING_OFFSET, encoded)
            time.sleep(0.05)
            self._packet.write1ByteTxRx(self._port, servo_id, _LOCK, 1)
            # The read-back races the relock; give the servo time to settle.
            time.sleep(0.15)
            if self.read_homing_offset(servo_id) == offset:
                return True
        return False

    def write_goal(self, servo_id: int, counts: int, speed: int, acc: int) -> None:
        """Command an absolute position (0-4095) at the given speed/accel.

        Speed and acceleration are RAM registers that hold their value for the
        session, so they are rewritten only when they change: a 20 Hz setpoint
        stream then costs one goal write per tick instead of three transactions
        on the bus the drive wheels share.
        """
        if self._rates.get(servo_id) != (speed, acc):
            comm, err = self._packet.write1ByteTxRx(self._port, servo_id, _ACC, acc)
            acc_ok = self._ok(comm, err)
            comm, err = self._packet.write2ByteTxRx(
                self._port, servo_id, _GOAL_SPEED, speed
            )
            if acc_ok and self._ok(comm, err):
                self._rates[servo_id] = (speed, acc)
        comm, err = self._packet.write2ByteTxRx(
            self._port, servo_id, _GOAL_POSITION, counts
        )
        if not self._ok(comm, err):
            raise BusError(f"servo {servo_id}: goal write failed")
