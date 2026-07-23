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

import time
from dataclasses import dataclass

# STS/SMS register addresses (from SMS_STS.h).
_TORQUE_ENABLE = 40
_ACC = 41
_GOAL_POSITION = 42
_GOAL_SPEED = 46
_LOCK = 55
_MODE = 33
_PRESENT_POSITION = 56
_PRESENT_LOAD = 60
_PRESENT_VOLTAGE = 62
_PRESENT_TEMPERATURE = 63

# STS/SMS is little-endian: protocol_end 0 in scservo_sdk terms.
_PROTOCOL_END = 0
_MODE_POSITION = 0


@dataclass
class ServoHealth:
    id: int
    position: int  # raw counts 0-4095
    voltage: float  # volts
    temperature: int  # degrees C
    load: int  # signed, +-1000 = +-100%


class BusError(RuntimeError):
    pass


class FeetechBus:
    """Position-mode control of Feetech STS servos over one serial bus."""

    def __init__(self, port: str, baud_rate: int):
        self._port_name = port
        self._baud = baud_rate
        self._port = None
        self._packet = None
        self._comm_success = 0

    def open(self) -> None:
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

    def ping(self, servo_id: int) -> bool:
        _model, comm, err = self._packet.ping(self._port, servo_id)
        return self._ok(comm, err)

    def read_position(self, servo_id: int) -> int | None:
        """Return raw encoder counts (0-4095), or None on comms failure."""
        pos, comm, err = self._packet.read2ByteTxRx(
            self._port, servo_id, _PRESENT_POSITION
        )
        return pos if self._ok(comm, err) else None

    def read_health(self, servo_id: int) -> ServoHealth | None:
        pos = self.read_position(servo_id)
        if pos is None:
            return None
        volt, comm, err = self._packet.read1ByteTxRx(
            self._port, servo_id, _PRESENT_VOLTAGE
        )
        temp, comm2, err2 = self._packet.read1ByteTxRx(
            self._port, servo_id, _PRESENT_TEMPERATURE
        )
        raw_load, comm3, err3 = self._packet.read2ByteTxRx(
            self._port, servo_id, _PRESENT_LOAD
        )
        if not (
            self._ok(comm, err) and self._ok(comm2, err2) and self._ok(comm3, err3)
        ):
            return None
        # Load bit 10 is the sign; low 10 bits are the magnitude.
        load = -(raw_load & 0x3FF) if raw_load & 0x400 else (raw_load & 0x3FF)
        return ServoHealth(
            id=servo_id,
            position=pos,
            voltage=volt / 10.0,
            temperature=temp,
            load=load,
        )

    def set_torque(self, servo_id: int, enable: bool) -> None:
        comm, err = self._packet.write1ByteTxRx(
            self._port, servo_id, _TORQUE_ENABLE, 1 if enable else 0
        )
        if not self._ok(comm, err):
            raise BusError(f"servo {servo_id}: torque write failed")

    def ensure_position_mode(self, servo_id: int) -> None:
        """Put the servo in position (servo) mode, writing EEPROM only if needed."""
        mode, comm, err = self._packet.read1ByteTxRx(self._port, servo_id, _MODE)
        if self._ok(comm, err) and mode == _MODE_POSITION:
            return
        # EEPROM writes: unlock, set mode, re-lock, with brief settle delays
        # between (mirrors the vendored C++ SDK usage in mote_hardware).
        self._packet.write1ByteTxRx(self._port, servo_id, _LOCK, 0)
        time.sleep(0.01)
        self._packet.write1ByteTxRx(self._port, servo_id, _MODE, _MODE_POSITION)
        time.sleep(0.01)
        self._packet.write1ByteTxRx(self._port, servo_id, _LOCK, 1)
        time.sleep(0.01)

    def write_goal(self, servo_id: int, counts: int, speed: int, acc: int) -> None:
        """Command an absolute position (0-4095) at the given speed/accel."""
        self._packet.write1ByteTxRx(self._port, servo_id, _ACC, acc)
        self._packet.write2ByteTxRx(self._port, servo_id, _GOAL_SPEED, speed)
        comm, err = self._packet.write2ByteTxRx(
            self._port, servo_id, _GOAL_POSITION, counts
        )
        if not self._ok(comm, err):
            raise BusError(f"servo {servo_id}: goal write failed")
