"""The servos' goal-range registers: reading, writing, and the backup pair.

These registers fence which goals a servo will accept and refuse the rest with
no error, no log line and no field in any config file. The arm this package was
written against arrived with five of six joints fenced inside their own travel,
which presented as the shoulder running out of torque. So the properties worth
holding are: a write is only reported once a read-back confirms it, an
out-of-range band is refused rather than truncated, and the as-found values are
recoverable after they have been overwritten.
"""

import pytest

from mote_arm import bus as bus_mod
from mote_arm.arm_limits import FULL_RANGE, cuts
from mote_arm.bus import FeetechBus
from mote_arm.calibrate import load_limits_backup, save_limits_backup
from mote_arm.config import JointSpec

COMM_OK = 0
COMM_FAIL = -1


class StubServo:
    """A packet handler backed by a dict of 2-byte registers."""

    def __init__(self, registers=None, writes_take=True):
        self.registers = dict(registers or {})
        self.writes = []
        self.writes_take = writes_take

    def read1ByteTxRx(self, _port, servo_id, addr):
        return self.registers.get((servo_id, addr), 0), COMM_OK, 0

    def write1ByteTxRx(self, _port, servo_id, addr, value):
        self.writes.append((servo_id, addr, value))
        return COMM_OK, 0

    def read2ByteTxRx(self, _port, servo_id, addr):
        return self.registers.get((servo_id, addr), 0), COMM_OK, 0

    def write2ByteTxRx(self, _port, servo_id, addr, value):
        self.writes.append((servo_id, addr, value))
        if self.writes_take:
            self.registers[(servo_id, addr)] = value
        return COMM_OK, 0


def make_bus(packet):
    bus = FeetechBus("/dev/fake", 1000000)
    bus._packet = packet
    bus._port = object()
    bus._comm_success = COMM_OK
    return bus


def limits_of(packet, servo_id):
    return (
        packet.registers.get((servo_id, bus_mod._MIN_ANGLE_LIMIT)),
        packet.registers.get((servo_id, bus_mod._MAX_ANGLE_LIMIT)),
    )


def fenced(low=1478, high=3859, servo_id=2):
    return StubServo(
        {
            (servo_id, bus_mod._MIN_ANGLE_LIMIT): low,
            (servo_id, bus_mod._MAX_ANGLE_LIMIT): high,
        }
    )


def test_the_band_is_read_back_as_written():
    packet = fenced()
    bus = make_bus(packet)
    assert bus.read_angle_limits(2) == (1478, 3859)
    assert bus.write_angle_limits(2, *FULL_RANGE) is True
    assert limits_of(packet, 2) == FULL_RANGE


def test_a_write_that_does_not_take_is_reported_as_failed():
    """A silent failure here leaves the arm capped while the tool says done."""
    packet = fenced()
    packet.writes_take = False
    assert make_bus(packet).write_angle_limits(2, *FULL_RANGE) is False
    assert limits_of(packet, 2) == (1478, 3859)


def test_an_unreadable_register_is_none_not_a_guess():
    class Deaf(StubServo):
        def read2ByteTxRx(self, _port, servo_id, addr):
            return 0, COMM_FAIL, 0

    assert make_bus(Deaf()).read_angle_limits(2) is None


@pytest.mark.parametrize("band", [(-1, 4095), (0, 4096), (3000, 1000)])
def test_a_band_outside_the_register_is_refused(band):
    with pytest.raises(ValueError):
        make_bus(fenced()).write_angle_limits(2, *band)


def joint(name="shoulder_lift", low=-1.7785, high=1.7785, zero=2048, invert=False):
    return JointSpec(
        name=name, id=2, min_rad=low, max_rad=high, zero_counts=zero, invert=invert
    )


def test_a_band_inside_the_configured_range_is_a_cut():
    # The register that started this: 1478 is -0.874 rad about a zero of 2048,
    # and the joint is configured to reach -1.7785.
    assert cuts(joint(), (1478, 3859)) is True


def test_the_whole_register_range_cuts_nothing():
    assert cuts(joint(), FULL_RANGE) is False


def test_an_inverted_joint_is_judged_by_angle_not_by_register():
    """counts_to_rad flips the sign, so the low count is the high angle."""
    assert cuts(joint(invert=True), (1478, 3859)) is True
    assert cuts(joint(invert=True), FULL_RANGE) is False


def test_the_backup_round_trips(tmp_path):
    path = tmp_path / "arm_limits_backup.yaml"
    found = {"shoulder_lift": (1478, 3859), "wrist_roll": (0, 4095)}
    save_limits_backup(found, {"shoulder_lift": 2, "wrist_roll": 5}, "now", path)
    assert load_limits_backup(path) == found


def test_a_missing_backup_reads_as_empty_rather_than_raising(tmp_path):
    assert load_limits_backup(tmp_path / "absent.yaml") == {}
