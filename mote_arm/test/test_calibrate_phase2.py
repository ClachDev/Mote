"""Phase 2 end to end against a fake bus: one confirmation, both EEPROM writes.

Each joint's fence is written straight after its own zero, because a fence is
compared against the corrected goal and so outlives the frame it was measured
in: the pair is what has to agree, and splitting them is what left this arm
capped for four months. Nothing is unfenced in between — a joint holds either
its old band or its new one. This drives the whole phase rather than its parts,
so an ordering mistake — fencing before the offset moves, backing up after the
first write, a gap where a joint has no fence — fails here rather than on
hardware.
"""

from mote_arm import arm_calibrate, arm_limits
from mote_arm.calibrate import (
    Sweep,
    calibrate_centred,
    load_limits_backup,
    load_offsets_backup,
)
from mote_arm.config import JointSpec

FULL = arm_limits.FULL_RANGE


class FakeBus:
    """A servo pair whose position reading follows its offset register."""

    def __init__(self, offsets, bands, positions):
        self.offsets = dict(offsets)
        self.bands = dict(bands)
        self.positions = dict(positions)
        self.log = []

    def read_homing_offset(self, servo_id):
        return self.offsets[servo_id]

    def write_homing_offset(self, servo_id, value):
        # present = actual - offset, so the reading moves by the delta.
        self.positions[servo_id] = (
            self.positions[servo_id] - (value - self.offsets[servo_id])
        ) % 4096
        self.offsets[servo_id] = value
        self.log.append(("offset", servo_id, value))
        return True

    def read_angle_limits(self, servo_id):
        return self.bands[servo_id]

    def write_angle_limits(self, servo_id, low, high):
        self.bands[servo_id] = (low, high)
        self.log.append(("fence", servo_id, low, high))
        return True

    def read_position(self, servo_id):
        return self.positions[servo_id]

    def read_position_settled(self, servo_id, **_kw):
        return self.positions[servo_id]


class Recorder:
    def __init__(self, low, high, name):
        self._sweep = Sweep(
            name=name,
            samples=1493,
            min_counts=low,
            max_counts=high,
            wraps=0,
            unwrapped_min=low,
            unwrapped_max=high,
        )

    def result(self):
        return self._sweep


class Args:
    yes = True


JOINTS = [
    JointSpec(name="shoulder_lift", id=2, min_rad=-1.7785, max_rad=1.7785),
    JointSpec(name="elbow_flex", id=3, min_rad=-1.6458, max_rad=1.6458),
]
RECORDERS = {
    "shoulder_lift": Recorder(852, 3236, "shoulder_lift"),
    "elbow_flex": Recorder(949, 3160, "elbow_flex"),
}


def calibrated():
    return {
        j.name: calibrate_centred(j, RECORDERS[j.name].result(), 0.05) for j in JOINTS
    }


def fresh_bus():
    # The state this arm was actually in: fenced by LeRobot in May, offsets
    # moved by a later calibration, fence left behind.
    return FakeBus(
        offsets={2: -1103, 3: 1551},
        bands={2: (1478, 3859), 3: (716, 2941)},
        positions={2: 861, 3: 3146},
    )


def run(bus, tmp_path, monkeypatch):
    monkeypatch.setenv("MOTE_HOME", str(tmp_path))
    return arm_calibrate._phase_centre(bus, JOINTS, RECORDERS, calibrated(), Args())


def test_each_joint_is_fenced_straight_after_its_own_zero(tmp_path, monkeypatch):
    """The pair is what must agree, so nothing comes between them."""
    bus = fresh_bus()
    run(bus, tmp_path, monkeypatch)
    cals = calibrated()
    assert bus.log == [
        ("offset", 2, -1107),
        ("fence", 2, *arm_calibrate.fence_counts(cals["shoulder_lift"])),
        ("offset", 3, 1557),
        ("fence", 3, *arm_calibrate.fence_counts(cals["elbow_flex"])),
    ]


def test_no_joint_is_ever_left_without_a_fence(tmp_path, monkeypatch):
    """Unfencing first would cost a write per joint and manufacture the gap."""
    bus = fresh_bus()
    run(bus, tmp_path, monkeypatch)
    fences = [(entry[2], entry[3]) for entry in bus.log if entry[0] == "fence"]
    assert FULL not in fences


def test_both_backups_are_written_before_the_first_write(tmp_path, monkeypatch):
    bus = fresh_bus()
    run(bus, tmp_path, monkeypatch)
    assert load_offsets_backup() == {"shoulder_lift": -1103, "elbow_flex": 1551}
    assert load_limits_backup() == {
        "shoulder_lift": (1478, 3859),
        "elbow_flex": (716, 2941),
    }


def test_the_arm_ends_fenced_at_its_measured_stops(tmp_path, monkeypatch):
    bus = fresh_bus()
    run(bus, tmp_path, monkeypatch)
    cals = calibrated()
    assert bus.bands == {j.id: arm_calibrate.fence_counts(cals[j.name]) for j in JOINTS}


# A second sweep, taken after a calibration: its centre is already 2048, so the
# offsets it asks for are the ones the servos hold. That is the settled arm.
SETTLED = {
    "shoulder_lift": Recorder(856, 3240, "shoulder_lift"),
    "elbow_flex": Recorder(943, 3153, "elbow_flex"),
}


def settled_calibration():
    return {
        j.name: calibrate_centred(j, SETTLED[j.name].result(), 0.05) for j in JOINTS
    }


def test_a_centred_and_correctly_fenced_arm_writes_nothing(
    tmp_path, monkeypatch, capsys
):
    cals = settled_calibration()
    bus = FakeBus(
        offsets={2: -1107, 3: 1557},
        bands={j.id: arm_calibrate.fence_counts(cals[j.name]) for j in JOINTS},
        positions={2: 2048, 3: 2048},
    )
    monkeypatch.setenv("MOTE_HOME", str(tmp_path))
    arm_calibrate._phase_centre(bus, JOINTS, SETTLED, cals, Args())
    assert bus.log == []
    assert "already centred and fenced" in capsys.readouterr().out


def test_a_stale_fence_is_rewritten_even_when_the_zeros_are_already_right(
    tmp_path, monkeypatch
):
    """The case that hid for four months: right zeros, fence from an old frame."""
    cals = settled_calibration()
    bus = FakeBus(
        offsets={2: -1107, 3: 1557},
        bands={2: (1478, 3859), 3: (716, 2941)},
        positions={2: 2048, 3: 2048},
    )
    monkeypatch.setenv("MOTE_HOME", str(tmp_path))
    arm_calibrate._phase_centre(bus, JOINTS, SETTLED, cals, Args())
    assert bus.bands == {j.id: arm_calibrate.fence_counts(cals[j.name]) for j in JOINTS}
    # No zero moved: only the fence was wrong.
    assert [e for e in bus.log if e[0] == "offset"] == []
