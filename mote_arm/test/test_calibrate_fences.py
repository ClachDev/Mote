"""`arm-calibrate` clearing the servos' goal-range fence before it moves a zero.

The fence binds only under torque, so phase 1 sweeps a limp joint straight
through it and measures travel the arm will afterwards refuse to make. Leaving
it in place therefore produces a calibration that describes a range the arm
stops short of, silently, at the same angle every time. The properties held
here: a fenced joint is cleared, an unfenced arm is left alone entirely, the
as-found bands are snapshotted before the first write, and a write that cannot
be verified stops the run rather than continuing into the offsets.
"""

import pytest

from mote_arm import arm_calibrate
from mote_arm.calibrate import Sweep, load_limits_backup
from mote_arm.config import JointSpec

FULL = arm_calibrate.FULL_RANGE


class FakeBus:
    def __init__(self, bands, writes_take=True):
        self.bands = dict(bands)
        self.writes_take = writes_take
        self.written = []

    def read_angle_limits(self, servo_id):
        return self.bands.get(servo_id)

    def write_angle_limits(self, servo_id, low, high):
        self.written.append((servo_id, low, high))
        if not self.writes_take:
            return False
        self.bands[servo_id] = (low, high)
        return True


class FakeRecorder:
    def __init__(self, span):
        self._span = span

    def result(self):
        return Sweep(
            name="j",
            samples=100,
            min_counts=852,
            max_counts=3236,
            wraps=0,
            unwrapped_min=852,
            unwrapped_max=852 + self._span,
        )


class Args:
    yes = True


def joints():
    return [
        JointSpec(name="shoulder_lift", id=2, min_rad=-1.7785, max_rad=1.7785),
        JointSpec(name="wrist_roll", id=5, min_rad=-2.88, max_rad=2.88),
    ]


def recorders():
    return {"shoulder_lift": FakeRecorder(2384), "wrist_roll": FakeRecorder(3820)}


def test_a_fenced_joint_is_cleared(tmp_path, monkeypatch, capsys):
    monkeypatch.setenv("MOTE_HOME", str(tmp_path))
    bus = FakeBus({2: (1478, 3859), 5: FULL})
    arm_calibrate._clear_fences(bus, joints(), recorders(), Args())
    assert bus.written == [(2, 0, 4095)]
    assert bus.bands[2] == FULL
    # The one joint that already accepted its whole range is not rewritten.
    assert "wrist_roll" not in capsys.readouterr().out.split("=== the servos")[1]


def test_the_as_found_bands_are_saved_before_the_first_write(tmp_path, monkeypatch):
    monkeypatch.setenv("MOTE_HOME", str(tmp_path))
    bus = FakeBus({2: (1478, 3859), 5: FULL})
    arm_calibrate._clear_fences(bus, joints(), recorders(), Args())
    assert load_limits_backup() == {"shoulder_lift": (1478, 3859), "wrist_roll": FULL}


def test_an_unfenced_arm_writes_nothing_and_says_nothing(tmp_path, monkeypatch, capsys):
    monkeypatch.setenv("MOTE_HOME", str(tmp_path))
    bus = FakeBus({2: FULL, 5: FULL})
    arm_calibrate._clear_fences(bus, joints(), recorders(), Args())
    assert bus.written == []
    assert capsys.readouterr().out == ""


def test_a_write_that_cannot_be_verified_stops_the_run(tmp_path, monkeypatch):
    monkeypatch.setenv("MOTE_HOME", str(tmp_path))
    bus = FakeBus({2: (1478, 3859), 5: FULL}, writes_take=False)
    with pytest.raises(SystemExit) as exc:
        arm_calibrate._clear_fences(bus, joints(), recorders(), Args())
    assert "arm-limits restore" in str(exc.value)


def test_an_unreadable_band_stops_the_run_rather_than_assuming_it_is_open(
    tmp_path, monkeypatch
):
    monkeypatch.setenv("MOTE_HOME", str(tmp_path))
    bus = FakeBus({5: FULL})  # servo 2 does not answer
    with pytest.raises(SystemExit) as exc:
        arm_calibrate._clear_fences(bus, joints(), recorders(), Args())
    assert "shoulder_lift" in str(exc.value)
