"""`arm-calibrate` writing the servos' goal-range fence with the zeros.

A fence is compared against the *corrected* goal, so it outlives the frame it
was measured in: move a zero under one and it goes on refusing the same counts,
which now name different angles, in silence. That is how this arm broke — a
LeRobot calibration wrote fence and offset together in May 2026, a later
`arm-calibrate` moved the offsets and left the fence behind, and five of six
joints spent four months stopping short of their own travel at 0% load.

So the properties held here are: the fence is the *measured stops*, wider than
the soft limits by the margin, so `arm.yaml` always binds first and a fence can
never be what stops the arm in ordinary use; and `--skip-homing` reports rather
than writes, because it promises to touch no servo. The ordering property — a
joint's fence written straight after its own zero — is in
`test_calibrate_phase2.py`, which drives the phase that does both.
"""

import pytest

from mote_arm import arm_calibrate, arm_limits
from mote_arm.calibrate import (
    JointCalibration,
    Sweep,
    calibrate_centred,
    fence_counts,
    load_limits_backup,
)
from mote_arm.config import RAD_PER_COUNT, JointSpec

FULL = arm_limits.FULL_RANGE


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


def sweep(low=852, high=3236, name="shoulder_lift"):
    return Sweep(
        name=name,
        samples=1493,
        min_counts=low,
        max_counts=high,
        wraps=0,
        unwrapped_min=low,
        unwrapped_max=high,
    )


def spec(name="shoulder_lift", servo_id=2, invert=False):
    return JointSpec(
        name=name, id=servo_id, min_rad=-1.7785, max_rad=1.7785, invert=invert
    )


def calibration(**kw):
    base = dict(
        name="shoulder_lift",
        id=2,
        invert=False,
        zero_counts=2048,
        min_rad=-1.0,
        max_rad=0.5,
        zero_source="the middle of the measured travel",
        margin=0.05,
        sweep=sweep(),
    )
    base.update(kw)
    return JointCalibration(**base)


# --- fence_counts: the band itself ------------------------------------------


def test_the_fence_is_the_measured_travel_not_the_soft_limits():
    """The whole point: arm.yaml binds first, so a fence never stops the arm."""
    cal = calibrate_centred(spec(), sweep(), margin=0.05)
    low, high = fence_counts(cal)
    assert high - low == sweep().unwrapped_span
    soft_low = cal.zero_counts + round(cal.min_rad / RAD_PER_COUNT)
    soft_high = cal.zero_counts + round(cal.max_rad / RAD_PER_COUNT)
    assert low < soft_low and high > soft_high


def test_the_margin_is_exactly_what_separates_them():
    cal = calibrate_centred(spec(), sweep(), margin=0.05)
    low, high = fence_counts(cal)
    margin_counts = round(0.05 / RAD_PER_COUNT)
    assert low == round(cal.zero_counts + cal.min_rad / RAD_PER_COUNT) - margin_counts
    assert high == round(cal.zero_counts + cal.max_rad / RAD_PER_COUNT) + margin_counts


def test_an_inverted_joint_fences_the_mirrored_band():
    """counts_to_rad flips the sign, so the low angle is the high count."""
    plain = fence_counts(calibration(invert=False))
    flipped = fence_counts(calibration(invert=True))
    assert plain == (1364, 2407)
    assert flipped == (1689, 2732)


def test_a_band_running_past_the_encoder_is_clamped_not_wrapped():
    """Wrapping here would fence the far side of the encoder — the whole arm."""
    low, high = fence_counts(calibration(zero_counts=100))
    assert low == 0
    assert high == 100 + round(0.5 / RAD_PER_COUNT) + round(0.05 / RAD_PER_COUNT)


# --- what --skip-homing reports instead of writing ---------------------------


def joints():
    return [spec(), spec(name="wrist_roll", servo_id=5)]


def calibrated():
    return {
        "shoulder_lift": calibrate_centred(spec(), sweep(), 0.05),
        "wrist_roll": calibrate_centred(
            spec(name="wrist_roll", servo_id=5), sweep(130, 3950), 0.05
        ),
    }


def test_an_unreadable_band_stops_the_run_rather_than_assuming_it_is_open():
    bus = FakeBus({5: FULL})  # servo 2 does not answer
    with pytest.raises(SystemExit) as exc:
        arm_calibrate._read_fences(bus, joints())
    assert "shoulder_lift" in str(exc.value)


def test_skip_homing_reports_a_cutting_fence_and_writes_nothing(capsys):
    bus = FakeBus({2: (1478, 3859), 5: FULL})
    arm_calibrate._report_fences(
        joints(), {"shoulder_lift": (1478, 3859), "wrist_roll": FULL}, calibrated()
    )
    assert bus.written == []
    out = capsys.readouterr().out
    assert "shoulder_lift" in out and "wrist_roll" not in out
    assert "arm-limits clear" in out


def test_skip_homing_says_nothing_when_no_fence_cuts(capsys):
    arm_calibrate._report_fences(
        joints(), {"shoulder_lift": FULL, "wrist_roll": FULL}, calibrated()
    )
    assert capsys.readouterr().out == ""


def test_the_as_found_bands_round_trip_through_the_backup(tmp_path, monkeypatch):
    from mote_arm.calibrate import save_limits_backup

    monkeypatch.setenv("MOTE_HOME", str(tmp_path))
    found = {"shoulder_lift": (1478, 3859), "wrist_roll": FULL}
    save_limits_backup(found, {"shoulder_lift": 2, "wrist_roll": 5}, "now")
    assert load_limits_backup() == found
