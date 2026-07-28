"""Unit tests for range calibration (ROS-free, no hardware).

The safety-critical rules live here: a wrapped sweep is never turned into
limits, limits sit inside the measured stops, and a home change is reported
against the poses it invalidates.
"""

import math

import pytest
import yaml

from mote_arm import bus, calibrate
from mote_arm.config import COUNTS_PER_REV, RAD_PER_COUNT, ArmConfig, JointSpec

SPEC = JointSpec(name="elbow_flex", id=3, min_rad=-1.0, max_rad=1.0, zero_counts=2048)


def _record(name, samples):
    rec = calibrate.SweepRecorder(name)
    for s in samples:
        rec.add(s)
    return rec


def _ramp(start, end, step=10):
    direction = 1 if end >= start else -1
    return list(range(start, end + direction, direction * step))


def test_recorder_tracks_min_and_max():
    sweep = _record("j", _ramp(2000, 2500) + _ramp(2500, 1800)).result()
    assert (sweep.min_counts, sweep.max_counts) == (1800, 2500)
    assert sweep.raw_span == 700
    assert sweep.unwrapped_span == 700
    assert sweep.span_rad == pytest.approx(700 * RAD_PER_COUNT)


def test_recorder_counts_samples():
    sweep = _record("j", [2000, 2001, 2002]).result()
    assert sweep.samples == 3


def test_no_wrap_flag_on_ordinary_motion():
    """Even a near-full-range sweep that never crosses 0/4095 is unwrapped."""
    sweep = _record("j", _ramp(20, 4070)).result()
    assert sweep.wraps == 0
    assert sweep.wrapped is False


def test_wrap_detected_crossing_zero():
    # 4090, 4095, then round past the boundary to 5, 15...
    sweep = _record("j", [4070, 4080, 4090, 5, 15, 25]).result()
    assert sweep.wrapped is True
    assert sweep.wraps == 1


def test_wrap_detected_crossing_the_other_way():
    sweep = _record("j", [20, 10, 4090, 4080]).result()
    assert sweep.wraps == 1


def test_unwrapped_span_is_true_across_a_wrap():
    """Raw min/max claim 4070 counts; the real travel 4060 -> 30 is 66."""
    sweep = _record("j", [4060, 4080, 10, 30]).result()
    assert sweep.raw_span == 4070
    assert sweep.unwrapped_span == 66


def test_recorder_rejects_out_of_range_reading():
    with pytest.raises(ValueError):
        _record("j", [COUNTS_PER_REV])


def test_result_without_samples_is_an_error():
    with pytest.raises(calibrate.CalibrationError):
        calibrate.SweepRecorder("j").result()


def test_live_span_available_mid_sweep():
    rec = _record("j", [2000, 2100])
    assert rec.unwrapped_span == 100
    rec.add(1900)
    assert rec.unwrapped_span == 200


def test_limits_sit_inside_the_measured_stops():
    sweep = _record("j", _ramp(1000, 3000)).result()
    lo, hi = calibrate.limits_from_sweep(sweep, zero_counts=2000, margin=0.05)
    outer_lo = (1000 - 2000) * RAD_PER_COUNT
    outer_hi = (3000 - 2000) * RAD_PER_COUNT
    assert lo == pytest.approx(outer_lo + 0.05)
    assert hi == pytest.approx(outer_hi - 0.05)
    assert outer_lo < lo < 0 < hi < outer_hi


def test_zero_margin_gives_the_stops_themselves():
    sweep = _record("j", _ramp(1000, 3000)).result()
    lo, hi = calibrate.limits_from_sweep(sweep, zero_counts=2000, margin=0.0)
    assert lo == pytest.approx((1000 - 2000) * RAD_PER_COUNT)
    assert hi == pytest.approx((3000 - 2000) * RAD_PER_COUNT)


def test_invert_mirrors_the_band():
    sweep = _record("j", _ramp(1000, 3000)).result()
    plain = calibrate.limits_from_sweep(sweep, 1500, invert=False, margin=0.0)
    flipped = calibrate.limits_from_sweep(sweep, 1500, invert=True, margin=0.0)
    assert flipped == pytest.approx((-plain[1], -plain[0]))


def test_wrapped_sweep_refuses_to_produce_limits():
    sweep = _record("j", [4060, 4080, 10, 30]).result()
    with pytest.raises(calibrate.CalibrationError, match="boundary"):
        calibrate.limits_from_sweep(sweep, zero_counts=4070)


def test_zero_outside_the_swept_range_is_refused():
    sweep = _record("j", _ramp(1000, 2000)).result()
    with pytest.raises(calibrate.CalibrationError, match="outside the swept range"):
        calibrate.limits_from_sweep(sweep, zero_counts=2500)


def test_range_too_narrow_for_the_margin_is_refused():
    """A joint swept 0.03 rad cannot carry a 0.05 rad margin at each end."""
    sweep = _record("j", _ramp(2000, 2020, step=5)).result()
    with pytest.raises(calibrate.CalibrationError, match="margin"):
        calibrate.limits_from_sweep(sweep, zero_counts=2010, margin=0.05)


def test_zero_too_close_to_a_stop_is_refused():
    """The shoulder_pan defect: a band that excludes its own zero is rejected."""
    sweep = _record("j", _ramp(2000, 3000)).result()
    with pytest.raises(calibrate.CalibrationError, match="zero"):
        calibrate.limits_from_sweep(sweep, zero_counts=2005, margin=0.05)


def test_every_failure_carries_a_short_reason_for_the_yaml_line():
    """The full message is for the operator; the reason has to fit on a line."""
    wrapped = _record("j", [4060, 4080, 10, 30]).result()
    narrow = _record("j", _ramp(2000, 2020, step=5)).result()
    wide = _record("j", _ramp(2000, 3000)).result()
    cases = [
        (wrapped, 4070, 0.05),  # crosses the encoder boundary
        (wide, 3500, 0.05),  # home outside the swept range
        (narrow, 2010, 0.05),  # range too short for the margin
        (wide, 2005, 0.05),  # home a hair from a stop; zero unreachable
    ]
    for sweep, home, margin in cases:
        try:
            calibrate.limits_from_sweep(sweep, home, margin=margin)
        except calibrate.CalibrationError as exc:
            assert exc.reason != "not calibrated"
            assert len(exc.reason) <= 60
            assert "\n" not in exc.reason
        else:
            raise AssertionError(f"expected a failure for home={home}")


def test_negative_margin_is_rejected():
    sweep = _record("j", _ramp(1000, 3000)).result()
    with pytest.raises(ValueError):
        calibrate.limits_from_sweep(sweep, 2000, margin=-0.01)


def test_calibrate_joint_defaults_zero_to_the_sweep_mid_point():
    sweep = _record("j", _ramp(1000, 3000)).result()
    cal = calibrate.calibrate_joint(SPEC, sweep)
    assert cal.zero_counts == 2000
    assert cal.zero_source == "sweep mid-point"
    assert cal.min_rad == pytest.approx(-cal.max_rad)


def test_calibrate_joint_uses_an_explicit_zero():
    sweep = _record("j", _ramp(1000, 3000)).result()
    cal = calibrate.calibrate_joint(SPEC, sweep, zero_counts=2200)
    assert cal.zero_counts == 2200
    assert cal.zero_source == "measured"
    assert cal.min_rad < 0 < cal.max_rad


def test_calibrate_joint_keeps_the_spec_identity():
    cal = calibrate.calibrate_joint(SPEC, _record("j", _ramp(1000, 3000)).result())
    assert (cal.name, cal.id, cal.invert) == (SPEC.name, SPEC.id, SPEC.invert)


def test_calibrate_joint_reports_a_wrap_rather_than_a_mid_point():
    sweep = _record("j", [4060, 4080, 10, 30]).result()
    with pytest.raises(calibrate.CalibrationError, match="boundary"):
        calibrate.calibrate_joint(SPEC, sweep)


def test_travel_beyond_one_revolution_is_its_own_diagnosis():
    """A continuous joint fits no single-turn frame — 'recentre it' is wrong."""
    # Two full laps: wraps, and an unwrapped span over 4096.
    samples = [(i * 40) % COUNTS_PER_REV for i in range(220)]
    sweep = _record("wrist_roll", samples).result()
    assert sweep.unwrapped_span >= COUNTS_PER_REV
    with pytest.raises(calibrate.CalibrationError, match="revolution") as exc:
        calibrate.limits_from_sweep(sweep, 2048)
    assert "continuous" in exc.value.reason


def test_homing_offset_centres_the_current_pose():
    """The offset that makes 'where it is now' read 2048."""
    assert calibrate.homing_offset(3000, 0) == 3000 - 2048
    assert calibrate.homing_offset(1000, 0) == 1000 - 2048
    assert calibrate.homing_offset(2048, 0) == 0


def test_homing_offset_accounts_for_the_offset_already_written():
    """Re-calibrating must not double-count the offset already in EEPROM.

    A servo reading 2048 that already carries an offset of 500 is physically at
    2548, so re-centring it there is a no-op only if the existing offset is
    included.
    """
    assert calibrate.homing_offset(2048, 500) == 500
    # And a fresh servo moved to the same physical place agrees.
    assert calibrate.homing_offset(2548, 0) == 500


def test_homing_offset_beyond_the_register_range_is_refused():
    with pytest.raises(calibrate.CalibrationError, match="correction range"):
        calibrate.homing_offset(4095, 2000)


def test_centre_is_the_middle_of_the_encoder_frame():
    assert calibrate.CENTRE_COUNTS == COUNTS_PER_REV // 2


def test_sign_magnitude_round_trips_over_the_whole_register_range():
    """Bit 11 is the sign, not two's complement — getting this wrong inverts it."""
    for value in (0, 1, -1, 5, -5, 2047, -2047):
        raw = bus.encode_sign_magnitude(value)
        assert bus.decode_sign_magnitude(raw) == value
    assert bus.encode_sign_magnitude(-1) == 0x801
    assert bus.encode_sign_magnitude(1) == 1
    assert bus.decode_sign_magnitude(0x800) == 0


def test_sign_magnitude_rejects_values_the_register_cannot_hold():
    with pytest.raises(ValueError):
        bus.encode_sign_magnitude(2048)
    with pytest.raises(ValueError):
        bus.encode_sign_magnitude(-2048)


def test_zero_shift_is_the_radian_error_left_in_stored_poses():
    assert calibrate.zero_shift(2048, 2048) == 0.0
    assert calibrate.zero_shift(2048, 2148) == pytest.approx(-100 * RAD_PER_COUNT)
    assert calibrate.zero_shift(2048, 2148, invert=True) == pytest.approx(
        100 * RAD_PER_COUNT
    )


def test_pose_impact_names_only_affected_poses():
    taught = {
        "home": {"elbow_flex": 0.0, "gripper": 0.0},
        "reachy": {"elbow_flex": -3.19},
        "untouched": {"wrist_roll": 0.2},
    }
    impact = calibrate.pose_impact(taught, {"elbow_flex": 0.05, "wrist_flex": 0.1})
    assert sorted(impact) == ["home", "reachy"]
    assert impact["home"] == {"elbow_flex": 0.05}


def test_pose_impact_ignores_unmoved_homes():
    taught = {"home": {"elbow_flex": 0.0}}
    assert calibrate.pose_impact(taught, {"elbow_flex": 0.0}) == {}


def test_pose_impact_empty_without_taught_poses():
    assert calibrate.pose_impact({}, {"elbow_flex": 0.1}) == {}


def _two_joint_config():
    return ArmConfig.from_dict(
        {
            "arm": {
                "port": "/dev/null",
                "baud_rate": 1000000,
                "joints": [
                    {
                        "name": "elbow_flex",
                        "id": 3,
                        "min": -1.0,
                        "max": 1.0,
                        "zero": 2048,
                    },
                    {"name": "gripper", "id": 6, "min": -0.1, "max": 0.1, "zero": 2056},
                ],
            }
        }
    )


def test_emitted_block_parses_back_into_an_arm_config():
    """The block is only useful if pasting it yields a loadable robot.yaml."""
    cfg = _two_joint_config()
    sweep = _record("elbow_flex", _ramp(1000, 3000)).result()
    cal = calibrate.calibrate_joint(cfg.joint("elbow_flex"), sweep)
    block = calibrate.joints_block(list(cfg.joints), {"elbow_flex": cal})

    parsed = yaml.safe_load("arm:\n  port: /dev/null\n  baud_rate: 1000000\n" + block)
    loaded = ArmConfig.from_dict(parsed)
    assert [j.name for j in loaded.joints] == ["elbow_flex", "gripper"]
    elbow = loaded.joint("elbow_flex")
    assert elbow.zero_counts == cal.zero_counts
    assert elbow.min_rad == pytest.approx(cal.min_rad, abs=5e-4)
    # An uncalibrated joint keeps exactly what it had.
    assert loaded.joint("gripper").zero_counts == 2056
    assert loaded.joint("gripper").min_rad == pytest.approx(-0.1)


def test_emitted_block_marks_uncalibrated_joints():
    cfg = _two_joint_config()
    sweep = _record("elbow_flex", _ramp(1000, 3000)).result()
    cal = calibrate.calibrate_joint(cfg.joint("elbow_flex"), sweep)
    block = calibrate.joints_block(
        list(cfg.joints), {"elbow_flex": cal}, {"gripper": "sweep crossed the wrap"}
    )
    assert "# unchanged, sweep crossed the wrap:" in block
    assert "swept 1000-3000" in block
    # The note sits above the line it applies to, not on it.
    lines = block.splitlines()
    assert lines[
        lines.index("    # unchanged, sweep crossed the wrap:") + 1
    ].startswith("    - {name: gripper,")


def test_emitted_block_lines_stay_readable_in_robot_yaml():
    cfg = _two_joint_config()
    sweep = _record("elbow_flex", _ramp(1000, 3000)).result()
    cal = calibrate.calibrate_joint(cfg.joint("elbow_flex"), sweep)
    block = calibrate.joints_block(
        list(cfg.joints), {"elbow_flex": cal}, None, "measured 2026-07-27"
    )
    assert max(len(line) for line in block.splitlines()) <= 110


def test_emitted_block_says_where_zero_came_from():
    cfg = _two_joint_config()
    sweep = _record("elbow_flex", _ramp(1000, 3000)).result()
    centred = calibrate.calibrate_joint(
        cfg.joint("elbow_flex"), sweep, 2100, zero_source="centred"
    )
    block = calibrate.joints_block(list(cfg.joints), {"elbow_flex": centred}, None, "x")
    # Single words, because the header is wrapped and a phrase may straddle lines.
    assert "centred" in block
    assert "INWARD" in block

    derived = calibrate.calibrate_joint(cfg.joint("elbow_flex"), sweep)
    assert "mid-point" in calibrate.joints_block(
        list(cfg.joints), {"elbow_flex": derived}
    )


def test_emitted_block_warns_that_zero_is_not_the_rest_pose():
    """The word collision that confused an operator at the bench, in the file."""
    cfg = _two_joint_config()
    sweep = _record("elbow_flex", _ramp(1000, 3000)).result()
    cal = calibrate.calibrate_joint(cfg.joint("elbow_flex"), sweep)
    block = calibrate.joints_block(list(cfg.joints), {"elbow_flex": cal})
    assert "zero:" in block
    assert "home:" not in block
    flat = " ".join(line.lstrip(" #") for line in block.splitlines())
    assert "not the arm's rest pose" in flat


def test_zero_is_inside_every_emitted_band():
    """Whatever the sweep, a calibrated joint can always be commanded to 0 rad."""
    cfg = _two_joint_config()
    for lo, hi, zero in ((1000, 3000, None), (500, 2500, 1200), (2000, 3800, 3000)):
        sweep = _record("elbow_flex", _ramp(lo, hi)).result()
        cal = calibrate.calibrate_joint(cfg.joint("elbow_flex"), sweep, zero)
        assert cal.min_rad <= 0.0 <= cal.max_rad


def test_record_roundtrip(tmp_path):
    cfg = _two_joint_config()
    sweep = _record("elbow_flex", _ramp(1000, 3000)).result()
    cal = calibrate.calibrate_joint(cfg.joint("elbow_flex"), sweep)
    path = calibrate.save_record(
        {"elbow_flex": cal}, "measured 2026-07-27", tmp_path / "arm_calibration.yaml"
    )
    data = yaml.safe_load(path.read_text())
    assert data["recorded"] == "measured 2026-07-27"
    entry = data["joints"]["elbow_flex"]
    assert (entry["min_counts"], entry["max_counts"]) == (1000, 3000)
    assert entry["zero_source"] == "sweep mid-point"
    assert entry["span_rad"] == pytest.approx(2000 * RAD_PER_COUNT, abs=1e-3)
    assert "homing_offset" not in entry


def test_record_keeps_the_homing_offset_when_one_was_written(tmp_path):
    """The offset lives only in EEPROM; this file is the sole record of it."""
    cfg = _two_joint_config()
    sweep = _record("elbow_flex", _ramp(1000, 3000)).result()
    cal = calibrate.calibrate_joint(cfg.joint("elbow_flex"), sweep)
    path = calibrate.save_record(
        {"elbow_flex": cal},
        "measured 2026-07-28",
        tmp_path / "arm_calibration.yaml",
        offsets={"elbow_flex": -412},
    )
    entry = yaml.safe_load(path.read_text())["joints"]["elbow_flex"]
    assert entry["homing_offset"] == -412


def test_record_path_follows_mote_home(tmp_path, monkeypatch):
    monkeypatch.setenv("MOTE_HOME", str(tmp_path))
    assert calibrate.record_path() == tmp_path / "arm_calibration.yaml"


def test_full_sweep_spans_a_sensible_arc():
    """A 2000-count sweep is ~pi radians — the conversion is not scaled wrong."""
    sweep = _record("j", _ramp(1000, 3048)).result()
    assert sweep.span_rad == pytest.approx(math.pi, abs=0.1)
