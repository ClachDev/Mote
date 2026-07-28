"""Unit tests for range calibration (ROS-free, no hardware).

The safety-critical rules live here: a wrapped sweep is never turned into
limits, limits sit inside the measured stops, and a home change is reported
against the poses it invalidates.
"""

import math

import pytest
import yaml

from mote_arm import bus, calibrate
from mote_arm import config as config_mod
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


def test_measured_centre_is_the_middle_of_an_ordinary_sweep():
    sweep = _record("j", _ramp(1000, 3000)).result()
    assert sweep.measured_centre == 2000


def test_measured_centre_survives_a_sweep_across_the_wrap():
    """The reason the centre comes from the sweep and not a held pose.

    A joint travelling 3900 -> 4095/0 -> 300 is centred at 4098 % 4096 == 2,
    which raw min/max (0 and 4095) could never tell you.
    """
    samples = [(3900 + i * 10) % COUNTS_PER_REV for i in range(50)]
    sweep = _record("j", samples).result()
    assert sweep.wrapped
    assert sweep.unwrapped_span == 490
    assert sweep.measured_centre == (3900 + 245) % COUNTS_PER_REV


def test_centred_limits_are_symmetric_about_zero():
    sweep = _record("j", _ramp(1000, 3000)).result()
    lo, hi = calibrate.centred_limits(sweep, margin=0.05)
    half = 1000 * RAD_PER_COUNT
    assert lo == pytest.approx(-half + 0.05)
    assert hi == pytest.approx(half - 0.05)
    assert lo < 0 < hi


def test_centred_limits_never_exclude_their_own_zero():
    """By construction, unlike the pose-envelope method that shipped."""
    for lo, hi in ((1000, 3000), (10, 400), (2000, 3999), (0, 4090)):
        sweep = _record("j", _ramp(lo, hi)).result()
        if sweep.unwrapped_span >= COUNTS_PER_REV:
            continue
        band = calibrate.centred_limits(sweep, margin=0.05)
        assert band[0] < 0 < band[1]


def test_centred_limits_work_for_a_wrapped_sweep():
    """limits_from_sweep must refuse this; centred_limits must not."""
    samples = [(3900 + i * 10) % COUNTS_PER_REV for i in range(50)]
    sweep = _record("j", samples).result()
    with pytest.raises(calibrate.CalibrationError, match="boundary"):
        calibrate.limits_from_sweep(sweep, 2048)
    lo, hi = calibrate.centred_limits(sweep, margin=0.05)
    assert lo < 0 < hi


def test_calibrate_centred_puts_zero_at_the_encoder_middle():
    sweep = _record("j", _ramp(1000, 3000)).result()
    cal = calibrate.calibrate_centred(SPEC, sweep)
    assert cal.zero_counts == calibrate.CENTRE_COUNTS
    assert cal.zero_source == "the middle of the measured travel"


def test_centred_limits_still_reject_a_continuous_joint():
    samples = [(i * 40) % COUNTS_PER_REV for i in range(220)]
    sweep = _record("wrist_roll", samples).result()
    with pytest.raises(calibrate.CalibrationError, match="revolution"):
        calibrate.centred_limits(sweep)


def test_centring_a_wrapped_joint_moves_its_whole_travel_inside_the_frame():
    """End to end: the shoulder_pan case, in numbers.

    A joint whose stops are 3200 and 5900 in the servo's own frame wraps. After
    the offset that centres its measured mid-travel, both stops sit inside
    0-4095 with room to spare, which is what makes it calibratable at all.
    """
    low_stop, high_stop = 3200, 5900
    samples = [
        (low_stop + (high_stop - low_stop) * i // 60) % COUNTS_PER_REV
        for i in range(61)
    ]
    sweep = _record("shoulder_pan", samples).result()
    assert sweep.wrapped

    offset = calibrate.homing_offset(sweep.measured_centre, 0)
    moved = [(stop - offset) % COUNTS_PER_REV for stop in (low_stop, high_stop)]
    assert all(0 <= m < COUNTS_PER_REV for m in moved)
    # Centred: the two stops straddle 2048 roughly evenly.
    assert min(moved) < calibrate.CENTRE_COUNTS < max(moved)
    assert abs((min(moved) + max(moved)) // 2 - calibrate.CENTRE_COUNTS) <= 1


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


def test_homing_offset_folds_instead_of_refusing():
    """The bench failure: 3056 is out of register range but equals -1040.

    `present = (actual - offset) mod 4096`, so offsets are modular and an
    arithmetic result outside the register is never a real failure. Rejecting
    one aborted a perfectly good calibration on the real arm.
    """
    assert calibrate.homing_offset(4095, 1009) == -1040
    assert 4095 + 1009 - 2048 == 3056  # what the naive arithmetic produced
    assert abs(calibrate.homing_offset(4095, 1009)) <= bus.OFFSET_MAX


def test_normalise_offset_is_modular_and_encodable():
    for raw, folded in ((3056, -1040), (-3056, 1040), (0, 0), (2047, 2047)):
        assert calibrate.normalise_offset(raw) == folded
    # Every residue folds into something the register can actually hold.
    for raw in range(-8000, 8000, 37):
        folded = calibrate.normalise_offset(raw)
        assert abs(folded) <= bus.OFFSET_MAX
        bus.encode_sign_magnitude(folded)


def test_normalise_offset_preserves_what_the_servo_computes():
    """A folded offset must command the same reading as the unfolded one."""
    for actual in (0, 500, 2048, 4095):
        for raw in (3056, -3056, 5000):
            unfolded = (actual - raw) % COUNTS_PER_REV
            folded = (actual - calibrate.normalise_offset(raw)) % COUNTS_PER_REV
            assert unfolded == folded


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


def test_zero_is_inside_every_emitted_band():
    """Whatever the sweep, a calibrated joint can always be commanded to 0 rad."""
    cfg = _two_joint_config()
    for lo, hi, zero in ((1000, 3000, None), (500, 2500, 1200), (2000, 3800, 3000)):
        sweep = _record("elbow_flex", _ramp(lo, hi)).result()
        cal = calibrate.calibrate_joint(cfg.joint("elbow_flex"), sweep, zero)
        assert cal.min_rad <= 0.0 <= cal.max_rad


def test_full_sweep_spans_a_sensible_arc():
    """A 2000-count sweep is ~pi radians — the conversion is not scaled wrong."""
    sweep = _record("j", _ramp(1000, 3048)).result()
    assert sweep.span_rad == pytest.approx(math.pi, abs=0.1)


def test_offsets_backup_round_trips(tmp_path):
    path = calibrate.save_offsets_backup(
        {"shoulder_pan": 2027, "gripper": 1317},
        {"shoulder_pan": 1, "gripper": 6},
        "2026-07-28 10:00:00Z",
        tmp_path / "arm_offsets_backup.yaml",
    )
    assert calibrate.load_offsets_backup(path) == {
        "shoulder_pan": 2027,
        "gripper": 1317,
    }
    data = yaml.safe_load(path.read_text())
    assert data["saved"] == "2026-07-28 10:00:00Z"
    assert data["offsets"]["shoulder_pan"]["id"] == 1


def test_offsets_backup_absent_is_empty_not_an_error(tmp_path):
    assert calibrate.load_offsets_backup(tmp_path / "nope.yaml") == {}


def test_offsets_backup_path_follows_mote_home(tmp_path, monkeypatch):
    monkeypatch.setenv("MOTE_HOME", str(tmp_path))
    assert calibrate.offsets_backup_path() == tmp_path / "arm_offsets_backup.yaml"


def test_offsets_backup_preserves_negative_values(tmp_path):
    """The real arm's offsets are signed; a backup that lost the sign is useless."""
    path = calibrate.save_offsets_backup(
        {"shoulder_lift": -1723, "wrist_flex": -1706},
        {"shoulder_lift": 2, "wrist_flex": 4},
        "now",
        tmp_path / "b.yaml",
    )
    assert calibrate.load_offsets_backup(path)["shoulder_lift"] == -1723


def _row(samples, now=None):
    from mote_arm import arm_calibrate

    return arm_calibrate._range_row(
        "shoulder_pan", _record("shoulder_pan", samples), now
    )


def _document(**kw):
    cfg = _two_joint_config()
    sweep = _record("elbow_flex", _ramp(1000, 3000)).result()
    cal = calibrate.calibrate_centred(cfg.joint("elbow_flex"), sweep)
    return cfg, calibrate.calibration_document(
        list(cfg.joints), {"elbow_flex": cal}, "measured 2026-07-28", **kw
    )


def test_calibration_document_holds_only_calibrated_joints():
    """A skipped joint is absent, so it keeps the packaged default."""
    _cfg, doc = _document()
    assert list(doc["joints"]) == ["elbow_flex"]
    assert doc["recorded"] == "measured 2026-07-28"


def test_calibration_document_carries_the_measurement_with_the_value():
    _cfg, doc = _document()
    entry = doc["joints"]["elbow_flex"]
    assert entry["zero"] == calibrate.CENTRE_COUNTS
    assert entry["swept_counts"] == [1000, 3000]
    assert entry["swept_rad"] == pytest.approx(2000 * RAD_PER_COUNT, abs=1e-3)


def test_calibration_document_records_the_homing_offset():
    """It exists nowhere but servo EEPROM, so this is the only record of it."""
    _cfg, doc = _document(offsets={"elbow_flex": -1613})
    assert doc["joints"]["elbow_flex"]["homing_offset"] == -1613


def test_calibration_overlays_only_the_measured_fields():
    cfg, doc = _document()
    merged = config_mod.apply_calibration(cfg, doc)
    elbow = merged.joint("elbow_flex")
    assert elbow.zero_counts == calibrate.CENTRE_COUNTS
    assert elbow.min_rad < 0 < elbow.max_rad
    # Identity and direction stay with the package.
    assert (elbow.id, elbow.invert) == (3, False)
    # An uncalibrated joint is untouched.
    assert merged.joint("gripper").zero_counts == 2056
    assert merged.joint("gripper").min_rad == pytest.approx(-0.1)


def test_calibration_absent_leaves_the_packaged_config_alone():
    cfg = _two_joint_config()
    assert config_mod.apply_calibration(cfg, {}) == cfg
    assert config_mod.apply_calibration(cfg, None) == cfg


def test_calibration_for_an_unknown_joint_is_ignored():
    """Removing a joint upstream must not brick a robot with an old file."""
    cfg = _two_joint_config()
    merged = config_mod.apply_calibration(cfg, {"joints": {"ghost": {"zero": 1}}})
    assert merged == cfg


def test_calibration_with_inverted_limits_is_refused():
    cfg = _two_joint_config()
    bad = {"joints": {"elbow_flex": {"min": 1.0, "max": -1.0}}}
    with pytest.raises(ValueError):
        config_mod.apply_calibration(cfg, bad)


def test_saved_calibration_round_trips_and_keeps_its_header(tmp_path):
    cfg, doc = _document(offsets={"elbow_flex": -1613})
    path = calibrate.save_calibration(doc, tmp_path / "arm.yaml")
    text = path.read_text()
    assert text.startswith("# This robot's measured arm calibration")
    unwrapped = " ".join(ln.lstrip("# ") for ln in text.splitlines())
    assert "NOT the arm's rest pose" in unwrapped
    loaded = config_mod.load_calibration(path)
    assert loaded == doc
    merged = config_mod.apply_calibration(cfg, loaded)
    assert merged.joint("elbow_flex").zero_counts == calibrate.CENTRE_COUNTS


def test_calibration_path_follows_mote_home(tmp_path, monkeypatch):
    monkeypatch.setenv("MOTE_HOME", str(tmp_path))
    assert config_mod.calibration_path() == tmp_path / "arm.yaml"


def test_missing_calibration_file_is_empty_not_an_error(tmp_path):
    assert config_mod.load_calibration(tmp_path / "nope.yaml") == {}


def test_range_row_is_the_same_shape_for_every_joint():
    """The bench complaint: one joint showed dashes where the others had numbers.

    A joint whose travel crosses the encoder zero gets centred like any other
    and ends up with an ordinary band, so its row must not look special.
    """
    crosses_zero = _row([(3900 + i * 10) % COUNTS_PER_REV for i in range(50)], now=10)
    ordinary = _row(_ramp(1000, 1490), now=1200)
    assert "-" not in crosses_zero
    assert crosses_zero.count(" rad") == ordinary.count(" rad") == 1
    # Same travel, so the same reported range.
    assert crosses_zero.split()[-2] == ordinary.split()[-2]


def test_range_row_does_not_count_how_often_the_operator_waved_the_joint():
    """Two passes over the boundary describe the same travel as one."""
    lap = [(3900 + i * 10) % COUNTS_PER_REV for i in range(50)]
    back = list(reversed(lap))
    assert _row(lap + back, now=3900) == _row(lap + back + lap + back, now=3900)


def test_range_row_reports_the_true_travel_across_the_boundary():
    """Raw min/max would claim ~4090 counts; the real travel is 490."""
    row = _row([(3900 + i * 10) % COUNTS_PER_REV for i in range(50)], now=10)
    assert "0.75 rad" in row
    assert "4093" not in row and "3900" not in row


def test_range_row_reports_a_joint_that_never_answered():
    from mote_arm import arm_calibrate

    row = arm_calibrate._range_row("gripper", calibrate.SweepRecorder("gripper"), None)
    assert "no readings" in row


def test_only_a_sweep_past_a_whole_turn_is_treated_as_continuous():
    """No fuzzy threshold: 94% of a turn is just a long range, and is accepted.

    A joint that spins freely but was rotated less than a full turn is
    indistinguishable from one with stops, so guessing at 90% would both miss
    most real cases and cry wolf on a long-but-stopped joint.
    """
    long_but_stopped = _record("wrist_roll", _ramp(0, int(0.94 * COUNTS_PER_REV)))
    lo, hi = calibrate.centred_limits(long_but_stopped.result(), margin=0.05)
    assert lo < 0 < hi

    two_laps = _record("wrist_roll", [(i * 40) % COUNTS_PER_REV for i in range(220)])
    with pytest.raises(calibrate.CalibrationError, match="revolution"):
        calibrate.centred_limits(two_laps.result())


REAL_YAML = """# leading comment that must survive
servos:
  port: /dev/mote_servos
  left_id: 7
arm:
  port: /dev/mote_servos
  baud_rate: 1000000
  gains:
    kp: 32
  # explanation the operator wrote and wants kept
  #
  # BEGIN arm.joints — rewritten by the tool
  # NOT YET CALIBRATED, provisional values
  joints:
    - {name: elbow_flex, id: 3, min: -1.0, max: 1.0, zero: 2048, invert: false}
    - {name: gripper, id: 6, min: -0.1, max: 0.1, zero: 2056, invert: false}
  # END arm.joints

lidar:
  port: /dev/mote_lidar
"""
