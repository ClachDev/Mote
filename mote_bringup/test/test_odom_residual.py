"""The slip detector's maths, with no ROS and no bag.

Everything the node decides is decided here, so these are the tests that matter:
the node itself only moves messages into the estimator and a verdict out of it.
"""

import math
import os

import pytest
import yaml

from mote_bringup.odom_residual import (
    ICP_FAULT,
    OK,
    SLIP,
    STUCK,
    UNKNOWN,
    PoseTrack,
    Residual,
    ResidualEstimator,
    Thresholds,
    VerdictFilter,
    classify,
    rel_motion,
    yaw_of_quat,
)

CONFIG = os.path.join(os.path.dirname(__file__), "..", "config", "slip.yaml")


def _quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def test_yaw_of_quat_round_trips():
    for yaw in (-3.0, -1.5, 0.0, 0.7, 3.0):
        assert yaw_of_quat(*_quat(yaw)) == pytest.approx(yaw, abs=1e-9)


def test_rel_motion_is_expressed_in_the_body_frame():
    """A pose one metre north, facing north, has moved one metre *forward*."""
    dx, dy, da = rel_motion(0.0, 0.0, math.pi / 2, 0.0, 1.0, math.pi / 2)
    assert dx == pytest.approx(1.0)
    assert dy == pytest.approx(0.0)
    assert da == pytest.approx(0.0)


def test_rel_motion_wraps_the_shortest_way():
    _, _, da = rel_motion(0.0, 0.0, 3.0, 0.0, 0.0, -3.0)
    assert da == pytest.approx(2 * math.pi - 6.0, abs=1e-9)


def test_pose_track_interpolates():
    track = PoseTrack()
    track.add(0.0, 0.0, 0.0, 0.0)
    track.add(1.0, 2.0, 4.0, 1.0)
    x, y, a = track.at(0.5)
    assert (x, y, a) == pytest.approx((1.0, 2.0, 0.5))


def test_pose_track_interpolation_crosses_the_branch_cut():
    """Yaw is accumulated unwrapped, so +pi to -pi is a small step, not a turn."""
    track = PoseTrack()
    track.add(0.0, 0.0, 0.0, math.pi - 0.1)
    track.add(1.0, 0.0, 0.0, -math.pi + 0.1)
    _, _, a = track.at(0.5)
    assert math.atan2(math.sin(a), math.cos(a)) == pytest.approx(math.pi, abs=1e-9)


def test_pose_track_refuses_to_extrapolate():
    """Clamping instead would report zero motion for a window with no data —
    indistinguishable from the robot having stopped."""
    track = PoseTrack()
    track.add(1.0, 0.0, 0.0, 0.0)
    track.add(2.0, 1.0, 0.0, 0.0)
    assert track.at(0.5) is None
    assert track.at(2.5) is None


def test_pose_track_drops_out_of_order_samples():
    track = PoseTrack()
    track.add(1.0, 0.0, 0.0, 0.0)
    track.add(0.5, 9.0, 9.0, 9.0)
    assert len(track) == 1


def test_pose_track_trims_to_its_horizon():
    track = PoseTrack(horizon=1.0)
    for i in range(100):
        track.add(i * 0.1, 0.0, 0.0, 0.0)
    assert len(track) < 20
    assert track.span[1] == pytest.approx(9.9)


def _residual(wheel_dist, icp_dist, wheel_yaw=0.0, icp_yaw=0.0, dt=1.0):
    return Residual(0.0, dt, wheel_dist, icp_dist, wheel_yaw, icp_yaw)


def test_agreement_is_ok():
    r = _residual(0.200, 0.198)
    assert classify(r, Thresholds()).state == OK


def test_slip_needs_both_the_floor_and_the_fraction():
    t = Thresholds()
    # Clears the fraction but not the floor: a small absolute disagreement at
    # low speed is noise, not slip.
    assert classify(_residual(0.050, 0.025), t).state == OK
    # Clears the floor but not the fraction: a larger disagreement on a larger
    # motion is the same measurement made slightly differently.
    assert classify(_residual(0.200, 0.160), t).state == OK
    # Both.
    assert classify(_residual(0.200, 0.100), t).state == SLIP


def test_icp_fault_is_the_other_direction():
    """Slip makes the wheels over-read, never the lidar, so lidar-over-reads is
    a different fault and must not be reported as slip."""
    verdict = classify(_residual(0.100, 0.200), Thresholds())
    assert verdict.state == ICP_FAULT


def test_impossible_speed_is_an_icp_fault_whatever_the_wheels_say():
    t = Thresholds().with_max_wheel_speed(0.218)
    # The wheels agree, but no drive can produce this — so it is the lidar.
    verdict = classify(_residual(0.300, 0.300), t)
    assert verdict.state == ICP_FAULT
    assert "the drive can produce" in verdict.detail


def test_too_slow_to_compare_gives_no_verdict():
    t = Thresholds()
    assert classify(_residual(0.020, 0.000), t).state == UNKNOWN


def test_stuck_needs_a_command():
    t = Thresholds()
    still = _residual(0.000, 0.000)
    # Nothing commanded: parked, not stuck.
    assert classify(still, t, command=None).state == UNKNOWN
    assert classify(still, t, command=(0.0, 0.0)).state == UNKNOWN
    assert classify(still, t, command=(0.2, 0.0)).state == STUCK
    # Commanded rotation counts too — a blocked in-place turn moves nothing.
    assert classify(still, t, command=(0.0, 0.5)).state == STUCK


def test_a_moving_robot_is_not_stuck():
    t = Thresholds()
    r = _residual(0.200, 0.198)
    assert classify(r, t, command=(0.2, 0.0)).state == OK


def test_a_spinning_robot_is_not_stuck():
    """Rotation is motion: the wheels report yaw even when translation is zero."""
    t = Thresholds()
    r = _residual(0.000, 0.000, wheel_yaw=0.5, icp_yaw=0.5)
    assert classify(r, t, command=(0.0, 0.5)).state == UNKNOWN


def test_missing_residual_reports_its_reason():
    verdict = classify(None, Thresholds(), reason="lidar odometry stale (3.0s)")
    assert verdict.state == UNKNOWN
    assert verdict.detail == "lidar odometry stale (3.0s)"


def _feed(estimator, t, wheel_x, icp_x, rate=50.0):
    """Drive both sources to the given x at time t, from wherever they were."""
    estimator.add_wheel(t, wheel_x, 0.0, 0.0)
    estimator.add_icp(t, icp_x, 0.0, 0.0)


def test_estimator_needs_a_full_window():
    est = ResidualEstimator(Thresholds(window=1.0))
    for i in range(5):
        _feed(est, i * 0.1, 0.0, 0.0)
    assert est.residual() is None
    assert "full window" in est.reason


def test_estimator_measures_a_clean_disagreement():
    est = ResidualEstimator(Thresholds(window=1.0))
    for i in range(31):
        t = i * 0.1
        _feed(est, t, wheel_x=0.2 * t, icp_x=0.1 * t)
    r = est.residual()
    assert r.wheel_speed == pytest.approx(0.2, abs=1e-6)
    assert r.icp_speed == pytest.approx(0.1, abs=1e-6)
    assert r.speed_residual == pytest.approx(0.1, abs=1e-6)
    assert r.relative == pytest.approx(0.5, abs=1e-6)


def test_estimator_refuses_a_stale_source():
    """A stalled lidar must not read as slip.

    Without the guard the window freezes at the last lidar pose while the wheels
    keep turning, which grows without bound and looks exactly like slip — the one
    way this node could blame the wheels for a sensor dropout. Measured on a real
    bag: 6 s of frozen scan while the wheels reported 0.21 m/s.
    """
    t = Thresholds(window=1.0, max_lag=1.0)
    est = ResidualEstimator(t)
    for i in range(31):
        _feed(est, i * 0.1, 0.2 * i * 0.1, 0.2 * i * 0.1)
    assert est.residual(now=3.0) is not None
    # Wheels keep reporting; the lidar stops.
    for i in range(31, 61):
        est.add_wheel(i * 0.1, 0.2 * i * 0.1, 0.0, 0.0)
    assert est.residual(now=6.0) is None
    assert "lidar odometry stale" in est.reason


def test_estimator_reports_which_source_is_missing():
    est = ResidualEstimator()
    est.add_wheel(0.0, 0.0, 0.0, 0.0)
    assert est.residual() is None
    assert "lidar" in est.reason


def _hold(vfilter, start, verdict, duration, step=0.2):
    t = start
    last = None
    while t <= start + duration + 1e-9:
        last = vfilter.update(t, verdict)
        t += step
    return last


def test_filter_ignores_a_verdict_that_does_not_hold():
    from mote_bringup.odom_residual import Verdict

    t = Thresholds(hold=0.5, release=2.0)
    f = VerdictFilter(t)
    f.update(0.0, Verdict(OK, "fine"))
    assert f.update(0.1, Verdict(SLIP, "blip")).state == OK


def test_filter_reports_a_verdict_that_holds():
    from mote_bringup.odom_residual import Verdict

    t = Thresholds(hold=0.5, release=2.0)
    f = VerdictFilter(t)
    assert _hold(f, 0.0, Verdict(SLIP, "slipping"), 1.0).state == SLIP


def test_filter_holds_through_a_gap_then_releases():
    """A flapping condition is one event, not several — but it does clear."""
    from mote_bringup.odom_residual import Verdict

    t = Thresholds(hold=0.5, release=2.0)
    f = VerdictFilter(t)
    _hold(f, 0.0, Verdict(SLIP, "slipping"), 1.0)
    # The clock starts at the first quiet window, so `release` is measured from
    # 1.5 s, not from when the slip began.
    assert f.update(1.5, Verdict(OK, "fine")).state == SLIP
    assert f.update(3.4, Verdict(OK, "fine")).state == SLIP
    assert f.update(3.6, Verdict(OK, "fine")).state == OK


def test_packaged_config_parses_into_thresholds():
    """The shipped slip.yaml must name only fields Thresholds has.

    A typo would otherwise be silently dropped by the filter in load_config and
    the robot would quietly run the defaults.
    """
    with open(CONFIG) as f:
        cfg = yaml.safe_load(f)
    known = set(Thresholds.__dataclass_fields__) | {"rate", "max_body_speed_tolerance"}
    assert set(cfg) <= known, f"unknown keys in slip.yaml: {set(cfg) - known}"
    fields = {k: v for k, v in cfg.items() if k in Thresholds.__dataclass_fields__}
    t = Thresholds(**fields)
    assert t.window > 0
    assert t.slip_speed > 0 and 0 < t.slip_fraction < 1


def test_thresholds_sit_clear_of_the_measured_quiet_distribution():
    """Calibration, asserted.

    The quiet bags (20260706_135320, _193037: 394 s of driving) measure a speed
    residual p99 of 0.0062 and 0.0210 m/s and a relative p99 of 0.040 and 0.179.
    The thresholds must stay above those or the detector reports ordinary
    driving. See docs/tuning/2026-07-28-slip-detection.md.
    """
    with open(CONFIG) as f:
        cfg = yaml.safe_load(f)
    fields = {k: v for k, v in cfg.items() if k in Thresholds.__dataclass_fields__}
    t = Thresholds(**fields)
    quiet_p99_speed = 0.0210
    quiet_p99_relative = 0.179
    assert t.slip_speed > quiet_p99_speed
    assert t.slip_fraction > quiet_p99_relative
    assert t.icp_speed > quiet_p99_speed
    assert t.icp_fraction > quiet_p99_relative
