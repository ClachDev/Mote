"""The stuck detector decides when the robot stops trusting its own scan and
backs off — pure threshold logic, so pin it down without ROS."""

from mote_bringup.explore import StuckDetector


def walk(det, t0, t1, x_of, forward=True, step=0.1):
    fired = False
    t = t0
    while t <= t1:
        fired = det.update(t, x_of(t), 0.0, forward)
        if fired:
            break
        t += step
    return fired, t


def test_no_motion_while_commanded_fires():
    det = StuckDetector(window=6.0, min_travel=0.10)
    fired, at = walk(det, 0.0, 20.0, lambda t: 0.0)
    assert fired
    assert 5.0 <= at <= 8.0


def test_normal_driving_never_fires():
    det = StuckDetector(window=6.0, min_travel=0.10)
    fired, _ = walk(det, 0.0, 30.0, lambda t: 0.28 * t)
    assert not fired


def test_creeping_below_threshold_fires():
    det = StuckDetector(window=6.0, min_travel=0.10)
    fired, _ = walk(det, 0.0, 30.0, lambda t: 0.005 * t)
    assert fired


def test_turn_in_place_resets():
    det = StuckDetector(window=6.0, min_travel=0.10)
    fired, _ = walk(det, 0.0, 5.0, lambda t: 0.0)
    assert not fired
    assert det.update(5.1, 0.0, 0.0, False) is False
    fired, _ = walk(det, 5.2, 10.0, lambda t: 0.0)
    assert not fired


def test_window_not_full_never_fires():
    det = StuckDetector(window=6.0, min_travel=0.10)
    fired, _ = walk(det, 0.0, 4.0, lambda t: 0.0)
    assert not fired


def test_reset_clears_history():
    det = StuckDetector(window=6.0, min_travel=0.10)
    walk(det, 0.0, 5.0, lambda t: 0.0)
    det.reset()
    fired, _ = walk(det, 5.0, 9.0, lambda t: 0.0)
    assert not fired
