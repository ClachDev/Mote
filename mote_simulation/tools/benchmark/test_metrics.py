#!/usr/bin/env python3
"""Unit tests for the ROS-free metrics module.

Runnable standalone (`python test_metrics.py`) or under pytest. Uses synthetic
trajectories so it needs neither ROS nor a sim — the point is to pin the metric
maths that both the live runner and a future offline bag scorer depend on.
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import metrics


def _synthetic_run(yaw_off=0.7, offset=(5.0, -2.0), noise=0.05):
    """A circular truth trajectory and an estimate of the same path expressed in
    a rotated+translated frame (as the SLAM map vs the world frame would be),
    plus gaussian position noise."""
    t = np.linspace(0, 20, 400)
    th = t / 20 * 2 * math.pi
    tx, ty = 3 * np.cos(th), 3 * np.sin(th)
    truth = np.column_stack((t, tx, ty, th)).tolist()
    r = np.array(
        [
            [math.cos(yaw_off), -math.sin(yaw_off)],
            [math.sin(yaw_off), math.cos(yaw_off)],
        ]
    )
    rng = np.random.default_rng(0)
    xy = (np.column_stack((tx, ty)) @ r.T) + np.array(offset)
    xy = xy + rng.normal(0, noise, xy.shape)
    est = np.column_stack((t, xy[:, 0], xy[:, 1], th + yaw_off)).tolist()
    return truth, est


def test_ate_aligns_offset_frames():
    truth, est = _synthetic_run(yaw_off=0.7, offset=(5.0, -2.0), noise=0.05)
    res = metrics.ate(truth, est)
    assert res["raw_rmse_m"] > 2.0  # frames offset -> large pre-alignment error
    assert res["rmse_m"] < 0.1  # aligned error ~= injected noise
    assert abs(res["alignment"]["yaw_deg"] - math.degrees(-0.7)) < 3.0


def test_ate_insufficient_overlap():
    assert metrics.ate([[0, 0, 0, 0]], [[0, 0, 0, 0]])["n"] == 0


def test_goal_stats():
    goals = [
        {"name": "pickup", "result": "ok", "duration": 12.0},
        {"name": "dropoff", "result": "aborted", "duration": None},
        {"name": "home", "result": "ok", "duration": 8.0},
    ]
    g = metrics.goal_stats(goals)
    assert g["success_rate"] == 2 / 3
    assert g["time_to_goal_s"]["total"] == 20.0
    assert g["results"] == {"ok": 2, "aborted": 1}


def test_clearance_bands():
    scan = [[i * 0.1, 0.5 - 0.4 * (i > 40)] for i in range(100)]
    c = metrics.clearance_stats(scan)
    assert abs(c["min_m"] - 0.1) < 1e-6
    assert 0.0 < c["frac_below_0.15m"] <= 1.0


def test_smoothness_counts_reversals():
    cmd = [[i * 0.1, math.sin(i * 0.3), 0.2 * math.cos(i * 0.3)] for i in range(200)]
    s = metrics.smoothness(cmd)
    assert s["direction_reversals"] > 5
    assert s["linear_jerk_rms"] > 0.0


def test_summarize_and_aggregate():
    truth, est = _synthetic_run()
    series = {
        "truth": truth,
        "est": est,
        "scan_min": [[i * 0.1, 0.4] for i in range(50)],
        "cmd": [[i * 0.1, math.sin(i * 0.3), 0.0] for i in range(100)],
        "goals": [{"name": "a", "result": "ok", "duration": 5.0}],
        "recoveries": {"spin": 1, "backup": 2, "wait": 0, "drive_on_heading": 0},
    }
    summ = metrics.summarize(series)
    assert summ["recoveries"]["total"] == 3
    agg = metrics.aggregate([summ, summ])
    assert agg["n_trials"] == 2
    assert agg["metrics"]["localization.rmse_m"]["std"] == 0.0


def main():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print(f"ok  {fn.__name__}")
    print(f"\n{len(fns)} metrics tests passed")


if __name__ == "__main__":
    main()
