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


def test_loop_drift_closes_and_ratio():
    # A closed square loop: start == end -> ~zero drift, small ratio.
    pts = [(0, 0), (2, 0), (2, 2), (0, 2), (0, 0)]
    traj = []
    for i in range(len(pts) - 1):
        (x0, y0), (x1, y1) = pts[i], pts[i + 1]
        for s in np.linspace(0, 1, 20, endpoint=False):
            traj.append([i + s, x0 + s * (x1 - x0), y0 + s * (y1 - y0), 0.0])
    traj.append([len(pts), 0.0, 0.0, 0.0])
    d = metrics.loop_drift(traj)
    assert d["start_end_dist_m"] < 1e-6
    assert abs(d["path_length_m"] - 8.0) < 0.1
    assert d["drift_ratio"] < 1e-6


def test_loop_drift_open_traverse():
    traj = [[i * 0.1, i * 0.1, 0.0, 0.0] for i in range(50)]
    d = metrics.loop_drift(traj)
    assert d["start_end_dist_m"] > 4.0  # A->B, honestly large
    assert 0.9 < d["drift_ratio"] <= 1.0  # straight line: dist ~= path length


def test_map_quality_crisp_vs_blurred():
    # Crisp: a single-cell-thick wall on an otherwise-free field.
    crisp = np.zeros((60, 60), dtype=int)
    crisp[30, 5:55] = 100
    # Blurred/smeared: a thick 5-cell wall (bad scan-match proxy).
    blurred = np.zeros((60, 60), dtype=int)
    blurred[28:33, 5:55] = 100
    qc = metrics.map_quality(crisp, 0.05)
    qb = metrics.map_quality(blurred, 0.05)
    assert qc["mean_wall_thickness_m"] < qb["mean_wall_thickness_m"]
    assert abs(qc["mean_wall_thickness_m"] - 0.05) < 1e-6  # 1 cell => ~1*res
    assert qc["unknown_frac"] == 0.0


def test_map_quality_speckle_and_unknown():
    grid = np.full((40, 40), -1, dtype=int)  # all unknown
    grid[10:30, 10:30] = 0  # a free box
    grid[0, 0] = 100  # one isolated occupied speck inside unknown
    q = metrics.map_quality(grid, 0.05)
    assert q["speckle_frac"] == 1.0  # the lone occupied cell is a speck
    assert 0.0 < q["unknown_frac"] < 1.0
    assert q["explored_area_m2"] > 0.0


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
