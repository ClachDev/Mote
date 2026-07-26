"""Objective mission metrics, computed from recorded time series.

This module is deliberately **ROS-free and numpy-only** so it can score any
run, whoever produced the series: the live sim runner (``record.py``) and a
future offline bag-replay scorer both feed it the same shapes. Nothing here
imports rclpy, reads a topic, or touches the filesystem — it is pure functions
over arrays of samples.

Series shapes (all timestamps are seconds on one clock, usually sim ``/clock``):

* ``truth`` / ``est`` — trajectories, ``list[[t, x, y, yaw]]`` (metres, radians)
* ``scan_min`` — nearest obstacle per scan, ``list[[t, min_range]]`` (metres)
* ``cmd`` — commanded velocity, ``list[[t, vx, wz]]`` (m/s, rad/s)
* ``goals`` — ``list[{name, x, y, yaw, result, t_send, t_done, duration}]`` where
  ``result`` is one of ``ok`` / ``aborted`` / ``rejected`` / ``timeout``
* ``recoveries`` — ``{spin, backup, drive_on_heading, wait, total}`` counts

``summarize(series)`` runs the whole battery and returns a JSON-friendly dict.
``aggregate(list_of_summaries)`` reduces repeated trials to mean/std/min/max so
two benchmark runs can be compared and the run-to-run variance is explicit.
"""

from __future__ import annotations

import math

import numpy as np

# Clearance thresholds (m): fraction of time the nearest obstacle sits inside
# each band. 0.20 m is roughly the robot's own half-footprint + margin, so time
# spent below it is "close-call" driving.
CLEARANCE_BANDS = (0.15, 0.20, 0.30)


def _arr(samples) -> np.ndarray:
    a = np.asarray(samples, dtype=float)
    return a.reshape(0, 0) if a.size == 0 else a


def rigid_align_2d(src: np.ndarray, dst: np.ndarray):
    """Least-squares SE(2) fit (rotation + translation, no scale) mapping ``src``
    onto ``dst``. Returns ``(R, t, yaw)`` with ``R`` a 2x2 rotation, ``t`` a
    length-2 translation, and ``yaw`` the rotation angle. Closed-form Umeyama.

    Alignment matters because the SLAM ``map`` frame and the Gazebo world frame
    share no fixed transform — the map origin is wherever mapping started — so
    truth and estimate live in different frames until aligned.
    """
    src = np.asarray(src, dtype=float)
    dst = np.asarray(dst, dtype=float)
    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    h = (src - mu_s).T @ (dst - mu_d)
    u, _, vt = np.linalg.svd(h)
    d = np.sign(np.linalg.det(vt.T @ u.T))
    r = vt.T @ np.diag([1.0, d]) @ u.T
    t = mu_d - r @ mu_s
    return r, t, math.atan2(r[1, 0], r[0, 0])


def _resample(ref_t: np.ndarray, other_t: np.ndarray, other_xy: np.ndarray):
    """Linearly interpolate ``other`` onto ``ref_t``, keeping only ref samples
    inside the other series' time span (no extrapolation). Returns
    ``(mask, xy_on_ref)``."""
    lo, hi = other_t[0], other_t[-1]
    mask = (ref_t >= lo) & (ref_t <= hi)
    x = np.interp(ref_t[mask], other_t, other_xy[:, 0])
    y = np.interp(ref_t[mask], other_t, other_xy[:, 1])
    return mask, np.column_stack((x, y))


def ate(truth, est) -> dict:
    """Absolute trajectory error: RMS/mean/median/max Euclidean distance between
    the estimated and true positions, after a rigid SE(2) alignment.

    The estimate is time-associated to truth by interpolating the (typically
    higher-rate) estimate onto the truth timestamps, then aligned and differenced.
    Also reports the *raw* (pre-alignment) RMSE, which is small only if the map
    and world frames already coincide.
    """
    t, e = _arr(truth), _arr(est)
    if t.shape[0] < 3 or e.shape[0] < 3:
        return {"n": 0, "note": "insufficient trajectory samples"}
    mask, est_xy = _resample(t[:, 0], e[:, 0], e[:, 1:3])
    truth_xy = t[mask, 1:3]
    n = truth_xy.shape[0]
    if n < 3:
        return {"n": n, "note": "no temporal overlap between truth and estimate"}
    raw = np.linalg.norm(est_xy - truth_xy, axis=1)
    r, tr, yaw = rigid_align_2d(est_xy, truth_xy)
    aligned = (est_xy @ r.T) + tr
    err = np.linalg.norm(aligned - truth_xy, axis=1)
    return {
        "n": int(n),
        "rmse_m": float(np.sqrt(np.mean(err**2))),
        "mean_m": float(err.mean()),
        "median_m": float(np.median(err)),
        "max_m": float(err.max()),
        "std_m": float(err.std()),
        "raw_rmse_m": float(np.sqrt(np.mean(raw**2))),
        "alignment": {
            "yaw_deg": float(math.degrees(yaw)),
            "tx_m": float(tr[0]),
            "ty_m": float(tr[1]),
        },
    }


def path_length(traj) -> float:
    a = _arr(traj)
    if a.shape[0] < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(a[:, 1:3], axis=0), axis=1)))


# ---------------------------------------------------------------------------
# Truth-free metrics (no ground truth needed)
#
# The metrics above compare an estimate against a known-true series — available
# in the sim, where Gazebo publishes the robot's true pose. On a real recorded
# bag there is no truth, so these functions instead score *self-consistency*:
# how tight a loop closes, and how crisp the resulting map is. They are proxies,
# not error measures — see loop_drift/map_quality docstrings for what each can
# and cannot prove. Same numpy-only, ROS-free contract as everything else here.
# ---------------------------------------------------------------------------


def loop_drift(traj) -> dict:
    """Odometry self-consistency from a single estimated trajectory.

    ``traj`` is ``list[[t, x, y, yaw]]``. Reports the straight-line distance
    between the first and last pose and its ratio to the path travelled. On a
    bag where the robot physically returns to its start (a closed loop), a small
    ``start_end_dist_m`` means the estimator drifted little over the whole run;
    ``drift_ratio`` normalises that by path length so runs of different sizes
    compare. This is only meaningful when the trajectory is actually a loop —
    it cannot tell an open A→B traverse (legitimately large end distance) from a
    drifting loop, so the caller must know the bag's shape. Truth-free: it never
    sees a true pose, only the estimate's own consistency.
    """
    a = _arr(traj)
    if a.shape[0] < 2:
        return {"n": int(a.shape[0]), "note": "insufficient trajectory samples"}
    start, end = a[0, 1:3], a[-1, 1:3]
    dist = float(np.linalg.norm(end - start))
    plen = path_length(traj)
    return {
        "n": int(a.shape[0]),
        "start_end_dist_m": dist,
        "path_length_m": plen,
        "drift_ratio": float(dist / plen) if plen > 1e-6 else 0.0,
        "duration_s": float(a[-1, 0] - a[0, 0]),
    }


def _erode4(mask: np.ndarray) -> np.ndarray:
    """One 4-connectivity binary erosion, numpy-only (border cells erode away).
    A True cell survives only if its N/S/E/W neighbours are all True."""
    out = np.zeros_like(mask)
    out[1:-1, 1:-1] = (
        mask[1:-1, 1:-1]
        & mask[:-2, 1:-1]
        & mask[2:, 1:-1]
        & mask[1:-1, :-2]
        & mask[1:-1, 2:]
    )
    return out


def _occ_neighbor_count(mask: np.ndarray) -> np.ndarray:
    """Number of 4-connected True neighbours per cell (0..4), numpy-only."""
    n = np.zeros(mask.shape, dtype=np.int8)
    n[1:, :] += mask[:-1, :]
    n[:-1, :] += mask[1:, :]
    n[:, 1:] += mask[:, :-1]
    n[:, :-1] += mask[:, 1:]
    return n


def map_quality(
    grid, resolution: float, occ_thresh=65, free_thresh=25, max_iter=30
) -> dict:
    """Crispness/coverage proxies for a finished occupancy grid.

    ``grid`` is a 2-D array in ROS ``OccupancyGrid`` convention: 0 (free) .. 100
    (occupied), -1 unknown. ``resolution`` is metres per cell. No ground truth is
    used — these are structural proxies for "did the map come out clean":

    * ``unknown_frac`` / ``free_frac`` / ``occ_frac`` — cell mix. A more complete
      map has more decided (non-unknown) cells.
    * ``explored_area_m2`` — decided cells x cell area.
    * ``mean_wall_thickness_m`` — mean thickness of occupied structure, from
      iterated 4-connectivity erosion (each occupied cell's erosion depth is its
      distance to the nearest free/unknown cell; thickness ~= 2*depth+1). A
      double-walled or smeared map from a bad scan-match reads thicker than a
      crisp single-cell wall. Lower is crisper.
    * ``speckle_frac`` — fraction of occupied cells with no occupied neighbour,
      i.e. isolated specks. Poor odometry/scan-matching sprays these; lower is
      cleaner.

    Truth-free caveat: a crisp map is not necessarily a *correct* one — a
    confidently wrong map (e.g. a mis-closed loop drawn sharply) can still score
    well here. These proxies catch blur, incompleteness, and noise, not global
    metric error, which needs a surveyed reference the bag does not carry.
    """
    g = np.asarray(grid)
    total = int(g.size)
    if total == 0:
        return {"n_cells": 0, "note": "empty grid"}
    unknown = g < 0
    occ = g >= occ_thresh
    free = (g >= 0) & (g <= free_thresh)
    n_occ = int(occ.sum())

    thickness_m = 0.0
    if n_occ:
        depth_sum = 0
        m = occ.copy()
        for _ in range(max_iter):
            m = _erode4(m)
            s = int(m.sum())
            if s == 0:
                break
            depth_sum += s
        mean_depth = depth_sum / n_occ
        thickness_m = float((2.0 * mean_depth + 1.0) * resolution)

    speckle_frac = 0.0
    if n_occ:
        isolated = occ & (_occ_neighbor_count(occ) == 0)
        speckle_frac = float(int(isolated.sum()) / n_occ)

    decided = total - int(unknown.sum())
    return {
        "n_cells": total,
        "unknown_frac": float(int(unknown.sum()) / total),
        "free_frac": float(int(free.sum()) / total),
        "occ_frac": float(n_occ / total),
        "explored_area_m2": float(decided * resolution * resolution),
        "mean_wall_thickness_m": thickness_m,
        "speckle_frac": speckle_frac,
    }


def goal_stats(goals) -> dict:
    """Success rate and time-to-goal over the scripted goal sequence."""
    goals = list(goals or [])
    n = len(goals)
    ok = [g for g in goals if g.get("result") == "ok"]
    times = [float(g["duration"]) for g in ok if g.get("duration") is not None]
    out = {
        "n_goals": n,
        "n_succeeded": len(ok),
        "success_rate": (len(ok) / n) if n else 0.0,
        "results": {},
    }
    for g in goals:
        r = g.get("result", "unknown")
        out["results"][r] = out["results"].get(r, 0) + 1
    if times:
        out["time_to_goal_s"] = {
            "mean": float(np.mean(times)),
            "median": float(np.median(times)),
            "max": float(np.max(times)),
            "total": float(np.sum(times)),
        }
    return out


def clearance_stats(scan_min) -> dict:
    """Obstacle-clearance summary from per-scan nearest range."""
    a = _arr(scan_min)
    if a.shape[0] == 0:
        return {"n": 0}
    r = a[:, 1]
    r = r[np.isfinite(r)]
    if r.size == 0:
        return {"n": 0}
    out = {
        "n": int(r.size),
        "min_m": float(r.min()),
        "p05_m": float(np.percentile(r, 5)),
        "mean_m": float(r.mean()),
    }
    for band in CLEARANCE_BANDS:
        out[f"frac_below_{band:.2f}m"] = float(np.mean(r < band))
    return out


def _deriv(t, v):
    """Finite-difference derivative on irregular timestamps, dropping any step
    with non-positive dt (duplicate stamps, common at sim-clock resolution).
    Returns (t_mid, dv/dt)."""
    dt = np.diff(t)
    good = dt > 1e-6
    tm = 0.5 * (t[:-1] + t[1:])
    return tm[good], np.diff(v)[good] / dt[good]


def smoothness(cmd) -> dict:
    """Motion smoothness from commanded velocity: RMS linear/angular jerk and
    the number of forward/backward reversals (a proxy for indecisive planning).

    Jerk is the second time-derivative of velocity, taken as two successive
    finite differences over the irregular cmd_vel timestamps. Steps with a
    non-positive time delta are dropped so duplicate stamps can't inject NaNs.
    """
    a = _arr(cmd)
    if a.shape[0] < 3:
        return {"n": int(a.shape[0]), "note": "insufficient cmd_vel samples"}
    t, vx, wz = a[:, 0], a[:, 1], a[:, 2]

    def jerk_rms(v):
        ta, acc = _deriv(t, v)
        if ta.size < 2:
            return 0.0
        _, jrk = _deriv(ta, acc)
        return float(np.sqrt(np.mean(jrk**2))) if jrk.size else 0.0

    moving = np.abs(vx) > 0.02
    sign = np.sign(vx[moving])
    reversals = int(np.count_nonzero(np.diff(sign) != 0)) if sign.size > 1 else 0
    return {
        "n": int(a.shape[0]),
        "linear_jerk_rms": jerk_rms(vx),
        "angular_jerk_rms": jerk_rms(wz),
        "direction_reversals": reversals,
        "cmd_duration_s": float(t[-1] - t[0]),
    }


def summarize(series: dict) -> dict:
    """Run every metric over one recorded run. ``series`` carries the arrays
    described in the module docstring; unknown keys are ignored, missing keys
    degrade gracefully to empty metrics."""
    truth = series.get("truth", [])
    est = series.get("est", [])
    recoveries = dict(series.get("recoveries", {}) or {})
    recoveries.setdefault(
        "total", sum(v for k, v in recoveries.items() if k != "total")
    )
    return {
        "localization": ate(truth, est),
        "goals": goal_stats(series.get("goals", [])),
        "clearance": clearance_stats(series.get("scan_min", [])),
        "smoothness": smoothness(series.get("cmd", [])),
        "recoveries": recoveries,
        "aborts": int(
            sum(1 for g in series.get("goals", []) if g.get("result") == "aborted")
        ),
        "trajectory": {
            "truth_path_len_m": path_length(truth),
            "est_path_len_m": path_length(est),
            "duration_s": float(_arr(truth)[-1, 0] - _arr(truth)[0, 0])
            if _arr(truth).shape[0] >= 2
            else 0.0,
        },
    }


def _flatten(d: dict, prefix: str = "") -> dict:
    """Flatten a nested numeric dict to dotted scalar leaves (for aggregation)."""
    out = {}
    for k, v in d.items():
        key = f"{prefix}{k}"
        if isinstance(v, dict):
            out.update(_flatten(v, key + "."))
        elif isinstance(v, bool):
            continue
        elif isinstance(v, (int, float)):
            out[key] = float(v)
    return out


def aggregate(summaries: list) -> dict:
    """Reduce N per-trial summaries to mean/std/min/max/n per scalar metric, so
    run-to-run variance is explicit and two benchmark configs are comparable."""
    flats = [_flatten(s) for s in summaries]
    keys = sorted({k for f in flats for k in f})
    stats = {}
    for k in keys:
        vals = [f[k] for f in flats if k in f]
        if not vals:
            continue
        arr = np.asarray(vals, dtype=float)
        stats[k] = {
            "mean": float(arr.mean()),
            "std": float(arr.std()),
            "min": float(arr.min()),
            "max": float(arr.max()),
            "n": int(arr.size),
        }
    return {"n_trials": len(summaries), "metrics": stats}
