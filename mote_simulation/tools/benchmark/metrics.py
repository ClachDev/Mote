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
