"""Score and rank parameter sets from their benchmark ``run.json`` outputs.

ROS-free. Scoring is **relative to the baseline** (the all-defaults set the
sweep always runs first), so a set's score is how much better or worse it is
than the committed config on the same worlds — the number that answers "should
we change anything?".

Scoring function
----------------
Four metrics, each read from a world's aggregate (mean across that world's
trials):

===============  ==============================  =========  ===============
metric           run.json key                    direction  default weight
===============  ==============================  =========  ===============
``success``      ``goals.success_rate``          higher      3.0
``localization`` ``localization.rmse_m``         lower       1.0
``time``         ``goals.time_to_goal_s.mean``   lower       1.0
``smoothness``   ``smoothness.linear_jerk_rms``  lower       0.5
===============  ==============================  =========  ===============

For each metric the *relative improvement* of candidate value ``v`` over
baseline ``b`` is::

    higher-is-better:  (v - b) / denom
    lower-is-better:   (b - v) / denom
    denom = |b| if |b| > 1e-9 else max(|v|, 1e-9)

Positive means better than default. The per-world score is the weighted sum of
the improvements; the total is the world-weighted mean of the per-world scores
(worlds equal-weighted unless the spec says otherwise). The baseline scores 0 by
construction, so any positive total beats the committed config.

Success is weighted highest so a set can never "win" by trading goal completions
for speed. Two hard gates back that up: the winner must be **feasible** (see
below) and must not drop mean success below the baseline (minus a small
tolerance).

Feasibility gate
----------------
The sim's differential drive will happily command wheel speeds the real STS3215
servos cannot reach, so a set could look fast in sim yet be undriveable. For
every recorded ``cmd`` sample the peak per-wheel speed is::

    v_wheel = |v_x| + |w_z| * wheel_separation / 2

If a set's peak exceeds the hardware wall (``robot.yaml`` ``max_wheel_speed`` =
0.218 m/s) by more than a tolerance, the set is marked infeasible and can never
be selected, regardless of its metric score.
"""

from __future__ import annotations

import json
from pathlib import Path

EPS = 1e-9

# metric name -> (dotted aggregate key, higher_is_better)
METRIC_SPECS = {
    "success": ("goals.success_rate", True),
    "localization": ("localization.rmse_m", False),
    "time": ("goals.time_to_goal_s.mean", False),
    "smoothness": ("smoothness.linear_jerk_rms", False),
}

DEFAULT_WEIGHTS = {
    "success": 3.0,
    "localization": 1.0,
    "time": 1.0,
    "smoothness": 0.5,
}

# A set may complete fewer goals than baseline by at most this (mean success
# fraction) and still be eligible to win.
SUCCESS_GATE_TOL = 0.01
# Allow this fractional slack above the wheel-speed wall before disqualifying.
FEASIBILITY_TOL = 0.05


def world_metrics(run):
    """``run.json`` dict -> ``{world: {metric_name: value_or_None}}`` using each
    world's aggregate mean."""
    out = {}
    for w in run.get("worlds", []):
        agg = (w.get("aggregate") or {}).get("metrics", {})
        vals = {}
        for name, (key, _hib) in METRIC_SPECS.items():
            stat = agg.get(key)
            vals[name] = stat["mean"] if stat and "mean" in stat else None
        out[w["world"]] = vals
    return out


def _improvement(value, baseline, higher_is_better):
    if value is None or baseline is None:
        return 0.0
    denom = abs(baseline) if abs(baseline) > EPS else max(abs(value), EPS)
    delta = (value - baseline) if higher_is_better else (baseline - value)
    return delta / denom


def score_set(candidate, baseline, weights=None, world_weights=None):
    """Score ``candidate`` world-metrics against ``baseline`` world-metrics.

    Returns a dict with the total score, per-world scores, and the per-metric
    improvements (for the report). Worlds present in both are compared; a world
    missing from either is skipped.
    """
    weights = {**DEFAULT_WEIGHTS, **(weights or {})}
    shared = [w for w in candidate if w in baseline]
    per_world = {}
    per_metric = {}
    for w in shared:
        ww = (world_weights or {}).get(w, 1.0)
        s = 0.0
        per_metric[w] = {}
        for name, (_key, hib) in METRIC_SPECS.items():
            imp = _improvement(candidate[w].get(name), baseline[w].get(name), hib)
            per_metric[w][name] = imp
            s += weights[name] * imp
        per_world[w] = {"score": s, "weight": ww}
    tw = sum(v["weight"] for v in per_world.values())
    total = (
        sum(v["score"] * v["weight"] for v in per_world.values()) / tw if tw else 0.0
    )
    return {"total": total, "per_world": per_world, "per_metric": per_metric}


def peak_wheel_speed(run_dir, wheel_separation):
    """Max per-wheel speed (m/s) over every ``series.json`` under ``run_dir``,
    or None if no cmd samples were recorded."""
    peak = None
    for series in Path(run_dir).rglob("series.json"):
        try:
            data = json.loads(series.read_text())
        except (OSError, json.JSONDecodeError):
            continue
        for sample in data.get("cmd", []):
            if len(sample) < 3:
                continue
            _t, vx, wz = sample[0], sample[1], sample[2]
            v = abs(vx) + abs(wz) * wheel_separation / 2.0
            if peak is None or v > peak:
                peak = v
    return peak


def feasibility(run_dir, wheel_separation, wall_mps, tol=FEASIBILITY_TOL):
    peak = peak_wheel_speed(run_dir, wheel_separation)
    limit = wall_mps * (1.0 + tol)
    feasible = peak is None or peak <= limit
    return {
        "feasible": bool(feasible),
        "peak_wheel_mps": peak,
        "wall_mps": wall_mps,
        "limit_mps": limit,
    }


def mean_success(world_metrics_dict):
    vals = [
        m["success"]
        for m in world_metrics_dict.values()
        if m.get("success") is not None
    ]
    return sum(vals) / len(vals) if vals else 0.0


def rank(sets, weights=None, world_weights=None):
    """Score and order a list of set records against the baseline (index 0).

    Each record is a dict with at least ``index``, ``metrics`` (world-metrics
    dict), and ``feasibility``. Adds ``score`` and ``eligible`` in place and
    returns the list ordered best-first: eligible sets by descending score, then
    ineligible ones. The baseline is always eligible (it is the reference).
    """
    baseline = sets[0]
    base_metrics = baseline["metrics"]
    base_success = mean_success(base_metrics)
    for rec in sets:
        rec["score"] = score_set(rec["metrics"], base_metrics, weights, world_weights)
        feasible = rec["feasibility"]["feasible"]
        success_ok = mean_success(rec["metrics"]) >= base_success - SUCCESS_GATE_TOL
        rec["eligible"] = bool(feasible and (success_ok or rec["index"] == 0))
    ordered = sorted(
        sets,
        key=lambda r: (not r["eligible"], -r["score"]["total"]),
    )
    return ordered
