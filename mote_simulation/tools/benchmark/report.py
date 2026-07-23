"""Aggregate per-trial metrics into a summary JSON and a markdown report.

ROS-free: consumes the ``metrics.json`` summaries written by ``record.py`` (or
any producer of the same shape) and the run provenance recorded by ``bench.py``.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import metrics  # noqa: E402


def _fmt(v, unit="", nd=3):
    if v is None:
        return "—"
    return f"{v:.{nd}f}{unit}"


def _cv(stat):
    """Coefficient of variation (std/|mean|) as a percentage string, the headline
    number for run-to-run variance."""
    if stat["mean"] == 0 or stat["n"] < 2:
        return "—"
    return f"{100 * stat['std'] / abs(stat['mean']):.1f}%"


def _agg_row(label, stats, key, unit="", nd=3):
    s = stats.get(key)
    if not s:
        return None
    return (
        f"| {label} | {_fmt(s['mean'], unit, nd)} | {_fmt(s['std'], unit, nd)} | "
        f"{_fmt(s['min'], unit, nd)} | {_fmt(s['max'], unit, nd)} | {_cv(s)} |"
    )


def build_markdown(run) -> str:
    """``run`` is the dict bench.py assembles: provenance + per-world results,
    each world carrying its list of trial summaries and their aggregate."""
    p = run["provenance"]
    lines = [
        "# Sim benchmark report",
        "",
        f"- **generated (UTC):** {p['timestamp']}",
        f"- **git commit:** `{p['git_commit']}`",
        f"- **trials per world:** {p['trials']}",
        f"- **goal order:** {p['order']}",
        f"- **nav2 params:** `{p['nav2_params']}`",
        "",
    ]
    for world in run["worlds"]:
        lines += _world_section(world)
    lines += [
        "## Notes",
        "",
        "- Localization error is ATE (truth vs estimate) after a rigid SE(2)"
        " alignment — the SLAM `map` frame and the Gazebo world frame do not"
        " share a fixed transform, so alignment is required before differencing.",
        "- Ground truth is the robot's true pose bridged from Gazebo's"
        " PosePublisher (`/model/mote/pose`, `gz.msgs.Pose` → `PoseStamped`).",
        "- Recovery counts are distinct goal IDs seen on the behavior-server"
        " action status topics (`/spin`, `/backup`, `/drive_on_heading`,"
        " `/wait`) — a best-effort proxy for how often Nav2 recovered.",
        "- **CV** = coefficient of variation (std/mean); the run-to-run variance"
        " to weigh when comparing two configs.",
        "",
    ]
    return "\n".join(lines)


def _world_section(world) -> list:
    trials = world["trials"]
    agg = world["aggregate"]["metrics"]
    lines = [
        f"## {world['world']}",
        "",
        f"- map revision: `{world['map_rev']}`",
        f"- map: `{world['map_yaml']}`",
        f"- successful trials: {world['n_ok_trials']}/{len(trials)}",
        "",
        "| metric | mean | std | min | max | CV |",
        "| --- | --- | --- | --- | --- | --- |",
    ]
    rows = [
        _agg_row("goal success rate", agg, "goals.success_rate", "", 3),
        _agg_row("time-to-goal mean (s)", agg, "goals.time_to_goal_s.mean", "", 1),
        _agg_row("ATE rmse (m)", agg, "localization.rmse_m", "", 3),
        _agg_row("ATE max (m)", agg, "localization.max_m", "", 3),
        _agg_row("min clearance (m)", agg, "clearance.min_m", "", 3),
        _agg_row("mean clearance (m)", agg, "clearance.mean_m", "", 3),
        _agg_row("linear jerk rms", agg, "smoothness.linear_jerk_rms", "", 2),
        _agg_row("direction reversals", agg, "smoothness.direction_reversals", "", 1),
        _agg_row("recoveries total", agg, "recoveries.total", "", 1),
        _agg_row("aborts", agg, "aborts", "", 1),
        _agg_row("est path length (m)", agg, "trajectory.est_path_len_m", "", 2),
    ]
    lines += [r for r in rows if r]
    lines.append("")

    lines += ["<details><summary>per-trial</summary>", ""]
    lines += [
        "| trial | goals ok | ATE rmse (m) | min clr (m) | recoveries | aborts |",
        "| --- | --- | --- | --- | --- | --- |",
    ]
    for i, t in enumerate(trials):
        g = t["goals"]
        lines.append(
            f"| {i} | {g['n_succeeded']}/{g['n_goals']} | "
            f"{_fmt(t['localization'].get('rmse_m'))} | "
            f"{_fmt(t['clearance'].get('min_m'))} | "
            f"{t['recoveries'].get('total', 0)} | {t['aborts']} |"
        )
    lines += ["", "</details>", ""]
    return lines


def build_run(provenance, world_results) -> dict:
    """Assemble the full run object (JSON-serialisable) from per-world trial lists."""
    worlds = []
    for w in world_results:
        trials = w["trials"]
        worlds.append(
            {
                **{k: w[k] for k in ("world", "map_rev", "map_yaml")},
                "n_ok_trials": sum(
                    1 for t in trials if t["goals"]["success_rate"] == 1.0
                ),
                "trials": trials,
                "aggregate": metrics.aggregate(trials),
            }
        )
    return {"provenance": provenance, "worlds": worlds}
