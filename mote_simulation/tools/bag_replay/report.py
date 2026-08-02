"""Build the comparative bag-replay report: markdown metrics table + map images.

Truth-free by construction — a real bag carries no ground truth, so the report
scores self-consistency (loop drift) and map crispness (:mod:`metrics`
``map_quality``) side by side across parameter sets, and embeds each run's
rendered map so the numbers can be eyeballed. ROS-free: consumes the ``run``
dict that ``replay.py`` assembles.
"""

from __future__ import annotations


def _fmt(v, nd=3):
    if v is None or (isinstance(v, float) and v != v):
        return "—"
    if isinstance(v, float):
        return f"{v:.{nd}f}"
    return str(v)


def _get(d, dotted):
    cur = d
    for k in dotted.split("."):
        if not isinstance(cur, dict) or k not in cur:
            return None
        cur = cur[k]
    return cur


# (label, dotted metric key, decimals, "lower"/"higher" is better)
ROWS = [
    ("scans replayed", "n_scans", 0, ""),
    ("pose-graph nodes", "n_inserted", 0, ""),
    ("replay wall (s)", "wall_s", 0, ""),
    ("traj samples", "traj_samples", 0, ""),
    ("path length (m)", "loop.path_length_m", 2, ""),
    ("loop drift (m)", "loop.start_end_dist_m", 3, "lower"),
    ("drift ratio", "loop.drift_ratio", 4, "lower"),
    ("explored area (m²)", "map.explored_area_m2", 1, "higher"),
    ("unknown frac", "map.unknown_frac", 3, ""),
    ("occupied frac", "map.occ_frac", 4, ""),
    ("wall thickness (m)", "map.mean_wall_thickness_m", 3, "lower"),
    ("speckle frac", "map.speckle_frac", 4, "lower"),
]


def build_markdown(run) -> str:
    p = run["provenance"]
    results = run["results"]
    lines = [
        "# Bag-replay scoring report",
        "",
        f"- **generated (UTC):** {p['timestamp']}",
        f"- **git commit:** `{p['git_commit']}`",
        f"- **bag:** `{p['bag']}`",
        f"- **mode:** {p['mode']}  ·  **feed:** {p.get('feed', 'paced')}"
        + (
            f"  ·  **replay rate:** {p['rate']}× realtime"
            if p.get("feed") != "lockstep"
            else ""
        ),
        f"- **parameter sets:** {len(results)}",
        "",
        "## Metrics",
        "",
        "Truth-free proxies — see Limitations. Arrows mark the better direction.",
        "",
    ]
    header = "| metric | " + " | ".join(r["name"] for r in results) + " |"
    sep = "| --- | " + " | ".join("---" for _ in results) + " |"
    lines += [header, sep]
    for label, key, nd, better in ROWS:
        arrow = {"lower": " ↓", "higher": " ↑"}.get(better, "")
        vals = [_get(r["metrics"], key) for r in results]
        cells = [_mark(v, vals, better, nd) for v in vals]
        lines.append(f"| {label}{arrow} | " + " | ".join(cells) + " |")
    lines.append("")

    lines += ["## Maps", ""]
    for r in results:
        lines += [f"### {r['name']}", ""]
        if r.get("params_file"):
            lines.append(f"- params: `{r['params_file']}`")
        if r.get("map_png"):
            lines.append("")
            lines.append(f"![map for {r['name']}]({r['map_png']})")
        else:
            lines.append("- _no map produced_")
        lines.append("")

    lines += _limitations()
    return "\n".join(lines)


def _mark(v, vals, better, nd):
    """Bold the best value in a row when a direction is defined."""
    s = _fmt(v, nd)
    if not better or v is None:
        return s
    nums = [x for x in vals if isinstance(x, (int, float))]
    if len(nums) < 2:
        return s
    best = min(nums) if better == "lower" else max(nums)
    return f"**{s}**" if v == best else s


def _limitations() -> list:
    return [
        "## Limitations",
        "",
        "These metrics are **truth-free proxies**, not error measures. A real bag"
        " carries no surveyed ground truth, so unlike the sim benchmark (which has"
        " Gazebo's true pose and reports ATE), this harness can only score"
        " *self-consistency* and *map appearance*:",
        "",
        "- **Loop drift** is only meaningful when the robot physically returned to"
        " its start — it cannot distinguish a legitimate open A→B traverse from a"
        " drifting loop. Know the bag's shape before reading it.",
        "- **Map crispness** (wall thickness, speckle, unknown fraction) catches"
        " blur, noise, and incompleteness. It does **not** catch a confidently"
        " *wrong* map: a mis-closed loop drawn with sharp walls scores well here.",
        "- No absolute scale/position check is possible without a reference map or"
        " survey. For metric-accuracy claims, use the sim benchmark's ATE.",
        "- Replaying the same recorded sensor stream makes the comparison"
        " deterministic in its *input*, but SLAM's solver is not bit-exact"
        " run-to-run; treat small deltas as noise.",
        "- **Trajectory rows are only comparable within one feed mode.** A paced"
        " leg samples `map→base_link` off TF on a fixed period; a lockstep leg"
        " takes the pose graph's own node poses, because `map→odom` is broadcast"
        " on a wall-clock timer that a lockstep leg outruns. Path length and"
        " drift ratio therefore differ by *sampling*, not by quality — the map"
        " rows and `pose-graph nodes` are the ones that cross the two.",
        "",
    ]


def build_run(provenance, results) -> dict:
    return {"provenance": provenance, "results": results}
