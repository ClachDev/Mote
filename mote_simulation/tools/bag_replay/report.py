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
    ("traj samples", "traj_samples", 0, ""),
    ("path length (m)", "loop.path_length_m", 2, ""),
    ("loop drift (m)", "loop.start_end_dist_m", 3, "lower"),
    ("drift ratio", "loop.drift_ratio", 4, "lower"),
    ("explored area (m²)", "map.explored_area_m2", 1, "higher"),
    ("unknown frac", "map.unknown_frac", 3, ""),
    ("occupied frac", "map.occ_frac", 4, ""),
    ("wall thickness (m)", "map.mean_wall_thickness_m", 3, "lower"),
    ("speckle frac", "map.speckle_frac", 4, "lower"),
    ("angular support (°)", "map.angular_support_deg", 2, "lower"),
    ("angular entropy", "map.angular_entropy_norm", 4, "lower"),
    ("unassigned energy frac", "map.unassigned_energy_frac", 4, "lower"),
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
        f"- **mode:** {p['mode']}  ·  **replay rate:** {p['rate']}× realtime",
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
        lines += _angular_tables(r["metrics"])

    lines += _limitations()
    return "\n".join(lines)


def _angular_tables(m) -> list:
    """Per-map wall-direction and frame structure.

    Structure, not score: these describe *this* map's angular makeup and are not
    comparable across parameter sets the way the metrics table is, so they live
    here rather than in ``ROWS``. A building with an angled hallway genuinely has
    an extra direction; a drift-rotated section duplicates a whole orthogonal
    frame. That difference is what the frame table is for.
    """
    directions = _get(m, "map.directions")
    frames = _get(m, "map.frames")
    if not directions:
        return []

    lines = ["**Wall directions** (image frame, energy-weighted):", ""]
    lines.append("| angle (°) | energy frac | width (°) |")
    lines.append("| --- | --- | --- |")
    for d in directions:
        lines.append(
            f"| {_fmt(d.get('angle_deg'), 2)} | {_fmt(d.get('energy_frac'), 3)}"
            f" | {_fmt(d.get('width_deg'), 2)} |"
        )
    lines.append("")

    if frames:
        share = _get(m, "map.dominant_frame_share")
        lines.append(f"**Orthogonal frames** (dominant share {_fmt(share, 3)}):")
        lines.append("")
        lines.append(
            "| frame (°) | energy frac | directions | offset from dominant (°) |"
        )
        lines.append("| --- | --- | --- | --- |")
        for f in frames:
            lines.append(
                f"| {_fmt(f.get('angle_deg'), 2)} | {_fmt(f.get('energy_frac'), 3)}"
                f" | {_fmt(f.get('n_directions'), 0)}"
                f" | {_fmt(f.get('offset_from_dominant_deg'), 1)} |"
            )
        lines.append("")
    return lines


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
        "- **Angular coherence** (support, entropy, unassigned energy) scores how"
        " geometrically self-consistent the walls are, which is the one thing"
        " crispness misses: a drift-rotated section is crisp and unspeckled and"
        " still at the wrong angle. Four confounds bound how it should be read:",
        "  - **Coverage confounds it.** A map that explored less has fewer long"
        " walls and so uses fewer directions, which reads as *tighter*. On the"
        " 2026-07-29 run-3 pair the leg that is clearly better by loop drift"
        " (0.551 m vs 8.776 m) scores worse on angular support (42.0 vs 38.1)"
        " because it covered 59 m² against 81 m². Always read these beside"
        " `explored area`; never rank two sets on them at different coverage.",
        "  - **A multi-angle building is not a defect.** A flat with an angled"
        " hallway genuinely has three dominant wall directions and always will."
        " Higher support is the honest number for it, not a fault.",
        "  - **Within one map, a coherent rotated section is indistinguishable"
        " from real architecture** — both are just an extra wall family, and"
        " `unassigned energy frac` does not rise for either. Separating them"
        " needs a prior: a declared direction set for the site, or the same"
        " building's other legs. The scorer accepts one"
        " (`angular_stats(..., reference_directions=...)`) but this report does"
        " not yet supply it.",
        "  - **The frame table is a diagnostic, not a threshold.** Grouping"
        " directions into orthogonal frames needs a merge tolerance (10°) that"
        " must exceed the shear a genuine frame carries (7.5° measured) — the"
        " same order as the section rotations worth catching. It resolves a"
        " large tear (run 3's 23–38°); below roughly its own tolerance a rotated"
        " section merges back into the dominant frame and it will show one"
        " frame, not two. `n_peaks` is likewise threshold-bound and censored by"
        " the direction cap, so it is reported but not ranked.",
        "- No absolute scale/position check is possible without a reference map or"
        " survey. For metric-accuracy claims, use the sim benchmark's ATE.",
        "- Replaying the same recorded sensor stream makes the comparison"
        " deterministic in its *input*, but SLAM's solver is not bit-exact"
        " run-to-run; treat small deltas as noise.",
        "",
    ]


def build_run(provenance, results) -> dict:
    return {"provenance": provenance, "results": results}
