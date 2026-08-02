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
    # Angular structure is reported per map under "## Maps", not ranked here.
    # It is a *tear detector*, not a quality ordering: on the one real pair
    # available it prefers the leg with 16x the loop drift, because that leg
    # explored more and a larger map uses more wall directions. Ranking is loop
    # drift's job (and, where the trajectory does not close, nobody's) -- so
    # these are descriptive columns and bolding a winner among them would be a
    # claim the numbers do not support.
    ("angular support (°)", "map.angular_support_deg", 2, ""),
    ("wall frames (≥15% energy)", "map.n_strong_frames", 0, ""),
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
        "- **Angular structure** is a **tear detector, not a quality ranking**,"
        " and is deliberately not bolded. It answers the one question loop drift"
        " cannot: loop drift is only meaningful when the trajectory *closes*, so"
        " a session that exits on its exploration budget gets no drift number at"
        " all, and for those maps the frame table below is the only automated"
        " tear signal there is. Read it like this:",
        "",
        "  - **`wall frames` > 1 with real energy share means two rectangular"
        " systems in one map** — i.e. a section drawn on its own axes. That is"
        " what a SLAM tear looks like. Check the per-map frame table for the"
        " offset; run 3's two legs were torn by 22.5° and 41°.",
        "  - **One extra *direction* is architecture, not damage.** A flat with"
        " an angled hallway genuinely has three wall directions. The frame table"
        " distinguishes them: a rotated section duplicates a whole frame"
        " (`directions: 2`), a hallway adds one (`directions: 1`).",
        "  - **It is blind below ~10°**, the frame merge tolerance, which has to"
        " exceed the shear a genuine frame carries (7.5° measured on a real leg)"
        " or honest shear would read as a tear. A small rotation will show one"
        " frame. Catching that needs a declared direction set for the site,"
        " which `angular_stats(..., reference_directions=...)` accepts and this"
        " report does not yet supply.",
        "  - **`angular support` is confounded by coverage** and must not be"
        " used to rank: a map that explored less has fewer long walls and reads"
        " as tighter. On the 2026-07-29 run-3 pair the leg that is better by"
        " loop drift (0.551 m vs 8.776 m) scores *worse* on it (43.0 vs 37.7),"
        " having covered 59 m² against 81 m². It is here to be read beside"
        " `explored area`, not to pick a winner.",
        "- No absolute scale/position check is possible without a reference map or"
        " survey. For metric-accuracy claims, use the sim benchmark's ATE.",
        "- Replaying the same recorded sensor stream makes the comparison"
        " deterministic in its *input*, but SLAM's solver is not bit-exact"
        " run-to-run; treat small deltas as noise.",
        "",
    ]


def build_run(provenance, results) -> dict:
    return {"provenance": provenance, "results": results}
