"""Render the sweep ranking + winning-set provenance report (markdown).

Consumes the ranked set records the runner assembles and the committed default
values, and writes the human-readable evidence the tuning doc is built from:
which parameters changed (old vs new), and the metric deltas across the world
ladder.
"""

from __future__ import annotations

import score


def _fmt(v, nd=3):
    return "—" if v is None else f"{v:.{nd}f}"


def _pct(imp):
    return f"{100 * imp:+.1f}%"


def build_markdown(ranked, baseline, spec, provenance, defaults):
    """``ranked`` best-first; ``baseline`` is the index-0 record; ``defaults`` maps
    a winner assignment id -> committed value (for the old/new table)."""
    winner = next((r for r in ranked if r.get("is_winner")), None)
    floor = provenance.get("noise_floor", 0.0)
    lines = [
        f"# Parameter sweep report — {spec.name}",
        "",
        f"- **generated (UTC):** {provenance['timestamp']}",
        f"- **git commit:** `{provenance['git_commit']}`",
        f"- **spec:** `{provenance.get('spec', '')}`",
        f"- **worlds:** {', '.join(spec.worlds)}",
        f"- **trials/world:** {spec.trials}   **goal timeout:** {spec.goal_timeout:g}s",
        f"- **sets evaluated:** {len([r for r in ranked if r.get('ran')])} "
        f"of {len(ranked)}",
        "",
        "## Scoring",
        "",
        "Score is the world-weighted, weighted-metric improvement over the "
        "**baseline** (committed defaults), so baseline = 0 and any positive "
        "score beats the current config. Weights: "
        + ", ".join(
            f"{k} {v}"
            for k, v in {**score.DEFAULT_WEIGHTS, **(spec.weights or {})}.items()
        )
        + ". A set must be *feasible* (peak per-wheel speed within the "
        f"{provenance['wall_mps']:.3f} m/s hardware wall) and hold goal success "
        "at or above baseline to be eligible. To be declared the winner it must "
        f"beat the **noise floor** ({floor:+.3f}) by more than a "
        f"{score.WIN_MARGIN:+.2f} margin. The noise floor is the best score of any "
        "baseline-*replicate* set (same config as the defaults) — its non-zero "
        "score is pure run-to-run variance, so a real improvement must clear it.",
        "",
    ]

    if winner is None:
        lines += [
            "## Result: keep the current defaults",
            "",
            f"No set beat the noise floor ({floor:+.3f}) by more than the "
            f"{score.WIN_MARGIN:+.2f} win margin, so every apparent improvement is "
            "within run-to-run variance. The committed config is the best of those "
            "tried (or an improvement was infeasible or cost goal success). Details "
            "below.",
            "",
        ]
    else:
        lines += [
            f"## Winner: {winner['label']}",
            "",
            f"- **score:** {winner['score']['total']:+.3f} vs baseline",
            f"- **feasible:** peak per-wheel "
            f"{_fmt(winner['feasibility']['peak_wheel_mps'])} m/s "
            f"(wall {provenance['wall_mps']:.3f} m/s)",
            "",
            "### Changed parameters",
            "",
            "| parameter | file | key path | default | winner |",
            "| --- | --- | --- | --- | --- |",
        ]
        for a in winner["assignments"]:
            paths = "; ".join(".".join(kp) for kp in a["key_paths"])
            old = defaults.get(a["id"])
            lines.append(
                f"| {a['name']} | {a['target']} | `{paths}` | "
                f"{old} | **{a['value']}** |"
            )
        lines += ["", "### Metric deltas (winner vs baseline)", ""]
        lines += _delta_tables(winner, baseline)

    lines += ["## Full ranking", ""]
    lines += _ranking_table(ranked)
    lines += [
        "",
        "## How to read this",
        "",
        "- **success** = goal success rate, **ATE** = localization RMS error, "
        "**time** = mean time-to-goal, **jerk** = RMS linear jerk (smoothness).",
        "- **peak wheel** = worst commanded per-wheel speed; a set over the "
        "hardware wall is flagged infeasible and cannot win, even if it scores "
        "well in sim.",
        "- Per-metric % in the winner table is improvement over baseline "
        "(positive = better).",
        "",
    ]
    return "\n".join(lines)


def _delta_tables(winner, baseline):
    lines = []
    for world in sorted(winner["metrics"]):
        if world not in baseline["metrics"]:
            continue
        b = baseline["metrics"][world]
        c = winner["metrics"][world]
        imp = winner["score"]["per_metric"].get(world, {})
        lines += [
            f"**{world}**",
            "",
            "| metric | baseline | winner | improvement |",
            "| --- | --- | --- | --- |",
        ]
        rows = [
            ("success rate", "success", 3),
            ("ATE rmse (m)", "localization", 3),
            ("time-to-goal (s)", "time", 1),
            ("linear jerk rms", "smoothness", 2),
        ]
        for label, key, nd in rows:
            lines.append(
                f"| {label} | {_fmt(b.get(key), nd)} | {_fmt(c.get(key), nd)} | "
                f"{_pct(imp.get(key, 0.0))} |"
            )
        lines.append("")
    return lines


def _ranking_table(ranked):
    lines = [
        "| rank | set | score | eligible | success | ATE (m) | time (s) | "
        "peak wheel (m/s) |",
        "| --- | --- | --- | --- | --- | --- | --- | --- |",
    ]
    for i, r in enumerate(ranked):
        if not r.get("ran"):
            lines.append(f"| — | {r['label']} | (not run) | — | — | — | — | — |")
            continue
        succ = score.mean_success(r["metrics"])
        ate = _avg(r["metrics"], "localization")
        t = _avg(r["metrics"], "time", 1)
        flag = "yes" if r["eligible"] else "**no**"
        base = " (baseline)" if r["index"] == 0 else ""
        lines.append(
            f"| {i + 1} | {r['label']}{base} | {r['score']['total']:+.3f} | "
            f"{flag} | {_fmt(succ, 3)} | {ate} | {t} | "
            f"{_fmt(r['feasibility']['peak_wheel_mps'])} |"
        )
    return lines


def _avg(metrics, key, nd=3):
    vals = [m[key] for m in metrics.values() if m.get(key) is not None]
    return _fmt(sum(vals) / len(vals), nd) if vals else "—"
