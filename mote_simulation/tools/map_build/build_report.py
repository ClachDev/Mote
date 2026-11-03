"""The build report: what was built, from what, and how it compares.

A candidate map is promoted by a human, and this is what they read before they
do it. Two rules shape it. Everything that decides the artifact — the bag's
digest, the parameter file's digest, the injected frame, the harness commit —
is printed, because "reproducible" means somebody can re-run this exact build.
And every number that is a *proxy* is labelled as one: the metrics here are
truth-free (the bag carries no ground truth), so they can say a map got
speckle-ier and they cannot say it got wrong.

Metric direction is stated, never inferred. A reader should not have to know
whether more ``explored_area_m2`` is good.
"""

from __future__ import annotations

import json
from pathlib import Path

#: The metrics carried into the baseline diff, with the direction that counts as
#: better and a one-line gloss. Adding a row is a claim that the metric ranks
#: one candidate against another, and two that ``map_quality`` reports are
#: deliberately **not** in it:
#:
#: * ``angular_support_deg`` is confounded by coverage — a map that explored
#:   less has fewer long walls and reads as tighter (bag_replay/README.md
#:   "Limitations").
#: * ``unknown_frac`` is a fraction of the *grid*, so it moves with the
#:   bounding box. Measured on the 2026-08-02 bag: a candidate 4 px wider than
#:   the baseline read 2.2% worse on it while covering 0.4 m² more floor, which
#:   is the opposite of what it appeared to say. ``explored_area_m2`` carries
#:   the same signal in metres and does not depend on the canvas.
#:
#: Everything ``map_quality`` measures is in ``build.json`` either way.
DIFFED = (
    ("loop.start_end_dist_m", "lower", "start↔end distance, if the run closed"),
    ("loop.drift_ratio", "lower", "that distance over path length"),
    ("map.mean_wall_thickness_m", "lower", "wall crispness; blur reads thicker"),
    ("map.speckle_frac", "lower", "isolated occupied cells"),
    ("map.explored_area_m2", "higher", "decided cells × cell area"),
)

#: Relative change below which a diff is reported as unchanged. The solver is
#: not bit-identical run to run, so a fraction of a percent on either side of a
#: proxy is noise; this is a legibility threshold and nothing gates on it.
DEADBAND = 0.02


def dig(source: dict, dotted: str):
    for key in dotted.split("."):
        if not isinstance(source, dict):
            return None
        source = source.get(key)
    return source


def compare(candidate: dict, baseline: dict | None) -> list[dict]:
    """One row per diffed metric: the two values, the change, and its direction."""
    rows = []
    for key, better, gloss in DIFFED:
        new = dig(candidate, key)
        if not isinstance(new, (int, float)):
            continue
        old = dig(baseline or {}, key)
        row = {"metric": key, "gloss": gloss, "better": better, "candidate": float(new)}
        if isinstance(old, (int, float)):
            row["baseline"] = float(old)
            row["delta"] = float(new) - float(old)
            scale = abs(float(old)) or 1.0
            relative = row["delta"] / scale
            if abs(relative) < DEADBAND:
                row["verdict"] = "same"
            elif (relative < 0) == (better == "lower"):
                row["verdict"] = "better"
            else:
                row["verdict"] = "worse"
            row["relative"] = relative
        rows.append(row)
    return rows


def _fmt(value) -> str:
    if value is None:
        return "—"
    if isinstance(value, float):
        return f"{value:.4g}"
    return str(value)


def _table(header: list[str], rows: list[list]) -> list[str]:
    out = ["| " + " | ".join(header) + " |", "|" + "---|" * len(header)]
    out += ["| " + " | ".join(_fmt(cell) for cell in row) + " |" for row in rows]
    return out


def build_markdown(build: dict) -> str:
    inputs = build["inputs"]
    lines = [
        f"# Map build {build['revision']}",
        "",
        f"**{build['verdict']}** — {build['verdict_detail']}",
        "",
        "## Inputs",
        "",
    ]
    lines += _table(
        ["input", "value"],
        [
            ["bag", inputs["bag"]["path"]],
            ["bag sha256", inputs["bag"]["sha256"]],
            ["bag bytes", sum(f["bytes"] for f in inputs["bag"]["files"])],
            ["slam params", inputs["params"]["path"]],
            ["params sha256", inputs["params"]["sha256"]],
            ["frame injection (x, y, yaw°)", inputs["frame"] or "none"],
            ["feed", inputs["feed"]],
            ["harness commit", inputs["harness_commit"]],
            ["built (UTC)", build["built"]],
        ],
    )

    lines += ["", "## Stages", ""]
    lines += _table(
        ["stage", "outcome", "detail"],
        [[s["name"], s["outcome"], s["detail"]] for s in build["stages"]],
    )

    report = build["validation"]
    lines += [
        "",
        "## Validation",
        "",
        f"`bundle.validate` — **{report['summary']}**",
        "",
    ]
    for error in report["errors"]:
        lines.append(f"- ERROR {error}")
    for warning in report["warnings"]:
        lines.append(f"- warning: {warning}")
    if not report["errors"] and not report["warnings"]:
        lines.append("- no errors, no warnings")

    lines += ["", "## Metrics", ""]
    baseline = build.get("baseline")
    if baseline:
        lines.append(f"Baseline: `{baseline['path']}`")
    else:
        lines.append(
            "No baseline given, so nothing is diffed — pass `--baseline` with the "
            "floor's current revision to compare against what is published."
        )
    lines += [
        "",
        "These are **truth-free proxies**: the bag carries no ground truth, so a "
        "confidently wrong map can score well. Read them beside the map.",
        "",
        "The `map.*` rows are the map this revision **serves** — after the "
        "declutter pass — on both sides, because that is what a promotion "
        "publishes. The raw solve's are in `build.json` under `map_raw`.",
        "",
    ]
    rows = [
        [
            row["metric"],
            row["candidate"],
            row.get("baseline"),
            row.get("delta"),
            row.get("verdict", "—"),
            f"{row['better']} is better — {row['gloss']}",
        ]
        for row in build["diff"]
    ]
    lines += _table(
        ["metric", "candidate", "baseline", "delta", "vs baseline", "reading"], rows
    )
    lines += [
        "",
        f"A change under {DEADBAND:.0%} reads as `same`: the solver is not "
        "bit-identical run to run. **Nothing here blocks** — a regression is "
        "evidence for the reviewer, not a gate.",
    ]

    angular = build.get("angular") or {}
    lines += ["", "## Wall structure", ""]
    if angular.get("frames"):
        lines += _table(
            ["frame", "angle (deg)", "directions", "energy share", "off dominant"],
            [
                [
                    index,
                    frame.get("angle_deg"),
                    frame.get("n_directions"),
                    frame.get("energy_frac"),
                    frame.get("offset_from_dominant_deg"),
                ]
                for index, frame in enumerate(angular["frames"])
            ],
        )
        lines.append("")
        lines.append(
            f"`angular_support_deg` {_fmt(angular.get('angular_support_deg'))}, "
            f"{angular.get('n_peaks')} wall direction(s), dominant frame share "
            f"{_fmt(angular.get('dominant_frame_share'))}. Support is **not** a "
            "quality ranking — a map that explored less has fewer long walls "
            "and reads as tighter."
        )
        lines += [
            "",
            "A rectilinear building puts every wall in one frame. A second "
            "frame carrying real energy with **two** directions in it means a "
            "section of the map is drawn on its own axes — a tear. A second "
            "frame with one direction is an angled hallway, which is "
            "architecture.",
        ]
    else:
        lines.append("No angular structure was measured.")
    lines += [
        "",
        "The build does **not** align the map frame. Measuring a map's wall "
        "rotation well enough to gate a re-solve on it is task 615 "
        "(`docs/tuning/2026-09-01-alignment-residual.md`): the estimator in "
        "the tree called four maps square that were 3.5–5.6° out. Until it "
        "lands, birth-alignment is an operator's judgment, passed as "
        "`--frame X Y YAW`, and recorded above.",
    ]

    zones = build.get("zones") or {}
    lines += ["", "## Zones", ""]
    lines.append(
        f"Segmentation proposed {len(zones.get('added', []))} room(s): "
        + (", ".join(f"`{name}`" for name in zones.get("added", [])) or "none")
    )
    if zones.get("carry_forward"):
        lines += ["", zones["carry_forward"]]

    if build.get("images"):
        lines += ["", "## Renders", ""]
        for caption, path in build["images"]:
            lines.append(f"### {caption}\n\n![{caption}]({path})\n")

    lines += ["", "## Next", "", build["next"], ""]
    return "\n".join(lines) + "\n"


def write(out_dir, build: dict) -> Path:
    out_dir = Path(out_dir)
    (out_dir / "build.json").write_text(json.dumps(build, indent=2))
    path = out_dir / "report.md"
    path.write_text(build_markdown(build))
    return path
