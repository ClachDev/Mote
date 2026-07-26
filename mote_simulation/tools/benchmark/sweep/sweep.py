#!/usr/bin/env python3
"""Parameter sweep on top of the sim benchmark harness.

Runs the benchmark once per parameter set (sequentially — one gz-sim instance at
a time), scores each set against the all-defaults baseline, and writes a ranking
plus a winning-set provenance report. Config overrides are applied at launch time
via merged temp files and the ``MOTE_*_PARAMS_FILE`` seam
(``mote_bringup.param_overrides``); the committed configs are never touched.

    pixi run bench-sweep mote_simulation/tools/benchmark/sweep/examples/office_nav.yaml
    pixi run bench-sweep <spec.yaml> --dry-run          # print the plan, launch nothing
    pixi run bench-sweep <spec.yaml> --max-sets 4       # cap sets (baseline + first N-1)
    pixi run bench-sweep <spec.yaml> --resume <run_dir> # finish an interrupted sweep

Outputs (under ``sweep_results/<UTC>/``, git-ignored):

    ranking.json    every set: assignments, metrics, feasibility, score, rank
    report.md       winning-set provenance: changed params + metric deltas
    set_<i>/        per-set benchmark run dir (bench.py's own report.md/run.json)
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import overrides  # noqa: E402
import score  # noqa: E402
import spec as spec_mod  # noqa: E402
import sweep_report  # noqa: E402

REPO = HERE.parents[3]
BENCH = HERE.parent / "bench.py"
ROBOT_YAML = REPO / "mote_description" / "config" / "robot.yaml"


def log(msg):
    print(f"[sweep] {msg}", flush=True)


def git_commit():
    r = subprocess.run(
        ["git", "rev-parse", "--short", "HEAD"],
        cwd=REPO,
        capture_output=True,
        text=True,
    )
    return r.stdout.strip() or "unknown"


def robot_wheel_params():
    cfg = yaml.safe_load(ROBOT_YAML.read_text())
    return float(cfg["wheel_separation"]), float(cfg["max_wheel_speed"])


def set_label(assignments):
    if not assignments:
        return "baseline"
    return ", ".join(p.label(v) for p, v in assignments)


def is_baseline_replicate(assignments):
    """True if a (non-empty) set assigns every parameter its committed default, so
    it re-runs the baseline config and its score is a pure run-to-run noise
    sample (see score.noise_floor)."""
    if not assignments:
        return False
    for param, value in assignments:
        if overrides.default_value(REPO, param.target, param.key_paths[0]) != value:
            return False
    return True


def assignments_json(assignments):
    return [
        {
            "id": f"{p.target}:{'|'.join('.'.join(kp) for kp in p.key_paths)}",
            "name": p.name,
            "target": p.target,
            "key_paths": p.key_paths,
            "value": v,
        }
        for p, v in assignments
    ]


def run_benchmark(spec, set_dir, env_overrides, domain_id):
    """Invoke bench.py for one parameter set; return its parsed run.json (or None
    on failure). ``env_overrides`` names the merged param files to apply;
    ``domain_id`` isolates the ROS graph on a dedicated DDS domain."""
    set_dir.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ)
    env.update(env_overrides)
    # Isolate the whole benchmark (sim + Nav2 + ground-truth bridge + recorder)
    # on a dedicated DDS domain, so a sim running in another worktree — or a live
    # robot on the network — can't pollute the graph and silently fail goals.
    env["ROS_DOMAIN_ID"] = str(domain_id)
    cmd = [
        sys.executable,
        str(BENCH),
        "--worlds",
        ",".join(spec.worlds),
        "--trials",
        str(spec.trials),
        "--order",
        spec.order,
        "--goal-timeout",
        str(spec.goal_timeout),
        "--settle",
        str(spec.settle),
        "--out",
        str(set_dir),
    ]
    log(f"running bench: worlds={spec.worlds} trials={spec.trials}")
    proc = subprocess.run(cmd, env=env)
    if proc.returncode != 0:
        log(f"bench.py exited {proc.returncode}")
    run_jsons = sorted(set_dir.rglob("run.json"))
    if not run_jsons:
        log("no run.json produced")
        return None, None
    run_json = run_jsons[-1]
    return json.loads(run_json.read_text()), run_json.parent


def existing_run(set_dir):
    """Return (run_json_dict, bench_dir) for a set already completed on disk, or
    (None, None). Only a run.json whose worlds carry aggregated trials counts as
    complete, so a partially-written run from an interrupted sweep is re-run
    rather than trusted."""
    for rj in sorted(Path(set_dir).rglob("run.json")):
        try:
            data = json.loads(rj.read_text())
        except (OSError, json.JSONDecodeError):
            continue
        worlds = data.get("worlds", [])
        if worlds and any(
            (w.get("aggregate") or {}).get("n_trials", 0) > 0 for w in worlds
        ):
            return data, rj.parent
    return None, None


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("spec", help="sweep spec YAML")
    ap.add_argument("--out", default=str(REPO / "sweep_results"))
    ap.add_argument(
        "--dry-run",
        action="store_true",
        help="expand the grid and write the merged param files, but launch no sim",
    )
    ap.add_argument(
        "--max-sets",
        type=int,
        default=0,
        help="cap the number of sets (baseline + first N-1); 0 = all",
    )
    ap.add_argument(
        "--ros-domain-id",
        type=int,
        default=42,
        help="DDS domain to isolate the benchmark graph on (avoids collisions "
        "with other sims/robots on the network)",
    )
    ap.add_argument(
        "--resume",
        default="",
        help="reuse an existing sweep dir: sets whose run.json already exists are "
        "loaded and skipped, the rest are run (a long sweep survives a restart)",
    )
    args = ap.parse_args()

    spec = spec_mod.load(args.spec)
    wheel_sep, wall = robot_wheel_params()
    grid = spec.grid()
    if args.max_sets and len(grid) > args.max_sets:
        log(f"capping {len(grid)} sets to {args.max_sets}")
        grid = grid[: args.max_sets]

    if args.resume:
        run_dir = Path(args.resume)
        if not run_dir.is_dir():
            log(f"--resume dir does not exist: {run_dir}")
            return 1
        ts = run_dir.name
        log(f"resuming {run_dir} (completed sets are reused)")
    else:
        ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        run_dir = Path(args.out) / ts
    run_dir.mkdir(parents=True, exist_ok=True)
    provenance = {
        "timestamp": ts,
        "git_commit": git_commit(),
        "spec": str(Path(args.spec).resolve()),
        "wall_mps": wall,
        "wheel_separation_m": wheel_sep,
        "ros_domain_id": args.ros_domain_id,
    }

    log(f"spec '{spec.name}': {spec.grid_size()} grid points + baseline")
    log(f"results -> {run_dir}")

    records = []
    for i, assignments in enumerate(grid):
        label = set_label(assignments)
        set_dir = run_dir / f"set_{i}"
        env_overrides = overrides.build_override_files(
            assignments, REPO, set_dir / "params"
        )
        log(f"=== set {i}/{len(grid) - 1}: {label} ===")
        rec = {
            "index": i,
            "label": label,
            "assignments": assignments_json(assignments),
            "override_files": env_overrides,
            "is_replicate": is_baseline_replicate(assignments),
            "ran": False,
            "metrics": {},
            "feasibility": {"feasible": True, "peak_wheel_mps": None},
        }
        if args.dry_run:
            log("dry-run: wrote merged params, skipping sim")
            records.append(rec)
            continue

        run_json, bench_dir = existing_run(set_dir)
        if run_json is not None:
            log(f"reusing existing result for set {i}")
        else:
            run_json, bench_dir = run_benchmark(
                spec, set_dir, env_overrides, args.ros_domain_id
            )
        if run_json is None:
            log(f"set {i} produced no metrics; leaving it unranked")
            records.append(rec)
            continue
        rec["ran"] = True
        rec["metrics"] = score.world_metrics(run_json)
        rec["feasibility"] = score.feasibility(bench_dir, wheel_sep, wall)
        records.append(rec)

    ran = [r for r in records if r["ran"]]
    if args.dry_run:
        _write_dry_run(run_dir, spec, provenance, records)
        log(f"dry-run plan written to {run_dir}")
        return 0
    if not ran:
        log("no set produced metrics — check the per-set bench logs")
        return 1

    ranked, floor = score.rank(ran, spec.weights, spec.world_weights)
    unran = [r for r in records if not r["ran"]]
    ordered = ranked + unran
    provenance["noise_floor"] = floor

    baseline = next(r for r in ran if r["index"] == 0)
    defaults = _winner_defaults(ordered)
    (run_dir / "ranking.json").write_text(
        json.dumps(
            {"provenance": provenance, "spec": spec.name, "sets": ordered}, indent=2
        )
    )
    (run_dir / "report.md").write_text(
        sweep_report.build_markdown(ordered, baseline, spec, provenance, defaults)
    )
    log(f"wrote {run_dir / 'report.md'} and ranking.json")

    winner = next((r for r in ordered if r.get("is_winner")), None)
    if winner:
        log(f"WINNER: {winner['label']}  score {winner['score']['total']:+.3f}")
    else:
        log(
            f"no set beat the noise floor ({floor:+.3f}) by the win margin — "
            "keep the defaults"
        )
    return 0


def _winner_defaults(ordered):
    """Committed default value for each assignment id, for the report's old/new
    table."""
    winner = next((r for r in ordered if r.get("is_winner")), None)
    if not winner:
        return {}
    out = {}
    for a in winner["assignments"]:
        out[a["id"]] = overrides.default_value(REPO, a["target"], a["key_paths"][0])
    return out


def _write_dry_run(run_dir, spec, provenance, records):
    (run_dir / "plan.json").write_text(
        json.dumps({"provenance": provenance, "sets": records}, indent=2)
    )
    lines = [
        f"# Sweep plan — {spec.name} (dry run)",
        "",
        f"- worlds: {', '.join(spec.worlds)}",
        f"- {spec.grid_size()} grid points + baseline = {len(records)} sets",
        "",
        "| # | set |",
        "| --- | --- |",
    ]
    for r in records:
        lines.append(f"| {r['index']} | {r['label']} |")
    (run_dir / "plan.md").write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    sys.exit(main())
