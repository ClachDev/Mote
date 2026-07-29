#!/usr/bin/env python3
"""Bag-replay scoring harness: validate parameter choices against real recordings.

The reality check that complements the sim benchmark. The sim proves parameters
against *simulated* sensors with Gazebo ground truth; this replays a *real*
recorded mapping bag through SLAM (or kinematic_icp) under two or more parameter
sets and scores the results with truth-free proxies — no ground truth needed —
then writes a side-by-side comparison report with a metrics table and rendered
map images.

    pixi run bag-replay -- --bag ~/.mote/bags/mapping/<ts> \\
        --params a.yaml b.yaml

Each parameter set is replayed in its own freshly-launched stack under a random
``ROS_DOMAIN_ID`` (so a stray node can never reach a live robot, and sequential
runs can't leak into each other — the sim sweep learned that lesson). The
per-replay ROS work lives in ``replayer.py`` (fresh rclpy context per set); the
metric maths is the sim benchmark's ROS-free ``metrics`` module, reused not
forked; rendering and the report are local siblings.

The stack is launched from ``replay_stack_launch.py`` (env packages only), so the
harness runs against a bag without the workspace being built.
"""

from __future__ import annotations

import argparse
import json
import os
import random
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
BRINGUP = REPO / "mote_bringup"
DEFAULT_SLAM_PARAMS = BRINGUP / "config" / "slam_toolbox_params.yaml"
STACK_LAUNCH = HERE / "replay_stack_launch.py"

sys.path.insert(0, str(REPO / "mote_simulation" / "tools" / "benchmark"))
import metrics  # noqa: E402

sys.path.insert(0, str(HERE))
import render  # noqa: E402
import report  # noqa: E402

# Readiness needle in the launch log, per mode. slam autostarts via
# nav2_lifecycle_manager, which announces activation; icp has no lifecycle.
READY_NEEDLE = {"slam": "Managed nodes are active", "icp": None}
KILL_NAMES = (
    "async_slam_toolbox_node",
    "kinematic_icp_online_node",
    "lifecycle_manager",
)


def log(msg):
    print(f"[bag-replay] {msg}", flush=True)


def popen_group(cmd, log_path, env):
    f = open(log_path, "wb")
    p = subprocess.Popen(
        cmd, stdout=f, stderr=subprocess.STDOUT, start_new_session=True, env=env
    )
    return p, f


def kill_group(p):
    if p and p.poll() is None:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
        except (ProcessLookupError, PermissionError):
            pass


def wait_for_ready(log_path, needle, proc, timeout_s, min_wait=5.0):
    """Block until ``needle`` appears (or a fixed settle if ``needle`` is None),
    failing fast if the launch process dies. Returns (ok, reason)."""
    deadline = time.monotonic() + timeout_s
    started = time.monotonic()
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            return False, f"stack exited early (code {proc.returncode})"
        text = (
            Path(log_path).read_text(errors="ignore") if Path(log_path).exists() else ""
        )
        if needle is None:
            if time.monotonic() - started >= min_wait:
                return True, "settled"
        elif needle in text:
            return True, "ready"
        time.sleep(1)
    return False, f"timed out waiting for '{needle}'"


def teardown(procs, files, env):
    for p in procs:
        kill_group(p)
    time.sleep(2)
    for name in KILL_NAMES:
        subprocess.run(["pkill", "-9", "-f", name], stderr=subprocess.DEVNULL, env=env)
    subprocess.run(
        ["ros2", "daemon", "stop"],
        stderr=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        env=env,
    )
    for f in files:
        try:
            f.close()
        except OSError:
            pass


def git_commit():
    r = subprocess.run(
        ["git", "rev-parse", "--short", "HEAD"],
        cwd=REPO,
        capture_output=True,
        text=True,
    )
    return r.stdout.strip() or "unknown"


def isolated_env():
    """A child env pinned to a random high ROS_DOMAIN_ID so a stray node can
    never touch a live robot and sequential replays can't leak into each other."""
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(random.randint(1, 232))
    return env


def score(series_path, map_npz):
    """Compute truth-free metrics for one replay from its recorded outputs."""
    series = json.loads(Path(series_path).read_text())
    traj = series.get("traj", [])
    out = {
        "n_scans": series.get("n_scans", 0),
        "traj_samples": len(traj),
        "loop": metrics.loop_drift(traj),
    }
    if map_npz and Path(map_npz).exists():
        import numpy as np

        data = np.load(map_npz)
        out["map"] = metrics.map_quality(data["grid"], float(data["resolution"]))
    return out, traj


def run_one(bag, params_file, name, mode, run_dir, args):
    """Launch the stack for one parameter set, replay the bag, score, render."""
    set_dir = run_dir / name
    set_dir.mkdir(parents=True, exist_ok=True)
    env = isolated_env()
    stack_log = set_dir / "stack.log"
    procs, files = [], []

    subprocess.run(
        ["ros2", "daemon", "stop"],
        stderr=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        env=env,
    )
    time.sleep(1)
    try:
        log(f"[{name}] launching {mode} stack (domain {env['ROS_DOMAIN_ID']})")
        cmd = [
            "ros2",
            "launch",
            str(STACK_LAUNCH),
            f"mode:={mode}",
            "use_sim_time:=true",
        ]
        if mode == "slam":
            cmd.append(f"slam_params_file:={params_file}")
        stack_p, stack_f = popen_group(cmd, stack_log, env)
        procs.append(stack_p)
        files.append(stack_f)

        ok, reason = wait_for_ready(
            stack_log, READY_NEEDLE[mode], stack_p, args.boot_timeout
        )
        if not ok:
            log(f"[{name}] FAIL: {reason}")
            return None
        log(f"[{name}] stack {reason}; replaying bag")

        rp = subprocess.run(
            [
                sys.executable,
                str(HERE / "replayer.py"),
                "--bag",
                str(bag),
                "--mode",
                mode,
                "--out-dir",
                str(set_dir),
                "--rate",
                str(args.rate),
                "--settle",
                str(args.settle),
                "--max-scans",
                str(args.max_scans),
                "--skip-secs",
                str(args.skip_secs),
                "--stop-secs",
                str(args.stop_secs),
                *(["--frame", *(str(v) for v in args.frame)] if args.frame else []),
            ],
            env=env,
            timeout=args.replay_timeout,
        )
        if rp.returncode != 0:
            log(f"[{name}] FAIL: replayer exited {rp.returncode}")
            return None
    except subprocess.TimeoutExpired:
        log(f"[{name}] FAIL: replay exceeded wall-clock timeout")
        return None
    finally:
        teardown(procs, files, env)
        time.sleep(2)

    map_npz = set_dir / "map.npz"
    m, traj = score(set_dir / "series.json", map_npz)
    map_png = None
    if map_npz.exists():
        rel = f"{name}/map.png"
        render.render_map(map_npz, run_dir / rel, traj=traj)
        map_png = rel
    return {
        "name": name,
        "params_file": str(params_file) if mode == "slam" else None,
        "metrics": m,
        "map_png": map_png,
        "series_json": str((set_dir / "series.json").relative_to(run_dir)),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--bag", required=True, help="path to a recorded mapping bag dir")
    ap.add_argument(
        "--params",
        nargs="*",
        default=[],
        help="slam_toolbox param YAMLs, one per set (slam mode). "
        "Default: the committed slam_toolbox_params.yaml as a single baseline.",
    )
    ap.add_argument("--mode", choices=("slam", "icp"), default="slam")
    ap.add_argument("--rate", type=float, default=1.0, help="replay speed x realtime")
    ap.add_argument("--settle", type=float, default=8.0)
    ap.add_argument("--max-scans", type=int, default=0, help="0 = whole bag")
    ap.add_argument(
        "--skip-secs", type=float, default=0.0, help="withhold scans before this"
    )
    ap.add_argument(
        "--stop-secs", type=float, default=0.0, help="stop feeding at this (0=end)"
    )
    ap.add_argument(
        "--frame",
        nargs=3,
        type=float,
        metavar=("X", "Y", "YAW_DEG"),
        help="SE2 applied to the odometry prior (birth-align the map frame)",
    )
    ap.add_argument("--out", default=str(REPO / "bag_replay_results"))
    ap.add_argument("--boot-timeout", type=float, default=120.0)
    ap.add_argument("--replay-timeout", type=float, default=3600.0)
    args = ap.parse_args()

    bag = Path(args.bag).expanduser()
    if not bag.exists():
        sys.exit(f"bag not found: {bag}")

    if args.mode == "slam":
        param_files = [Path(p).expanduser() for p in args.params] or [
            DEFAULT_SLAM_PARAMS
        ]
        for p in param_files:
            if not p.exists():
                sys.exit(f"param file not found: {p}")
        sets = [(p.stem, p) for p in param_files]
    else:
        # icp params are inline in the launch; a set is just a repeat label.
        labels = args.params or ["icp"]
        sets = [(str(lbl), None) for lbl in labels]
    # Disambiguate duplicate names (e.g. two files both named slam_toolbox_params).
    seen = {}
    uniq = []
    for name, pf in sets:
        seen[name] = seen.get(name, 0) + 1
        uniq.append((f"{name}_{seen[name]}" if seen[name] > 1 else name, pf))
    sets = uniq

    ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = Path(args.out) / ts
    run_dir.mkdir(parents=True, exist_ok=True)
    log(f"results -> {run_dir}")
    log(f"bag: {bag}  mode: {args.mode}  sets: {[n for n, _ in sets]}")

    provenance = {
        "timestamp": ts,
        "git_commit": git_commit(),
        "bag": str(bag),
        "mode": args.mode,
        "rate": args.rate,
    }

    results = []
    for name, pf in sets:
        log(f"=== parameter set: {name} ===")
        r = run_one(bag, pf, name, args.mode, run_dir, args)
        if r is not None:
            results.append(r)

    if not results:
        log("no replays completed — check per-set stack.log / replay.log")
        return 1

    run = report.build_run(provenance, results)
    (run_dir / "run.json").write_text(json.dumps(run, indent=2))
    (run_dir / "report.md").write_text(report.build_markdown(run))
    log(f"wrote {run_dir / 'report.md'} and run.json ({len(results)} set(s))")
    return 0


if __name__ == "__main__":
    sys.exit(main())
