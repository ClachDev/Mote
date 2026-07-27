#!/usr/bin/env python3
"""Sim benchmark harness: score nav missions against Gazebo ground truth.

Launches the *real* nav mission (`sim_launch.py mode:=nav`) headless for each
world, bridges the robot's true pose out of Gazebo, drives a scripted goal
sequence, and emits objective metrics — so Nav2/SLAM/localization changes can be
proven rather than eyeballed. One gz-sim instance runs at a time, so worlds and
trials run strictly sequentially.

Process management mirrors the smoke test and ``map_world.sh``: each launch runs
in its own session (``start_new_session=True`` == ``setsid``) so the whole
process group is SIGTERM'd then SIGKILL'd on teardown — force-killing the group
reaps slow-exiting Nav2 lifecycle nodes that would otherwise pile up across the
sequential trials of a parameter sweep — with a repo-scoped ``pkill`` backstop.
Readiness is gated on the launch log, not a fixed sleep. The per-trial ROS work
lives in ``record.py`` (run as a fresh subprocess per trial for a clean rclpy
context); metric maths lives in ``metrics.py`` (ROS-free, reused offline).

Each invocation claims a free ``ROS_DOMAIN_ID`` (and a matching ``GZ_PARTITION``)
unless one is inherited, so two benchmarks running at once on one machine cannot
see each other's graph — see ``pick_domain_id``. The sim pixi environment also
pins DDS discovery to this host (``ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST``),
so a run is invisible to the LAN.

    pixi run bench                                   # default worlds, 2 trials
    pixi run bench -- --worlds mote_world.sdf --trials 3
    pixi run bench -- --worlds mote_world.sdf,hospital_world.sdf
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

sys.path.insert(0, str(Path(__file__).resolve().parent))
import report  # noqa: E402

REPO = Path(__file__).resolve().parents[3]
BRINGUP = REPO / "mote_bringup"
SIM = REPO / "mote_simulation"
WORLDS = SIM / "worlds"
CONTROLLERS_READY = "Configured and activated diff_drive_controller"
PLUGIN_FAIL = "Failed to load system plugin"
# Domain 0 is the everything-else default; ROS 2 documents 0-101 as the range
# whose DDS ports stay clear of the Linux ephemeral port range.
DOMAIN_CANDIDATES = range(1, 102)
# Every DDS port for domain N falls in [7400 + 250*N, +250): multicast discovery
# at the base, per-participant unicast ports above it. Which of those are bound
# depends on the discovery mode — under ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
# CycloneDDS binds only the unicast ports — so the whole block is what tells us
# whether a domain is in use.
DDS_PORT_BASE, DDS_PORT_STEP = 7400, 250


def log(msg):
    print(f"[bench] {msg}", flush=True)


def bound_udp_ports():
    """Local UDP ports currently bound on this host, from /proc (no ss needed)."""
    ports = set()
    for path in ("/proc/net/udp", "/proc/net/udp6"):
        try:
            lines = Path(path).read_text().splitlines()[1:]
        except OSError:
            continue
        for line in lines:
            fields = line.split()
            if len(fields) > 1 and ":" in fields[1]:
                try:
                    ports.add(int(fields[1].split(":")[1], 16))
                except ValueError:
                    pass
    return ports


def pick_domain_id():
    """(domain_id, how) for this benchmark's ROS graph.

    An inherited ROS_DOMAIN_ID wins — the parameter sweep already assigns one per
    invocation, and a caller who set it meant it. Otherwise claim a domain whose
    DDS port block is unused on this host, so two benchmarks started at the same
    time (or a sim left running in another worktree) land on different graphs
    instead of silently sharing goals, /clock and TF. The port probe is a
    best-effort filter, not a lock: it cannot see a domain that is
    claimed-but-not-yet-running, so the candidate order is randomised to make
    that race unlikely.
    """
    inherited = os.environ.get("ROS_DOMAIN_ID", "").strip()
    if inherited:
        return int(inherited), "inherited"
    busy = bound_udp_ports()
    candidates = list(DOMAIN_CANDIDATES)
    random.shuffle(candidates)
    for domain in candidates:
        base = DDS_PORT_BASE + DDS_PORT_STEP * domain
        if not any(port in busy for port in range(base, base + DDS_PORT_STEP)):
            return domain, "claimed"
    return 0, "fallback (no free domain found)"


def popen_group(cmd, log_path):
    f = open(log_path, "wb")
    p = subprocess.Popen(
        cmd, stdout=f, stderr=subprocess.STDOUT, start_new_session=True
    )
    return p, f


def kill_group(p, sig=signal.SIGTERM):
    if p and p.poll() is None:
        try:
            os.killpg(os.getpgid(p.pid), sig)
        except (ProcessLookupError, PermissionError):
            pass


def wait_for_line(log_path, needle, sim_proc, timeout_s):
    """Poll ``log_path`` for ``needle``; fail fast on a plugin-load error or if
    the launch process exits. Returns (ok, reason)."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        text = (
            Path(log_path).read_text(errors="ignore") if Path(log_path).exists() else ""
        )
        if needle in text:
            return True, "ready"
        if PLUGIN_FAIL in text:
            return False, "gz_ros2_control plugin failed to load"
        if sim_proc.poll() is not None:
            return False, f"sim exited early (code {sim_proc.returncode})"
        time.sleep(2)
    return False, f"timed out waiting for '{needle}'"


def teardown(procs, files):
    # Graceful stop, then force-kill each launch's *whole* process group. Nav2
    # lifecycle nodes (controller_server, amcl, planner_server, ...) catch
    # SIGTERM and can outlive a short grace period; a name-matched pkill missed
    # them, so across many sequential trials (a parameter sweep) they piled up
    # and starved later runs until Nav2 bringup timed out. SIGKILL on the group
    # reaps them regardless of node name, and stays scoped to our own launches.
    for p in procs:
        kill_group(p, signal.SIGTERM)
    time.sleep(3)
    for p in procs:
        kill_group(p, signal.SIGKILL)
    # Backstop for a gz server that escaped its group, scoped to THIS repo's
    # world path so a benchmark never kills another worktree's concurrent sim.
    subprocess.run(["pkill", "-9", "-f", f"gz sim.*{REPO}"], stderr=subprocess.DEVNULL)
    subprocess.run(
        ["ros2", "daemon", "stop"], stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL
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


def map_provenance(world_stem):
    """(map_rev, map_yaml) for a world's committed sim site, via mote_bringup.sites
    under the sim MOTE_HOME. Returns ('unknown', path) if sites can't resolve."""
    try:
        from mote_bringup import sites

        fdir = sites.floor_dir(world_stem, "ground")
        rev = sites.current_revision(fdir) or "unknown"
        return rev, str(fdir / "map" / "map.yaml")
    except Exception as e:  # noqa: BLE001 - provenance must never abort a run
        return f"unresolved ({e})", ""


def run_trial(world, trial_dir, args):
    """Launch the nav sim + ground-truth bridge, run one recording trial, tear
    down. Returns the trial's metrics summary dict, or None on setup failure."""
    trial_dir.mkdir(parents=True, exist_ok=True)
    stem = world.removesuffix(".sdf")
    gt_topic = f"/model/{args.robot_name}/pose"
    sim_log = trial_dir / "sim.log"
    bridge_log = trial_dir / "bridge.log"
    procs, files = [], []

    subprocess.run(
        ["ros2", "daemon", "stop"], stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL
    )
    time.sleep(1)

    try:
        log(f"launching nav sim (world={world}, wheel_mu={args.wheel_mu})")
        sim_p, sim_f = popen_group(
            [
                "ros2",
                "launch",
                "mote_simulation",
                "sim_launch.py",
                "mode:=nav",
                f"world:={world}",
                f"wheel_mu:={args.wheel_mu}",
            ],
            sim_log,
        )
        procs.append(sim_p)
        files.append(sim_f)

        ok, reason = wait_for_line(sim_log, CONTROLLERS_READY, sim_p, args.boot_timeout)
        if not ok:
            log(f"FAIL: {reason}")
            return None
        log("controllers active; starting ground-truth bridge")

        bridge_p, bridge_f = popen_group(
            [
                "ros2",
                "run",
                "ros_gz_bridge",
                "parameter_bridge",
                f"{gt_topic}@geometry_msgs/msg/PoseStamped[gz.msgs.Pose",
            ],
            bridge_log,
        )
        procs.append(bridge_p)
        files.append(bridge_f)

        zones_file = WORLDS / f"{stem}.zones.yaml"
        rec = subprocess.run(
            [
                sys.executable,
                str(Path(__file__).with_name("record.py")),
                "--zones-file",
                str(zones_file),
                "--out-dir",
                str(trial_dir),
                "--gt-topic",
                gt_topic,
                "--order",
                args.order,
                "--goal-timeout",
                str(args.goal_timeout),
                "--settle",
                str(args.settle),
            ],
            timeout=args.trial_timeout,
        )
        if rec.returncode != 0:
            log(
                f"FAIL: record.py exited {rec.returncode} (see {trial_dir / 'error.txt'})"
            )
            return None
        return json.loads((trial_dir / "metrics.json").read_text())
    except subprocess.TimeoutExpired:
        log("FAIL: trial exceeded wall-clock timeout")
        return None
    finally:
        teardown(procs, files)
        time.sleep(2)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--worlds",
        default="mote_world.sdf,hospital_world.sdf",
        help="comma-separated world files in mote_simulation/worlds",
    )
    ap.add_argument("--trials", type=int, default=2)
    ap.add_argument(
        "--wheel-mu",
        type=float,
        default=1.0,
        help="drive-wheel friction (sim_launch wheel_mu:=); <1 induces slip",
    )
    ap.add_argument(
        "--slip",
        action="store_true",
        help="shorthand: set --wheel-mu 0.4 (slip condition) unless overridden",
    )
    ap.add_argument(
        "--order",
        default="pickup,dropoff,home",
        help="zone names to cycle as NavigateToPose goals",
    )
    ap.add_argument("--goal-timeout", type=float, default=120.0, help="sim s per goal")
    ap.add_argument(
        "--settle",
        type=float,
        default=8.0,
        help="sim s to settle localization/costmaps before goal 1",
    )
    ap.add_argument(
        "--robot-name",
        default="mote",
        help="gz model name of the robot (ground truth = /model/<name>/pose)",
    )
    ap.add_argument("--out", default=str(REPO / "benchmark_results"))
    ap.add_argument(
        "--boot-timeout",
        type=float,
        default=240.0,
        help="wall s to wait for controllers to activate",
    )
    ap.add_argument(
        "--trial-timeout",
        type=float,
        default=900.0,
        help="wall s hard cap for one recording trial",
    )
    args = ap.parse_args()

    domain, how = pick_domain_id()
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    # Gazebo transport has its own discovery, independent of DDS; partition it
    # too or two concurrent benchmarks would still share one gz graph.
    os.environ.setdefault("GZ_PARTITION", f"mote-bench-{domain}")
    log(
        f"ROS_DOMAIN_ID={domain} ({how}), "
        f"GZ_PARTITION={os.environ['GZ_PARTITION']}, "
        f"discovery range={os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE', 'default')}"
    )

    if args.slip and args.wheel_mu == 1.0:
        args.wheel_mu = 0.4

    worlds = [w.strip() for w in args.worlds.split(",") if w.strip()]
    ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = Path(args.out) / ts
    run_dir.mkdir(parents=True, exist_ok=True)
    log(f"results -> {run_dir}")

    provenance = {
        "timestamp": ts,
        "git_commit": git_commit(),
        "trials": args.trials,
        "order": args.order,
        "goal_timeout_s": args.goal_timeout,
        "nav2_params": str(BRINGUP / "config" / "nav2_params.yaml"),
        "worlds": worlds,
        "ros_domain_id": domain,
        "gz_partition": os.environ["GZ_PARTITION"],
        "wheel_mu": args.wheel_mu,
    }

    world_results = []
    for world in worlds:
        stem = world.removesuffix(".sdf")
        rev, map_yaml = map_provenance(stem)
        log(f"=== world {world} (map rev {rev}) ===")
        trials = []
        for i in range(args.trials):
            log(f"--- {stem} trial {i + 1}/{args.trials} ---")
            summary = run_trial(world, run_dir / stem / f"trial_{i}", args)
            if summary is not None:
                trials.append(summary)
        world_results.append(
            {"world": world, "map_rev": rev, "map_yaml": map_yaml, "trials": trials}
        )

    run = report.build_run(provenance, world_results)
    (run_dir / "run.json").write_text(json.dumps(run, indent=2))
    (run_dir / "report.md").write_text(report.build_markdown(run))
    log(f"wrote {run_dir / 'report.md'} and run.json")

    total = sum(len(w["trials"]) for w in world_results)
    if total == 0:
        log("no trials completed — check the per-trial sim.log")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
