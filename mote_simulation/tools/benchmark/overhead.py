#!/usr/bin/env python3
"""Sample the OS-level cost of the running nav stack.

``bench.py`` scores what navigation *achieves*; this scores what it *costs* the
machine to achieve it — process count, CPU, resident memory, and the system-wide
context-switch and interrupt rates that a many-process architecture drives up.
Run it alongside a benchmark:

    python overhead.py --duration 900 --out /tmp/before &
    pixi run bench -- --worlds office_world.sdf --trials 3

Two sources, one clock. Per-process figures come from ``/proc/<pid>/stat`` (CPU
as a delta of utime+stime over the interval, so the first row is a warm-up and
is dropped) and ``/proc/<pid>/statm`` — ``pidstat`` is not installed on either
the workstation or the Pi, and /proc needs nothing beyond stdlib. System-wide
figures come from ``/proc/stat``'s ``ctxt`` and ``intr`` counters, the same ones
``vmstat`` reports, differenced over the same interval.

Processes are matched on the basename of ``/proc/<pid>/exe`` — what the process
*is*, not what its command line says — so this can never match the sampler, the
``pixi run`` wrapper, or an editor holding a file open. ``--names`` defaults to
the union of the standalone-server and composed-container names, so the same
invocation measures both sides of a composition change. The match is also
confined to executables under this checkout (``--exe-prefix``, default: the
repo root): a workstation running several worktrees — or holding a leaked stack
from an earlier run — would otherwise have its other Nav2 processes counted as
this one's.

Every row is kept, including the ones where nothing is running, so a single
sampling session covering a whole benchmark yields both the loaded figures and
the machine's idle baseline. That baseline is what makes the system-wide
counters comparable: ``ctxt_per_s`` includes every other process on the box, so
only the rise above idle is attributable to the nav stack.

``--summary`` re-reads a completed run's CSV and prints the table without
sampling, which is what the report quotes.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import statistics
import sys
import time
from pathlib import Path

# The nav stack under either architecture: the ten standalone Nav2 servers, and
# the component container they collapse into. Matching both means one command
# measures a before/after pair without being told which world it is in.
DEFAULT_NAMES = [
    "map_server",
    "amcl",
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
    "lifecycle_manager",
    "component_container",
    "component_container_mt",
    "component_container_isolated",
]

CLOCK_TICKS = os.sysconf("SC_CLK_TCK")
PAGE_SIZE = os.sysconf("SC_PAGE_SIZE")
REPO = Path(__file__).resolve().parents[3]


def matching_pids(names, exe_prefix):
    """PIDs whose executable basename is in ``names`` and lives under ``exe_prefix``."""
    found = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            exe = os.readlink(entry / "exe")
        except OSError:
            continue
        if os.path.basename(exe) in names and exe.startswith(exe_prefix):
            found.append(int(entry.name))
    return found


def proc_cpu_ticks(pid):
    """utime+stime in clock ticks, or None if the process is gone."""
    try:
        # comm can contain spaces and parentheses, so split after the last ')'.
        raw = Path(f"/proc/{pid}/stat").read_text()
        fields = raw[raw.rindex(")") + 2 :].split()
    except (OSError, ValueError):
        return None
    # fields[0] is state (field 3 in proc(5)); utime/stime are fields 14/15.
    return int(fields[11]) + int(fields[12])


def proc_rss_bytes(pid):
    try:
        return int(Path(f"/proc/{pid}/statm").read_text().split()[1]) * PAGE_SIZE
    except (OSError, IndexError, ValueError):
        return 0


def system_counters():
    """(context switches, interrupts) since boot — vmstat's cs and in columns."""
    ctxt = intr = 0
    for line in Path("/proc/stat").read_text().splitlines():
        if line.startswith("ctxt "):
            ctxt = int(line.split()[1])
        elif line.startswith("intr "):
            intr = int(line.split()[1])
    return ctxt, intr


def sample(names, exe_prefix, prev_ticks):
    """One observation. Returns (row-without-rates, cpu-ticks-by-pid)."""
    pids = matching_pids(names, exe_prefix)
    ticks = {}
    rss = 0
    for pid in pids:
        t = proc_cpu_ticks(pid)
        if t is None:
            continue
        ticks[pid] = t
        rss += proc_rss_bytes(pid)
    # Only pids seen in both samples contribute CPU; a server that respawned
    # mid-interval would otherwise read as a huge negative delta.
    used = sum(
        ticks[pid] - prev_ticks[pid] for pid in ticks if pid in prev_ticks
    )
    return {"procs": len(pids), "rss_mb": rss / 1e6, "cpu_ticks": used}, ticks


def run(args):
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / "overhead.csv"
    names = set(args.names.split(",")) if args.names else set(DEFAULT_NAMES)

    _, prev_ticks = sample(names, args.exe_prefix, {})
    prev_ctxt, prev_intr = system_counters()
    prev_t = time.monotonic()
    deadline = prev_t + args.duration

    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            ["t_s", "procs", "cpu_pct", "rss_mb", "ctxt_per_s", "intr_per_s"]
        )
        start = prev_t
        while time.monotonic() < deadline:
            time.sleep(args.interval)
            now = time.monotonic()
            dt = now - prev_t
            row, ticks = sample(names, args.exe_prefix, prev_ticks)
            ctxt, intr = system_counters()
            w.writerow(
                [
                    round(now - start, 1),
                    row["procs"],
                    round(100.0 * row["cpu_ticks"] / CLOCK_TICKS / dt, 2),
                    round(row["rss_mb"], 1),
                    round((ctxt - prev_ctxt) / dt, 1),
                    round((intr - prev_intr) / dt, 1),
                ]
            )
            f.flush()
            prev_ticks, prev_ctxt, prev_intr, prev_t = ticks, ctxt, intr, now
    return summarize(csv_path, args.min_procs)


def summarize(csv_path, min_procs):
    """Loaded-window figures, the idle baseline, and the rise between them."""
    with open(csv_path) as f:
        rows = [
            {k: float(v) for k, v in r.items()} for r in csv.DictReader(f)
        ]
    active = [r for r in rows if r["procs"] >= min_procs]
    idle = [r for r in rows if r["procs"] == 0]
    if not active:
        print(
            f"no samples with >= {min_procs} nav processes "
            f"({len(rows)} rows total) — was the stack up?",
            file=sys.stderr,
        )
        return 1

    def stat(subset, key):
        vals = [r[key] for r in subset]
        if not vals:
            return None
        return {
            "mean": round(statistics.mean(vals), 1),
            "median": round(statistics.median(vals), 1),
            "max": round(max(vals), 1),
        }

    def window(subset):
        if not subset:
            return None
        return {
            "samples": len(subset),
            "span_s": round(subset[-1]["t_s"] - subset[0]["t_s"], 1),
            "procs": stat(subset, "procs"),
            "cpu_pct": stat(subset, "cpu_pct"),
            "rss_mb": stat(subset, "rss_mb"),
            "ctxt_per_s": stat(subset, "ctxt_per_s"),
            "intr_per_s": stat(subset, "intr_per_s"),
        }

    result = {
        "csv": str(csv_path),
        "samples_total": len(rows),
        "loaded": window(active),
        "idle": window(idle),
    }
    if idle:
        result["rise_over_idle"] = {
            key: round(
                result["loaded"][key]["mean"] - result["idle"][key]["mean"], 1
            )
            for key in ("cpu_pct", "rss_mb", "ctxt_per_s", "intr_per_s")
        }
    Path(csv_path).with_name("overhead.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--duration", type=float, default=900.0, help="wall s to sample")
    ap.add_argument("--interval", type=float, default=5.0, help="s between samples")
    ap.add_argument("--out", default="overhead", help="directory for the CSV/JSON")
    ap.add_argument(
        "--names",
        default="",
        help="comma-separated executable basenames to match "
        f"(default: {','.join(DEFAULT_NAMES)})",
    )
    ap.add_argument(
        "--exe-prefix",
        default=str(REPO),
        help="only count processes whose executable lives under this path "
        "(default: this checkout, so a sibling worktree's stack is ignored)",
    )
    ap.add_argument(
        "--min-procs",
        type=int,
        default=2,
        help="a sample counts as 'stack up' at or above this process count",
    )
    ap.add_argument(
        "--summary",
        default="",
        help="skip sampling; summarize this existing overhead.csv",
    )
    args = ap.parse_args()
    if args.summary:
        return summarize(Path(args.summary), args.min_procs)
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
