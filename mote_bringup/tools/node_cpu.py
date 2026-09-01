#!/usr/bin/env python3
"""Sample per-node CPU on the robot, one column per node.

The monitor nodes are Python processes whose own logic runs at 1-10 Hz, so what
they cost is dominated by how often they are *woken* rather than by what they
compute. Answering that needs a figure per node, before and after a change, on
the same workload:

    pixi run node-cpu --duration 60 --tag idle \
        --out docs/tuning/2026-08-11-monitor-cpu/before

CPU comes from ``/proc/<pid>/stat`` as a delta of utime+stime over the interval,
the same source and arithmetic as the benchmark's ``overhead.py`` — which is not
reused here because it lives in ``mote_simulation``, deliberately excluded from
``pixi run sync``, and this has to run on the Pi. It also totals its matches into
one figure, where the question here is which node is expensive. Both are stdlib
only: ``pidstat`` is installed on neither the workstation nor the Pi.

A Python node has no distinguishing executable — every one of them is the
interpreter — so a node is identified by what its command line says it is
running (see :func:`node_instance`), confined to processes whose command line
names ``--prefix`` (default: this checkout). A box running several worktrees, or
holding a stack leaked by a dead agent job (see ``pixi run sweep``), would
otherwise have another checkout's monitors counted as this one's. A C++ node is
the opposite case and is matched on being an installed executable itself, which
is what keeps the sampler working across a port from one language to the other —
the measurement that justifies such a port needs both builds in one run.

Two builds of one node can be sampled *against each other* by starting the
second one renamed, which is the only sound way to compare them on a robot whose
own condition drifts between runs — a servo bus that answers in one run and not
the next moves ``/joint_states`` and ``/tf`` by tens of Hz, and those rates are
exactly what the monitors cost:

    ros2 run mote_health health_monitor --ros-args -r __node:=health_monitor_b \
        -r diagnostics_agg:=diagnostics_agg_b -r health:=health_b
    pixi run node-cpu --nodes health_monitor,health_monitor_b

Load average is recorded beside each sample: a percentage of a core means little
without knowing how contended the machine was when it was measured.
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

# The nodes this exists to weigh: the monitors, plus the task layer.
DEFAULT_NODES = ["health_monitor", "task_server", "slip_monitor", "system_monitor"]

CLOCK_TICKS = os.sysconf("SC_CLK_TCK")
PAGE_SIZE = os.sysconf("SC_PAGE_SIZE")
REPO = Path(__file__).resolve().parents[2]


def _candidates(prefix):
    """[(pid, argv)] for the processes this sampler is allowed to match."""
    self_pid = os.getpid()
    this_file = Path(__file__).name
    out = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        if pid == self_pid:
            continue
        try:
            raw = entry.joinpath("cmdline").read_bytes().decode(errors="replace")
        except OSError:
            continue
        argv = [tok for tok in raw.split("\0") if tok]
        if not argv:
            continue
        joined = " ".join(argv)
        # This sampler names every node it measures on its own command line.
        if this_file in joined or "node_cpu" in joined:
            continue
        if prefix and prefix not in joined:
            continue
        out.append((pid, argv))
    return out


def _program(argv):
    """The installed executable this process is, or None if it is not one.

    A node started through ``ros2 run`` or ``pixi run`` is wrapped by processes
    that repeat its whole command line in their own, so a substring match would
    weigh a wrapper — one of which does no work at all — instead of the node.

    A Python node is the only one of them an interpreter runs directly: argv[0]
    is a python and argv[1] is the installed entry point, whose name is the
    node's. A C++ node has no interpreter at all, and is instead the process
    whose argv[0] is itself an installed executable — ``ros2 run`` finds it
    under ``install/<pkg>/lib/<pkg>/``, which is what tells it apart from the
    wrappers (whose argv[0] is python, pixi or a shell) inside a candidate set
    already confined to this checkout.
    """
    program = os.path.basename(argv[0])
    if "python" in program:
        if len(argv) < 2:
            return None
        program = os.path.basename(argv[1])
        return None if program == "ros2" else program
    parts = Path(argv[0]).parts
    if "install" in parts and "lib" in parts:
        return program
    return None


def node_instance(argv):
    """The node this process *is*, or None if it is not one.

    The name reported is the ``__node:=`` rename where a launch or a remap gave
    it one, and the executable's own name otherwise. That is what lets two
    builds of one node be weighed against each other in a single run: they
    differ only by the rename, and the rename is what tells them apart.
    """
    program = _program(argv)
    if program is None:
        return None
    for tok in argv[1:]:
        if tok.startswith("__node:="):
            return tok.split(":=", 1)[1]
    return program


def matching_pids(nodes, prefix):
    """{node name: pid} for the named nodes running out of ``prefix``.

    A node that is not running is simply absent, so a run with the stack half up
    still yields figures for what is up.
    """
    found = {}
    for pid, argv in _candidates(prefix):
        instance = node_instance(argv)
        if instance in nodes and instance not in found:
            found[instance] = pid
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


def run(args):
    nodes = [n for n in args.nodes.split(",") if n]
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / f"{args.tag}.csv"

    pids = matching_pids(nodes, args.prefix)
    missing = [n for n in nodes if n not in pids]
    if missing:
        print(f"not running: {', '.join(missing)}", file=sys.stderr)
    if not pids:
        print("no matching processes — is the stack up?", file=sys.stderr)
        return 1
    print(
        f"sampling {args.duration:.0f}s: "
        + ", ".join(f"{n}={p}" for n, p in pids.items())
    )

    prev = {n: proc_cpu_ticks(p) for n, p in pids.items()}
    prev_t = time.monotonic()
    deadline = prev_t + args.duration
    start = prev_t

    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "t_s",
                *(f"{n}_cpu_pct" for n in nodes),
                *(f"{n}_rss_mb" for n in nodes),
                "load1",
            ]
        )
        while time.monotonic() < deadline:
            time.sleep(args.interval)
            now = time.monotonic()
            dt = now - prev_t
            cpu, rss = {}, {}
            for node, pid in pids.items():
                ticks = proc_cpu_ticks(pid)
                # A node that died mid-run leaves blanks rather than a huge
                # negative delta from a recycled pid.
                if ticks is None or prev.get(node) is None:
                    cpu[node] = rss[node] = None
                else:
                    cpu[node] = 100.0 * (ticks - prev[node]) / CLOCK_TICKS / dt
                    rss[node] = proc_rss_bytes(pid) / 1e6
                prev[node] = ticks
            w.writerow(
                [
                    round(now - start, 1),
                    *(None if cpu.get(n) is None else round(cpu[n], 2) for n in nodes),
                    *(None if rss.get(n) is None else round(rss[n], 1) for n in nodes),
                    round(os.getloadavg()[0], 2),
                ]
            )
            f.flush()
            prev_t = now
    return summarize(csv_path, nodes)


def summarize(csv_path, nodes=None):
    with open(csv_path) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        print(f"{csv_path}: no samples", file=sys.stderr)
        return 1
    if nodes is None:
        nodes = [k[: -len("_cpu_pct")] for k in rows[0] if k.endswith("_cpu_pct")]

    def stat(key):
        vals = [float(r[key]) for r in rows if r.get(key) not in (None, "")]
        if not vals:
            return None
        return {
            "mean": round(statistics.mean(vals), 1),
            "median": round(statistics.median(vals), 1),
            "p95": round(sorted(vals)[min(len(vals) - 1, int(0.95 * len(vals)))], 1),
            "max": round(max(vals), 1),
        }

    result = {
        "csv": str(csv_path),
        "samples": len(rows),
        "span_s": round(float(rows[-1]["t_s"]) - float(rows[0]["t_s"]), 1),
        "load1": stat("load1"),
        "cpu_pct": {n: stat(f"{n}_cpu_pct") for n in nodes},
        "rss_mb": {n: stat(f"{n}_rss_mb") for n in nodes},
    }
    result["cpu_pct"] = {k: v for k, v in result["cpu_pct"].items() if v}
    result["rss_mb"] = {k: v for k, v in result["rss_mb"].items() if v}
    Path(csv_path).with_suffix(".json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--duration", type=float, default=60.0, help="wall s to sample")
    ap.add_argument("--interval", type=float, default=1.0, help="s between samples")
    ap.add_argument("--out", default="node_cpu", help="directory for the CSV/JSON")
    ap.add_argument("--tag", default="run", help="names the CSV, e.g. 'before-idle'")
    ap.add_argument(
        "--nodes",
        default=",".join(DEFAULT_NODES),
        help=f"comma-separated node names to match (default: {','.join(DEFAULT_NODES)})",
    )
    ap.add_argument(
        "--prefix",
        default=str(REPO),
        help="only count processes whose command line names this path "
        "(default: this checkout, so a sibling worktree's stack is ignored)",
    )
    ap.add_argument("--summary", default="", help="skip sampling; summarize this CSV")
    args = ap.parse_args()
    if args.summary:
        return summarize(Path(args.summary))
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
