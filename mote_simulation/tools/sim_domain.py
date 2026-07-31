#!/usr/bin/env python3
"""Claim an unused ROS graph (and Gazebo partition) for one sim run.

Every sim entry point that brings up a gz server — the benchmark, the smoke
test, ``map_world.sh`` — needs the same thing: a ``ROS_DOMAIN_ID`` nobody else on
this machine is using, so two runs started at once (in two worktrees, or a
benchmark beside a smoke test) cannot see each other's ``/scan``, ``/tf``,
``/clock`` and goals. Gazebo transport has its own discovery, independent of DDS,
so a matching ``GZ_PARTITION`` goes with it or the gz graphs would still merge.

Python callers use ``claim()``; shell callers eval the module::

    eval "$(python3 sim_domain.py --shell --prefix mote-smoke)"

which prints ``export`` lines plus a ``MOTE_DOMAIN_HOW`` describing where the
domain came from, for the caller to log.

Stdlib only, so a shell script can run it before any environment is sourced.
"""

from __future__ import annotations

import argparse
import os
import random
import sys
from pathlib import Path

# Domain 0 is the everything-else default; ROS 2 documents 0-101 as the range
# whose DDS ports stay clear of the Linux ephemeral port range.
DOMAIN_CANDIDATES = range(1, 102)
# Every DDS port for domain N falls in [7400 + 250*N, +250): multicast discovery
# at the base, per-participant unicast ports above it. Which of those are bound
# depends on the discovery mode — under ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
# CycloneDDS binds only the unicast ports — so the whole block is what tells us
# whether a domain is in use.
DDS_PORT_BASE, DDS_PORT_STEP = 7400, 250


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


def pick_domain_id(env=None):
    """(domain_id, how) for this run's ROS graph.

    An inherited ROS_DOMAIN_ID wins — the parameter sweep already assigns one per
    invocation, and a caller who set it meant it. Otherwise claim a domain whose
    DDS port block is unused on this host, so two runs started at the same time
    (or a sim left running in another worktree) land on different graphs instead
    of silently sharing goals, /clock and TF. The port probe is a best-effort
    filter, not a lock: it cannot see a domain that is claimed-but-not-yet-
    running, so the candidate order is randomised to make that race unlikely.
    """
    env = os.environ if env is None else env
    inherited = env.get("ROS_DOMAIN_ID", "").strip()
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


def claim(prefix, env=None):
    """Pick a domain + partition and write both into ``env`` (default os.environ).

    Returns (domain, how, partition). An inherited GZ_PARTITION is kept, for the
    same reason an inherited domain is.
    """
    env = os.environ if env is None else env
    domain, how = pick_domain_id(env)
    env["ROS_DOMAIN_ID"] = str(domain)
    partition = env.setdefault("GZ_PARTITION", f"{prefix}-{domain}")
    return domain, how, partition


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--prefix",
        default="mote-sim",
        help="GZ_PARTITION name prefix (default mote-sim)",
    )
    ap.add_argument(
        "--shell",
        action="store_true",
        help="print shell assignments to eval (default: '<domain> <how>')",
    )
    args = ap.parse_args(argv)

    env = {k: v for k, v in os.environ.items()}
    domain, how, partition = claim(args.prefix, env)
    if args.shell:
        print(f"export ROS_DOMAIN_ID={domain}")
        print(f"export GZ_PARTITION={partition}")
        print(f"MOTE_DOMAIN_HOW='{how}'")
    else:
        print(f"{domain} {how}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
