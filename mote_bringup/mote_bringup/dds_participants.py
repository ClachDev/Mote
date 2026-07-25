"""Count the DDS participants on this host against the localhost-discovery cap.

Pinning discovery to the local host (``ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST``)
makes rmw_cyclonedds hand CycloneDDS this config::

    <Discovery><ParticipantIndex>auto</ParticipantIndex>
    <MaxAutoParticipantIndex>32</MaxAutoParticipantIndex>
    <Peers><Peer address="localhost"/></Peers></Discovery>

so each participant claims the first free *participant index* 0..32 and binds a
pair of UDP ports derived from it (RTPS port mapping, PB=7400, DG=250, PG=2)::

    discovery port = 7400 + 250*domain + 10 + 2*index
    user port      = 7400 + 250*domain + 11 + 2*index

Past index 32 participant creation fails, which means *node* creation fails —
so the cap is a real ceiling on how many ROS processes can run on one robot.
rmw_cyclonedds creates one participant per context, i.e. one per process for a
normal launch, so the full stack (bringup + Nav2 + SLAM + perception + task
server + the fleet agent + foxglove_bridge) has to fit under it.

This tool answers "how close are we?" from ``/proc/net/udp`` alone (no ROS, no
CycloneDDS tooling, nothing to install on the Pi): it lists the claimed indices
with the process holding each one, and reports the headroom left.

    ros2 run mote_bringup dds_participants          # pixi run dds-check
    ros2 run mote_bringup dds_participants --json

If headroom ever runs out, raise ``MaxAutoParticipantIndex`` with a
``CYCLONEDDS_URI`` config (there is no CycloneDDS XML in the repo today — that
would be the first).
"""

import argparse
import json
import os
import sys
from pathlib import Path

PB, DG, PG = 7400, 250, 2
D0_MULTICAST_DISCOVERY, D1_UNICAST_DISCOVERY = 0, 10
D2_MULTICAST_USER, D3_UNICAST_USER = 1, 11
DEFAULT_MAX_INDEX = 32  # rmw_cyclonedds' MaxAutoParticipantIndex under LOCALHOST


def domain_base(domain: int) -> int:
    return PB + DG * domain


def discovery_port(domain: int, index: int) -> int:
    return domain_base(domain) + D1_UNICAST_DISCOVERY + PG * index


def bound_udp_ports() -> dict[int, list[int]]:
    """{port: [socket inode, ...]} for every bound UDP socket on this host."""
    ports: dict[int, list[int]] = {}
    for name in ("udp", "udp6"):
        try:
            lines = Path("/proc/net", name).read_text().splitlines()[1:]
        except OSError:
            continue
        for line in lines:
            fields = line.split()
            if len(fields) < 10:
                continue
            port = int(fields[1].rsplit(":", 1)[1], 16)
            ports.setdefault(port, []).append(int(fields[9]))
    return ports


def inode_owners() -> dict[int, tuple[int, str]]:
    """{socket inode: (pid, command)} for processes this user can inspect."""
    owners: dict[int, tuple[int, str]] = {}
    for proc in Path("/proc").iterdir():
        if not proc.name.isdigit():
            continue
        try:
            fds = list((proc / "fd").iterdir())
        except OSError:
            continue  # another user's process, or it exited
        command = ""
        for fd in fds:
            try:
                target = os.readlink(fd)
            except OSError:
                continue
            if not target.startswith("socket:["):
                continue
            if not command:
                command = process_name(proc)
            owners[int(target[8:-1])] = (int(proc.name), command)
    return owners


def process_name(proc: Path) -> str:
    try:
        cmdline = (proc / "cmdline").read_bytes().decode(errors="replace")
    except OSError:
        return "?"
    parts = [p for p in cmdline.split("\0") if p]
    if not parts:
        return "?"
    name = Path(parts[0]).name
    # Interpreters tell you nothing; the script/node name is what identifies a
    # participant (python3 -> depth_obstacle_node, ros2 -> launch, ...).
    for arg in parts[1:]:
        if arg.startswith("-"):
            continue
        return f"{name} {Path(arg).name}"
    return name


def scan(domain: int, max_index: int) -> dict:
    ports = bound_udp_ports()
    owners = inode_owners()
    participants = []
    for index in range(max_index + 1):
        port = discovery_port(domain, index)
        inodes = ports.get(port)
        if not inodes:
            continue
        pid, command = next(
            (owners[i] for i in inodes if i in owners), (None, "(other user)")
        )
        participants.append(
            {"index": index, "port": port, "pid": pid, "command": command}
        )
    base = domain_base(domain)
    return {
        "domain": domain,
        "discovery_range": os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE", "(unset)"),
        "multicast_discovery_bound": base + D0_MULTICAST_DISCOVERY in ports,
        "max_index": max_index,
        "capacity": max_index + 1,
        "used": len(participants),
        "free": max_index + 1 - len(participants),
        "participants": participants,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument(
        "--domain",
        type=int,
        default=int(os.environ.get("ROS_DOMAIN_ID", 0)),
        help="ROS domain to scan (default: ROS_DOMAIN_ID, else 0)",
    )
    parser.add_argument(
        "--max-index",
        type=int,
        default=DEFAULT_MAX_INDEX,
        help=f"MaxAutoParticipantIndex in force (default {DEFAULT_MAX_INDEX})",
    )
    parser.add_argument(
        "--min-free",
        type=int,
        default=4,
        help="exit non-zero if fewer than this many slots are left (default 4)",
    )
    parser.add_argument("--json", action="store_true", help="machine-readable output")
    args = parser.parse_args()

    result = scan(args.domain, args.max_index)
    if args.json:
        print(json.dumps(result, indent=2))
    else:
        print(
            f"domain {result['domain']}  "
            f"ROS_AUTOMATIC_DISCOVERY_RANGE={result['discovery_range']}  "
            f"multicast discovery port bound: "
            f"{'yes' if result['multicast_discovery_bound'] else 'no'}"
        )
        for p in result["participants"]:
            pid = p["pid"] if p["pid"] is not None else "?"
            print(
                f"  index {p['index']:>3}  port {p['port']}  pid {pid:<8} {p['command']}"
            )
        print(
            f"{result['used']}/{result['capacity']} participant slots used, "
            f"{result['free']} free (MaxAutoParticipantIndex={result['max_index']})"
        )

    if result["free"] < args.min_free:
        print(
            f"only {result['free']} participant slots left — raise "
            "MaxAutoParticipantIndex via CYCLONEDDS_URI before adding processes",
            file=sys.stderr,
        )
        sys.exit(1)


if __name__ == "__main__":
    main()
