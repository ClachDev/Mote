"""Standalone arm bus enumeration + health check (first-contact bench tool).

Opens the arm bus directly (no ROS node), pings every configured joint, and
prints position / voltage / temperature / load. It can also dump a robot.yaml
``home:`` snippet from the arm's current pose (``--save-home``) for calibration.

Read-only: it never enables torque or commands a goal, so it is the safe first
contact with the arm. Run it with the driver NOT running — the arm shares the
drive-wheel bus, so only one process may hold the port:
    pixi run arm-check
    pixi run arm-check -- --save-home
"""

from __future__ import annotations

import argparse
import os

from mote_arm import config
from mote_arm.bus import BusError, FeetechBus, port_holders


def _resolve_device(port: str) -> str:
    """Follow a symlink like /dev/mote_servos to its real /dev/tty* node."""
    return os.path.realpath(port)


def main() -> None:
    parser = argparse.ArgumentParser(description="SO-101 arm bus check")
    parser.add_argument("--robot-yaml", default="", help="override robot.yaml path")
    parser.add_argument(
        "--save-home",
        action="store_true",
        help="print a robot.yaml home: snippet from the current pose",
    )
    args = parser.parse_args()

    cfg = (
        config.ArmConfig.from_yaml_file(args.robot_yaml)
        if args.robot_yaml
        else config.load()
    )

    print(f"arm bus: {cfg.port} (-> {_resolve_device(cfg.port)}) @ {cfg.baud_rate}")
    print(f"expected joints: {cfg.names}")

    holders = port_holders(cfg.port)
    if holders:
        print("\nport is already open by:")
        for pid, cmd in holders:
            print(f"  pid {pid}: {cmd}")
        raise SystemExit(
            "refusing to share the bus — stop the arm driver / robot base first "
            "(`pixi run kill`)."
        )

    try:
        bus = FeetechBus(cfg.port, cfg.baud_rate)
        bus.open()
    except BusError as exc:
        raise SystemExit(f"cannot open bus: {exc}")

    homes: list[tuple[str, int]] = []
    missing = []
    try:
        print(
            f"\n{'joint':<14} {'id':>3} {'pos':>5} {'rad':>7} "
            f"{'volt':>5} {'temp':>4} {'load':>6}"
        )
        for joint in cfg.joints:
            health = bus.read_health(joint.id) if bus.ping(joint.id) else None
            if health is None:
                missing.append(joint)
                print(f"{joint.name:<14} {joint.id:>3}   --- NO RESPONSE ---")
                continue
            homes.append((joint.name, health.position))
            print(
                f"{joint.name:<14} {joint.id:>3} {health.position:>5} "
                f"{joint.counts_to_rad(health.position):>+7.3f} "
                f"{health.voltage:>5.1f} {health.temperature:>4} {health.load:>6}"
            )
    finally:
        bus.close()

    if missing:
        print(
            f"\nWARNING: {len(missing)} joint(s) did not respond: "
            f"{[j.name for j in missing]}"
        )
        print("check power, wiring, and servo IDs (see mote_hardware setup_ids).")
    else:
        print("\nall configured joints responded.")

    if args.save_home and homes:
        print("\ncalibration snapshot — pose each joint at its mechanical zero,")
        print("then paste these 'home:' values into robot.yaml's arm.joints:")
        for name, counts in homes:
            print(f"    # {name}: home: {counts}")


if __name__ == "__main__":
    main()
