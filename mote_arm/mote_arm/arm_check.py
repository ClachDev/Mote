"""Standalone arm bus enumeration + health check (first-contact bench tool).

Opens the arm bus directly (no ROS node), pings every configured joint, and
prints position / voltage / temperature / load. It can also dump a robot.yaml
``zero:`` snippet from the arm's current pose (``--save-zero``). That is a
convenience, not calibration: it measures no range, so the limits stay as they
were. `pixi run arm-calibrate` is what sets zeros and limits together.

Read-only: it never enables torque or commands a goal, so it is the safe first
contact with the arm. Run it with the driver NOT running — the arm shares the
drive-wheel bus, so only one process may hold the port:
    pixi run arm-check
    pixi run arm-check -- --save-zero
"""

from __future__ import annotations

import argparse
import os

from mote_arm import config
from mote_arm.bus import BusError, FeetechBus, port_holders


def _resolve_device(port: str) -> str:
    """Follow a symlink like /dev/mote_servos to its real /dev/tty* node."""
    return os.path.realpath(port)


def _report_angle_limits(limits: list) -> None:
    """What the servo itself will accept, against what the config asks for.

    These registers live only in the servo. A joint whose configured band runs
    past them stops dead at the same angle every time, in one direction, at any
    load -- indistinguishable from running out of torque, and invisible in
    robot.yaml, arm.yaml, the URDF and every other tool here.
    """
    if not limits:
        return
    print("\nservo goal-range limits (EEPROM, registers 9-12):")
    print(f"{'joint':<14} {'min':>5} {'max':>5}   {'accepts (rad)':>18}   configured")
    problems = []
    for joint, band in limits:
        if band is None:
            print(f"{joint.name:<14}   ---   ---   (could not read)")
            continue
        low, high = band
        # counts_to_rad honours `invert`, so an inverted joint's low count is
        # its high angle; order them by angle, not by register.
        angles = sorted((joint.counts_to_rad(low), joint.counts_to_rad(high)))
        note = ""
        if angles[0] > joint.min_rad + 1e-6 or angles[1] < joint.max_rad - 1e-6:
            note = "  <-- NARROWER THAN CONFIGURED"
            problems.append(joint.name)
        print(
            f"{joint.name:<14} {low:>5} {high:>5}   "
            f"[{angles[0]:+.3f}, {angles[1]:+.3f}]   "
            f"[{joint.min_rad:+.3f}, {joint.max_rad:+.3f}]{note}"
        )
    if problems:
        print(
            f"\n{', '.join(problems)}: the servo will refuse goals outside its own "
            "band, so the joint stops there whatever robot.yaml and arm.yaml say. "
            "Widen the register or narrow the configured limits to match."
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="SO-101 arm bus check")
    parser.add_argument("--robot-yaml", default="", help="override robot.yaml path")
    parser.add_argument(
        "--save-zero",
        "--save-home",
        dest="save_zero",
        action="store_true",
        help="print a robot.yaml zero: snippet from the current pose",
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

    zeros: list[tuple[str, int]] = []
    missing = []
    try:
        print(
            f"\n{'joint':<14} {'id':>3} {'pos':>5} {'rad':>7} "
            f"{'volt':>5} {'temp':>4} {'load':>6}"
        )
        limits: list = []
        for joint in cfg.joints:
            health = bus.read_health(joint.id) if bus.ping(joint.id) else None
            if health is None:
                missing.append(joint)
                print(f"{joint.name:<14} {joint.id:>3}   --- NO RESPONSE ---")
                continue
            zeros.append((joint.name, health.position))
            print(
                f"{joint.name:<14} {joint.id:>3} {health.position:>5} "
                f"{joint.counts_to_rad(health.position):>+7.3f} "
                f"{health.voltage:>5.1f} {health.temperature:>4} {health.load:>6}"
            )
            limits.append((joint, bus.read_angle_limits(joint.id)))
        _report_angle_limits(limits)
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

    if args.save_zero and zeros:
        print(
            "\nsnapshot of the current pose (this sets no limits — see arm-calibrate):"
        )
        print("paste these 'zero:' values into robot.yaml's arm.joints:")
        for name, counts in zeros:
            print(f"    # {name}: zero: {counts}")


if __name__ == "__main__":
    main()
