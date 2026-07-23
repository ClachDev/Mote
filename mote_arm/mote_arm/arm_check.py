"""Standalone arm bus enumeration + health check (first-contact bench tool).

Opens the arm bus directly (no ROS node), pings every configured joint, and
prints position / voltage / temperature / load. It also prints a ready-to-paste
udev line for the arm's USB-serial adapter, and can dump a robot.yaml ``home:``
snippet from the arm's current pose (``--save-home``) for calibration.

Run this with the driver NOT running (it owns the same port):
    pixi run arm-check
    pixi run arm-check -- --save-home
"""

from __future__ import annotations

import argparse
import os
import subprocess

from mote_arm import config
from mote_arm.bus import BusError, FeetechBus


def _resolve_device(port: str) -> str:
    """Follow a symlink like /dev/mote_arm to its real /dev/tty* node."""
    return os.path.realpath(port)


def _udev_hint(port: str) -> None:
    dev = _resolve_device(port)
    print(f"\nudev helper for {port} (-> {dev}):")
    try:
        out = subprocess.run(
            ["udevadm", "info", "-a", "-n", dev],
            capture_output=True,
            text=True,
            check=False,
        ).stdout
    except FileNotFoundError:
        print("  udevadm not found; run on the robot to read the adapter IDs")
        return

    def first(attr: str) -> str:
        for line in out.splitlines():
            line = line.strip()
            if line.startswith(f"ATTRS{{{attr}}}=="):
                return line.split("==", 1)[1].strip().strip('"')
        return "????"

    vid, pid, serial = first("idVendor"), first("idProduct"), first("serial")
    print(f"  idVendor={vid}  idProduct={pid}  serial={serial}")
    print("  paste into mote_bringup/udev/99-mote.rules (pin by serial if the")
    print("  wheel board shares this VID:PID):")
    print(
        f'  SUBSYSTEM=="tty", ATTRS{{idVendor}}=="{vid}", '
        f'ATTRS{{idProduct}}=="{pid}", ATTRS{{serial}}=="{serial}", '
        'SYMLINK+="mote_arm", MODE="0666"'
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="SO-101 arm bus check")
    parser.add_argument("--robot-yaml", default="", help="override robot.yaml path")
    parser.add_argument(
        "--save-home",
        action="store_true",
        help="print a robot.yaml home: snippet from the current pose",
    )
    parser.add_argument("--no-udev", action="store_true", help="skip the udev hint")
    args = parser.parse_args()

    cfg = (
        config.ArmConfig.from_yaml_file(args.robot_yaml)
        if args.robot_yaml
        else config.load()
    )

    print(f"arm bus: {cfg.port} @ {cfg.baud_rate}")
    print(f"expected joints: {cfg.names}")

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
        print("\ncalibration snapshot — set each joint to its mechanical zero, then")
        print("re-run with --save-home and paste these 'home:' values into robot.yaml:")
        for name, counts in homes:
            print(f"  # {name}: home: {counts}")

    if not args.no_udev:
        _udev_hint(cfg.port)


if __name__ == "__main__":
    main()
