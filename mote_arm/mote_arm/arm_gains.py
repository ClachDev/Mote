"""Show or apply the arm servos' position-loop gains.

The gains live in each servo's EEPROM, which makes them invisible config: swap a
servo and the tuning silently reverts. ``robot.yaml``'s ``arm.gains`` is the
source of truth, and this tool reconciles the hardware with it.

    pixi run arm-gains show     # read-only comparison against robot.yaml
    pixi run arm-gains apply    # write robot.yaml's gains, verifying each servo

Opens the bus directly, so run it with the driver stopped. ``apply`` writes
EEPROM — a persistent change — so it asks first unless ``--yes`` is given, and
reports success only for servos whose read-back confirms the new values.
"""

from __future__ import annotations

import argparse

from mote_arm import config
from mote_arm.bus import BusError, FeetechBus, port_holders


def _open_bus(cfg) -> FeetechBus:
    holders = port_holders(cfg.port)
    if holders:
        for pid, cmd in holders:
            print(f"  port held by pid {pid}: {cmd}")
        raise SystemExit(
            "refusing to share the bus — stop the arm driver / robot base first "
            "(`pixi run kill`)."
        )
    bus = FeetechBus(cfg.port, cfg.baud_rate)
    try:
        bus.open()
    except BusError as exc:
        raise SystemExit(f"cannot open bus: {exc}")
    return bus


def _report(cfg, bus) -> list[tuple[str, int, tuple[int, int, int] | None]]:
    want = (cfg.gains.kp, cfg.gains.kd, cfg.gains.ki)
    print(f"robot.yaml arm.gains: kp={want[0]} kd={want[1]} ki={want[2]}\n")
    print(f"{'joint':<16}{'id':>3}{'kp':>6}{'kd':>6}{'ki':>6}   status")
    rows = []
    for joint in cfg.joints:
        got = bus.read_gains(joint.id)
        if got is None:
            status = "UNREADABLE"
            shown = ("?", "?", "?")
        else:
            status = "matches robot.yaml" if got == want else "DIFFERS"
            shown = got
        print(
            f"{joint.name:<16}{joint.id:>3}"
            f"{str(shown[0]):>6}{str(shown[1]):>6}{str(shown[2]):>6}   {status}"
        )
        rows.append((joint.name, joint.id, got))
    return rows


def _cmd_show(cfg, bus, args) -> None:
    _report(cfg, bus)


def _cmd_apply(cfg, bus, args) -> None:
    want = (cfg.gains.kp, cfg.gains.kd, cfg.gains.ki)
    rows = _report(cfg, bus)
    stale = [(n, i, g) for n, i, g in rows if g != want]
    if not stale:
        print("\nall servos already match robot.yaml — nothing to write.")
        return

    print(
        f"\nwill write kp={want[0]} kd={want[1]} ki={want[2]} to "
        f"{len(stale)} servo(s): {[n for n, _, _ in stale]}"
    )
    print("this writes servo EEPROM — a persistent hardware-config change.")
    if not args.yes:
        if input("proceed? [y/N] ").strip().lower() not in ("y", "yes"):
            print("aborted; nothing written")
            return

    failures = []
    for name, servo_id, _ in stale:
        ok = bus.write_gains(servo_id, *want)
        print(f"  {name:<16} {'written and verified' if ok else 'FAILED'}")
        if not ok:
            failures.append(name)

    print("\nfinal state:")
    _report(cfg, bus)
    if failures:
        raise SystemExit(f"could not verify gains on: {failures}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Arm servo position-loop gains")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_show = sub.add_parser("show", help="read gains and compare to robot.yaml")
    p_show.set_defaults(func=_cmd_show)

    p_apply = sub.add_parser("apply", help="write robot.yaml's gains to the servos")
    p_apply.add_argument("--yes", action="store_true", help="skip confirmation")
    p_apply.set_defaults(func=_cmd_apply)

    args = parser.parse_args()
    cfg = config.load()
    bus = _open_bus(cfg)
    try:
        args.func(cfg, bus, args)
    finally:
        bus.close()


if __name__ == "__main__":
    main()
