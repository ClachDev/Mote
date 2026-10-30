"""Read, clear and restore the servos' goal-range registers.

Registers 9 and 11 (``Min_Angle_Limit`` / ``Max_Angle_Limit``, EEPROM) fence
which goal positions a servo will accept. A goal outside the band is refused
**silently**: the joint stops at the same angle every time, in one direction
only, at any load — which reads exactly like running out of torque, and appears
in no config file, no URDF and no log.

    pixi run arm-setup limits show      # read-only: the band, in counts and radians
    pixi run arm-setup limits clear     # hand every joint its whole 0-4095 range back
    pixi run arm-setup limits restore   # write the as-found bands back

``clear`` is the normal state for this arm. The soft limits that matter are in
``$MOTE_HOME/arm.yaml``, enforced by ``MoteHardware`` and by ``teleop.py``,
where they are visible and versioned; a second band hidden in EEPROM only
duplicates them until the day the zero moves and the two disagree. There is
deliberately no ``set``: a narrower envelope belongs in ``arm.yaml``.

Opens the bus directly, so run it with the driver stopped (`pixi run kill`).
``show`` never writes; ``clear`` and ``restore`` write EEPROM and ask first.
"""

from __future__ import annotations

from datetime import datetime, timezone

from mote_arm.bus import COUNTS_PER_TURN
from mote_arm.calibrate import (
    limits_backup_path,
    load_limits_backup,
    save_limits_backup,
)

FULL_RANGE = (0, COUNTS_PER_TURN - 1)


def cuts(joint, band: tuple[int, int]) -> bool:
    """True if the servo's band refuses part of the joint's configured range."""
    low, high = sorted((joint.counts_to_rad(band[0]), joint.counts_to_rad(band[1])))
    return low > joint.min_rad + 1e-6 or high < joint.max_rad - 1e-6


def _read_all(cfg, bus) -> dict[str, tuple[int, int] | None]:
    return {j.name: bus.read_angle_limits(j.id) for j in cfg.joints}


def _print_table(cfg, bus) -> dict[str, tuple[int, int] | None]:
    bands = _read_all(cfg, bus)
    print(
        f"\n{'joint':<16}{'id':>3}{'min':>7}{'max':>7}"
        f"   {'accepts (rad)':<20}{'configured':<20}"
    )
    for joint in cfg.joints:
        band = bands[joint.name]
        if band is None:
            print(f"{joint.name:<16}{joint.id:>3}{'unreadable':>14}")
            continue
        # counts_to_rad honours `invert`, so an inverted joint's low count is
        # its high angle; order them by angle, not by register.
        low, high = sorted((joint.counts_to_rad(band[0]), joint.counts_to_rad(band[1])))
        note = "  <-- CUTS THE CONFIGURED RANGE" if cuts(joint, band) else ""
        print(
            f"{joint.name:<16}{joint.id:>3}{band[0]:>7}{band[1]:>7}   "
            f"{f'[{low:+.3f}, {high:+.3f}]':<20}"
            f"{f'[{joint.min_rad:+.3f}, {joint.max_rad:+.3f}]':<20}{note}"
        )
    return bands


def _cmd_show(cfg, bus, args) -> None:
    bands = _print_table(cfg, bus)
    fenced = [j.name for j in cfg.joints if bands[j.name] and cuts(j, bands[j.name])]
    if fenced:
        print(
            f"\n{', '.join(fenced)} stop short of their configured range and say "
            "nothing about it. `pixi run arm-setup limits clear` hands the whole "
            "0-4095 range back; the soft limits in arm.yaml still apply."
        )
    backup = load_limits_backup()
    if backup:
        print(f"\na backup exists at {limits_backup_path()}:")
        for name, (low, high) in sorted(backup.items()):
            print(f"  {name:<16}{low:>7}{high:>7}")


def _write(bus, cfg, wanted: dict[str, tuple[int, int]], args) -> None:
    print(f"\nwill write {len(wanted)} band(s):")
    for name, (low, high) in sorted(wanted.items()):
        print(f"  {name:<16}{low:>7}{high:>7}")
    print("this writes servo EEPROM — a persistent hardware-config change.")
    if not args.yes and input("proceed? [y/N] ").strip().lower() not in ("y", "yes"):
        print("aborted; nothing written")
        return

    failures = []
    for name, (low, high) in sorted(wanted.items()):
        joint = cfg.joint(name)
        ok = bus.write_angle_limits(joint.id, low, high)
        print(f"  {name:<16} {'written and verified' if ok else 'FAILED'}")
        if not ok:
            failures.append(name)
    _print_table(cfg, bus)
    if failures:
        raise SystemExit(
            f"could not verify the band on: {failures}. "
            f"`pixi run arm-setup limits restore` puts the as-found values back."
        )


def _backup_once(cfg, bus, bands) -> None:
    """Snapshot the as-found bands before the first write, never after.

    Written once and then left alone: a second snapshot taken after a `clear`
    would record 0-4095 over the values it exists to preserve.
    """
    if limits_backup_path().exists():
        return
    missing = [n for n, v in bands.items() if v is None]
    if missing:
        raise SystemExit(
            f"could not read the band on {missing} — refusing to write a "
            "partial backup, since restoring from it would be wrong."
        )
    path = save_limits_backup(
        {n: v for n, v in bands.items() if v is not None},
        {j.name: j.id for j in cfg.joints},
        datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%SZ"),
    )
    print(f"\nbacked the as-found bands up to {path}")


def _cmd_clear(cfg, bus, args) -> None:
    bands = _print_table(cfg, bus)
    selected = [j for j in cfg.joints if not args.joint or j.name == args.joint]
    if args.joint and not selected:
        raise SystemExit(f"unknown joint {args.joint!r}; have {cfg.names}")
    wanted = {
        j.name: FULL_RANGE for j in selected if bands[j.name] not in (FULL_RANGE, None)
    }
    if not wanted:
        print("\nevery joint already accepts its whole range — nothing to write.")
        return
    _backup_once(cfg, bus, bands)
    _write(bus, cfg, wanted, args)


def _cmd_restore(cfg, bus, args) -> None:
    backup = load_limits_backup()
    if not backup:
        raise SystemExit(
            f"no backup at {limits_backup_path()} — nothing to restore from."
        )
    current = _print_table(cfg, bus)
    wanted = {
        name: band
        for name, band in backup.items()
        if name in cfg.names and current.get(name) != band
    }
    if not wanted:
        print("\nevery servo already matches the backup — nothing to write.")
        return
    _write(bus, cfg, wanted, args)


def add_subparser(sub) -> None:
    parser = sub.add_parser(
        "limits", help="the servos' goal-range fence (EEPROM registers 9 and 11)"
    )
    inner = parser.add_subparsers(dest="action", required=True)
    inner.add_parser("show", help="read the bands (read-only)").set_defaults(
        func=_cmd_show
    )
    p_clear = inner.add_parser("clear", help="accept the whole 0-4095 range")
    p_clear.add_argument("--joint", default="", help="one joint (default: all)")
    p_clear.set_defaults(func=_cmd_clear)
    inner.add_parser("restore", help="write the as-found bands back").set_defaults(
        func=_cmd_restore
    )


__all__ = ["add_subparser", "cuts", "FULL_RANGE"]
