"""Read, back up, restore and set the servos' position-correction offsets.

The offset register (EEPROM, ``SMS_STS_OFS_L/H``) is the one piece of arm state
with no copy anywhere else: it lives only in the servo, and overwriting it
destroys the previous value. ``arm-setup calibrate`` writes it, so this exists to see
what is there, to put it back, and to set one by hand.

    pixi run arm-setup offsets show       # read-only: raw register + decoded value
    pixi run arm-setup offsets backup     # snapshot the current offsets to ~/.mote
    pixi run arm-setup offsets restore    # write the snapshot back
    pixi run arm-setup offsets set --joint shoulder_pan --value 2027

Opens the bus directly, so run it with the driver stopped. ``show`` and
``backup`` never write. ``restore`` and ``set`` write EEPROM and ask first.

``show`` prints the raw register alongside the decoded value on purpose. The
decoding assumes bit 11 is a sign bit; if a servo's raw value looks nothing like
its decode, that assumption is what to doubt first.
"""

from __future__ import annotations

from datetime import datetime, timezone

from mote_arm.bus import OFFSET_MAX, decode_sign_magnitude
from mote_arm.calibrate import (
    load_offsets_backup,
    offsets_backup_path,
    save_offsets_backup,
)


def _read_all(cfg, bus) -> dict[str, int | None]:
    return {j.name: bus.read_homing_offset(j.id) for j in cfg.joints}


def _print_table(cfg, bus) -> dict[str, int | None]:
    offsets = _read_all(cfg, bus)
    print(f"\n{'joint':<16}{'id':>3}{'raw':>8}{'offset':>8}{'position':>10}")
    for joint in cfg.joints:
        raw = bus.read_homing_offset_raw(joint.id)
        value = offsets[joint.name]
        pos = bus.read_position(joint.id)
        print(
            f"{joint.name:<16}{joint.id:>3}"
            f"{'?' if raw is None else raw:>8}"
            f"{'?' if value is None else value:>8}"
            f"{'?' if pos is None else pos:>10}"
        )
    return offsets


def _cmd_show(cfg, bus, args) -> None:
    _print_table(cfg, bus)
    backup = load_offsets_backup()
    if backup:
        print(f"\na backup exists at {offsets_backup_path()}:")
        for name, value in sorted(backup.items()):
            print(f"  {name:<16}{value:>8}")


def _cmd_backup(cfg, bus, args) -> None:
    offsets = _print_table(cfg, bus)
    missing = [n for n, v in offsets.items() if v is None]
    if missing:
        raise SystemExit(
            f"could not read the offset on {missing} — refusing to write a "
            "partial backup, since restoring from it would be wrong."
        )
    when = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%SZ")
    path = save_offsets_backup(offsets, {j.name: j.id for j in cfg.joints}, when)
    print(f"\nbacked up to {path}")


def _write(bus, cfg, wanted: dict[str, int], args) -> None:
    print(f"\nwill write {len(wanted)} offset(s):")
    for name, value in sorted(wanted.items()):
        print(f"  {name:<16}{value:>8}")
    print("this writes servo EEPROM — a persistent hardware-config change.")
    if not args.yes and input("proceed? [y/N] ").strip().lower() not in ("y", "yes"):
        print("aborted; nothing written")
        return

    failures = []
    for name, value in sorted(wanted.items()):
        joint = cfg.joint(name)
        ok = bus.write_homing_offset(joint.id, value)
        print(f"  {name:<16} {'written and verified' if ok else 'FAILED'}")
        if not ok:
            failures.append(name)
    _print_table(cfg, bus)
    if failures:
        raise SystemExit(f"could not verify the offset on: {failures}")


def _cmd_restore(cfg, bus, args) -> None:
    backup = load_offsets_backup()
    if not backup:
        raise SystemExit(
            f"no backup at {offsets_backup_path()} — nothing to restore from."
        )
    current = _print_table(cfg, bus)
    wanted = {
        name: value
        for name, value in backup.items()
        if name in cfg.names and current.get(name) != value
    }
    if not wanted:
        print("\nevery servo already matches the backup — nothing to write.")
        return
    _write(bus, cfg, wanted, args)


def _cmd_set(cfg, bus, args) -> None:
    if args.joint not in cfg.names:
        raise SystemExit(f"unknown joint {args.joint!r}; have {cfg.names}")
    if abs(args.value) > OFFSET_MAX:
        raise SystemExit(
            f"offset {args.value} outside the register's +-{OFFSET_MAX} range"
        )
    _print_table(cfg, bus)
    _write(bus, cfg, {args.joint: args.value}, args)


def add_subparser(sub) -> None:
    parser = sub.add_parser(
        "offsets", help="the servos' position-correction registers (EEPROM)"
    )
    inner = parser.add_subparsers(dest="action", required=True)
    inner.add_parser("show", help="read the offsets (read-only)").set_defaults(
        func=_cmd_show
    )
    inner.add_parser("backup", help="snapshot the offsets to ~/.mote").set_defaults(
        func=_cmd_backup
    )
    inner.add_parser("restore", help="write the snapshot back").set_defaults(
        func=_cmd_restore
    )
    p_set = inner.add_parser("set", help="write one joint's offset")
    p_set.add_argument("--joint", required=True)
    p_set.add_argument("--value", required=True, type=int)
    p_set.set_defaults(func=_cmd_set)


__all__ = ["add_subparser", "decode_sign_magnitude"]
