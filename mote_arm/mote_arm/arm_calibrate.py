"""Guided range calibration, LeRobot-style: centre the joints, then sweep them.

Two phases, and the first is what makes the second trustworthy:

1. **Homing offsets.** You park the arm with every joint near the middle of its
   travel; each servo's position-correction register is written so that pose
   reads 2048. This re-centres every joint's travel inside the 0-4095 encoder
   frame, which is what stops a joint's range straddling the wrap. It writes
   EEPROM, so it asks first.
2. **Ranges.** You move every joint to both of its mechanical stops while a
   single live table records all six at once. One Enter ends it.

    pixi run arm-calibrate
    pixi run arm-calibrate -- --skip-homing        # ranges only, writes nothing
    pixi run arm-calibrate -- --joints wrist_roll  # redo one joint

This replaces ``arm-pose limits`` as the way soft limits are set. That command
widens *outward* from poses a human has already vetted, which only ever
describes where the arm has been — it never learns where the stops are, and a
joint that barely moved between two taught poses ends up with a near-zero band.
Calibration measures the stops directly and works inward from them.

**It opens the serial bus directly**, like ``arm_check`` and ``arm_gains``, so
run it with the driver stopped. It is a bus owner rather than an ``arm_driver``
client for two reasons: the driver reports radians about the very zero this tool
exists to replace, so a client would be measuring against the offset under test;
and the arm must be limp and back-drivable throughout, which is the opposite of
what the driver is for.

Note the vocabulary. ``zero`` is where 0 rad sits — after calibration, the
middle of each joint's travel. ``home`` is a *taught pose* in
``arm_poses.yaml``, usually the arm's rest position. They are different places
and this tool never conflates them.
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from datetime import datetime, timezone

from mote_arm import config, poses
from mote_arm.bus import BusError, FeetechBus, port_holders
from mote_arm.calibrate import (
    CENTRE_COUNTS,
    DEFAULT_MARGIN,
    CalibrationError,
    SweepRecorder,
    calibrate_joint,
    homing_offset,
    joints_block,
    pose_impact,
    save_record,
    zero_shift,
)
from mote_arm.config import RAD_PER_COUNT


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


def _confirm(prompt: str, assume_yes: bool) -> bool:
    if assume_yes:
        return True
    return input(prompt).strip().lower() in ("y", "yes")


def _wait_for_enter() -> threading.Event:
    """Event set when the operator presses Enter, so a loop can poll on it."""
    done = threading.Event()

    def wait() -> None:
        try:
            input()
        except EOFError:
            pass
        done.set()

    threading.Thread(target=wait, daemon=True).start()
    return done


class _LiveTable:
    """A table that redraws over itself, so the whole arm is visible at once.

    Falls back to printing nothing when stdout is not a terminal: the cursor
    control would otherwise fill a log with escape codes.
    """

    def __init__(self, header: str, columns: str):
        self.header = header
        self.columns = columns
        self._lines = 0
        self._tty = sys.stdout.isatty()

    def draw(self, rows: list[str]) -> None:
        if not self._tty:
            return
        if self._lines:
            sys.stdout.write(f"\033[{self._lines}A")
        out = [self.header, self.columns, *rows]
        for line in out:
            sys.stdout.write(f"\r{line}\033[K\n")
        sys.stdout.flush()
        self._lines = len(out)

    def final(self, rows: list[str]) -> None:
        """Leave the finished table on screen, drawing it if we never could."""
        if self._tty:
            self.draw(rows)
            return
        for line in [self.header, self.columns, *rows]:
            print(line)


def _read_all(bus, joints) -> dict[str, int | None]:
    return {j.name: bus.read_position(j.id) for j in joints}


def _phase_offsets(bus, joints, args) -> dict[str, int] | None:
    """Write each servo's homing offset so the parked pose reads centre."""
    print("\n=== Phase 1 of 2: centre the joints ===")
    print(
        "Move the arm so EVERY joint sits near the middle of its travel — not\n"
        "against any stop — then press Enter. That pose becomes 0 rad."
    )
    input()

    present = _read_all(bus, joints)
    unreadable = [n for n, p in present.items() if p is None]
    if unreadable:
        raise SystemExit(f"could not read position for {unreadable} — check wiring")

    existing: dict[str, int] = {}
    for joint in joints:
        current = bus.read_homing_offset(joint.id)
        if current is None:
            raise SystemExit(
                f"could not read {joint.name}'s existing homing offset. Refusing "
                "to write a new one blind — a wrong offset silently moves every "
                "angle the arm reports."
            )
        existing[joint.name] = current

    wanted: dict[str, int] = {}
    print(f"\n{'joint':<16}{'now':>6}{'offset':>8}{'->':>4}{'new':>7}")
    for joint in joints:
        try:
            new = homing_offset(present[joint.name], existing[joint.name])
        except CalibrationError as exc:
            raise SystemExit(f"{joint.name}: {exc}")
        wanted[joint.name] = new
        print(
            f"{joint.name:<16}{present[joint.name]:>6}"
            f"{existing[joint.name]:>8}{'->':>4}{new:>7}"
        )

    stale = {n: v for n, v in wanted.items() if v != existing[n]}
    if not stale:
        print("\nevery servo is already centred here — nothing to write.")
        return wanted

    print(
        f"\nThis writes the position-correction register of {len(stale)} servo(s) "
        "— EEPROM,\na persistent hardware change. Afterwards the zero counts in "
        "robot.yaml are\nstale until you paste the block this prints at the end."
    )
    if not _confirm("write homing offsets? [y/N] ", args.yes):
        raise SystemExit("aborted; nothing written")

    failed = []
    for joint in joints:
        if joint.name not in stale:
            continue
        ok = bus.write_homing_offset(joint.id, wanted[joint.name])
        print(f"  {joint.name:<16} {'written and verified' if ok else 'FAILED'}")
        if not ok:
            failed.append(joint.name)
    if failed:
        raise SystemExit(f"could not verify the homing offset on: {failed}")

    # Prove it took where it matters: the joints should now read centre.
    time.sleep(0.2)
    after = _read_all(bus, joints)
    drift = {n: p for n, p in after.items() if p is None or abs(p - CENTRE_COUNTS) > 40}
    if drift:
        print(
            f"\nWARNING: after writing, {sorted(drift)} do not read near "
            f"{CENTRE_COUNTS}.\nThe arm may have moved while limp, which is "
            "harmless, but if it persists\nthe offset did not take."
        )
    else:
        print(f"\nall joints now read within 40 counts of {CENTRE_COUNTS}.")
    return wanted


def _phase_ranges(bus, joints, rate_hz: float) -> tuple[dict, int]:
    """Record every joint's range at once while the operator moves them."""
    print("\n=== Phase 2 of 2: record the ranges ===")
    print(
        "Move each joint gently to both of its mechanical stops, then leave the\n"
        "arm somewhere safe. The stop is where it resists — do not force it.\n"
        "Take the joints in any order; all of them are being recorded.\n"
        "\nPress Enter when every joint has been to both stops."
    )

    recorders = {j.name: SweepRecorder(j.name) for j in joints}
    table = _LiveTable("", f"  {'joint':<16}{'min':>6}{'now':>6}{'max':>6}{'span':>10}")
    done = _wait_for_enter()
    period = 1.0 / rate_hz
    misses = 0
    last: dict[str, int | None] = {j.name: None for j in joints}
    while not done.is_set():
        rows = []
        for joint in joints:
            counts = bus.read_position(joint.id)
            rec = recorders[joint.name]
            if counts is None:
                misses += 1
            else:
                rec.add(counts)
                last[joint.name] = counts
            rows.append(_range_row(joint.name, rec, counts))
        table.draw(rows)
        time.sleep(period)
    table.final([_range_row(j.name, recorders[j.name], last[j.name]) for j in joints])
    return recorders, misses


def _range_row(name: str, rec: SweepRecorder, now: int | None) -> str:
    if rec.samples == 0:
        return f"  {name:<16}{'-':>6}{'-':>6}{'-':>6}{'no readings':>10}"
    span = rec.unwrapped_span
    flag = f"  WRAP x{rec.wraps}" if rec.wraps else ""
    shown = f"{now:>6}" if now is not None else f"{'':>6}"
    return (
        f"  {name:<16}{rec.min_counts:>6}{shown}{rec.max_counts:>6}"
        f"{span * RAD_PER_COUNT:>8.2f} rad{flag}"
    )


def _warn_about_poses(cfg, calibrated) -> None:
    """Name the taught poses a changed zero invalidates, before emitting it."""
    shifts = {
        name: zero_shift(
            cfg.joint(name).zero_counts, cal.zero_counts, cfg.joint(name).invert
        )
        for name, cal in calibrated.items()
    }
    moved = {n: s for n, s in shifts.items() if s != 0.0}
    if not moved:
        print("\nzero is unchanged on every joint — taught poses still hold.")
        return

    taught = poses.load_poses()
    impact = pose_impact(taught, moved)
    if not impact:
        print(
            f"\nzero moved on {len(moved)} joint(s); no taught poses are affected "
            f"({len(taught)} pose(s) in {poses.poses_path()})."
        )
        return
    print(
        f"\nRE-TEACH THESE POSES. A pose is stored as radians from the zero, and "
        f"the\nzero moved on {len(moved)} joint(s), so these now name different "
        "physical\npositions. Paste the block below and `pixi run build` FIRST, "
        "then re-teach:"
    )
    for pose in sorted(impact):
        print(f"  pixi run arm-pose save {pose}")


def _select(cfg, spec_names: str):
    if not spec_names:
        return list(cfg.joints)
    wanted = [n.strip() for n in spec_names.split(",") if n.strip()]
    unknown = [n for n in wanted if n not in cfg.names]
    if unknown:
        raise SystemExit(f"unknown joint(s) {unknown}; have {cfg.names}")
    return [cfg.joint(n) for n in wanted]


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Guided full-range arm calibration (centre, then sweep)"
    )
    parser.add_argument("--robot-yaml", default="", help="override robot.yaml path")
    parser.add_argument(
        "--joints",
        default="",
        help="comma-separated subset to calibrate (default: all; the rest keep "
        "their current values)",
    )
    parser.add_argument(
        "--margin",
        type=float,
        default=DEFAULT_MARGIN,
        help=f"radians kept between each soft limit and the measured stop "
        f"(default {DEFAULT_MARGIN})",
    )
    parser.add_argument(
        "--skip-homing",
        action="store_true",
        help="skip phase 1: record ranges against the zeros already in "
        "robot.yaml and write nothing to the servos",
    )
    parser.add_argument(
        "--rate", type=float, default=20.0, help="encoder sample rate, Hz"
    )
    parser.add_argument("--yes", action="store_true", help="skip confirmations")
    parser.add_argument(
        "--no-record",
        action="store_true",
        help=f"do not write the measurements to {poses.mote_home()}/"
        "arm_calibration.yaml",
    )
    args = parser.parse_args()

    cfg = (
        config.ArmConfig.from_yaml_file(args.robot_yaml)
        if args.robot_yaml
        else config.load()
    )
    selected = _select(cfg, args.joints)

    bus = _open_bus(cfg)
    try:
        _run(bus, cfg, selected, args)
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
    finally:
        bus.close()


def _run(bus, cfg, selected, args) -> None:
    print(f"arm bus: {cfg.port} @ {cfg.baud_rate}")
    print(f"calibrating: {[j.name for j in selected]}")

    missing = [j.name for j in selected if not bus.ping(j.id)]
    if missing:
        raise SystemExit(
            f"joint(s) {missing} did not respond — fix wiring/IDs before "
            "calibrating (see `pixi run arm-check`)."
        )

    print(
        "\nThe arm will be made LIMP so you can move it by hand. Support it or "
        "rest it\nin a stable pose first — an unsupported arm falls when torque "
        "is released."
    )
    if not _confirm("release torque on all joints? [y/N] ", args.yes):
        raise SystemExit("aborted; nothing changed")
    for joint in cfg.joints:
        try:
            bus.set_torque(joint.id, False)
        except BusError as exc:
            print(f"  {joint.name}: {exc}")
    print("torque off — the arm is back-drivable.")

    offsets: dict[str, int] = {}
    if args.skip_homing:
        print("\n--skip-homing: keeping the zeros already in robot.yaml.")
        zeros = {j.name: j.zero_counts for j in selected}
        source = "kept from robot.yaml"
    else:
        offsets = _phase_offsets(bus, selected, args) or {}
        zeros = {j.name: CENTRE_COUNTS for j in selected}
        source = "centred by a homing offset"

    recorders, misses = _phase_ranges(bus, selected, args.rate)
    if misses:
        print(f"\n{misses} reading(s) did not come back — bus contention or wiring")

    calibrated: dict = {}
    chosen = {j.name for j in selected}
    failures: dict[str, str] = {
        j.name: "not selected this run" for j in cfg.joints if j.name not in chosen
    }
    for joint in selected:
        rec = recorders[joint.name]
        if rec.samples == 0:
            failures[joint.name] = "no encoder readings"
            continue
        try:
            calibrated[joint.name] = calibrate_joint(
                joint, rec.result(), zeros[joint.name], args.margin, source
            )
        except CalibrationError as exc:
            failures[joint.name] = exc.reason
            print(f"\n{joint.name} NOT CALIBRATED: {exc}")

    if not calibrated:
        raise SystemExit("\nno joint was calibrated — nothing to emit")

    print("\ncalibrated:")
    for name, cal in calibrated.items():
        print(
            f"  {name:<16} {cal.min_rad:+.3f} to {cal.max_rad:+.3f} rad "
            f"about zero {cal.zero_counts} ({cal.sweep.span_rad:.2f} rad swept)"
        )

    _warn_about_poses(cfg, calibrated)

    recorded = datetime.now(timezone.utc).strftime("measured %Y-%m-%d")
    print("\nPaste into robot.yaml's arm: section, then `pixi run build`:\n")
    print(joints_block(list(cfg.joints), calibrated, failures, recorded))

    real = {n: r for n, r in failures.items() if n in chosen}
    if real:
        print("\nNOT calibrated, kept their existing values:")
        for name, reason in sorted(real.items()):
            print(f"  {name:<16} {reason}")
    if not args.skip_homing:
        print(
            "\nThe servos' homing offsets have changed, so robot.yaml is now "
            "STALE.\nDo not run `pixi run arm` until you have pasted the block "
            "above and rebuilt:\nthe driver would read every angle against the "
            "old zero."
        )
    if not args.no_record:
        path = save_record(calibrated, recorded, offsets=offsets)
        print(f"\nmeasurements recorded in {path}")


if __name__ == "__main__":
    main()
