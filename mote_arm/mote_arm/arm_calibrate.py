"""Guided range calibration: sweep every joint, then move its zero to the middle.

Two phases, but the operator only does one thing:

1. **Ranges.** You move every joint to both of its mechanical stops while a
   single live table records all six at once. One Enter ends it.
2. **Zeros.** Each joint's zero is moved to the *measured* middle of the range
   just swept, by writing the servo's position-correction register. The arm can
   be left wherever it ended up. It writes EEPROM, so it asks first.

LeRobot's own flow asks the operator to first hold the arm with every joint at
mid-travel and takes the zero from that pose. That works, but it is an awkward,
unbalanced position to hold, and eyeballing the middle is less accurate than the
measurement the sweep is about to take anyway. Taking the centre from the sweep
gives a better zero for less effort — and it still works for a joint that
crossed the encoder wrap during the sweep, because the recorder unwraps.

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
    DEFAULT_MARGIN,
    CalibrationError,
    SweepRecorder,
    calibrate_centred,
    calibrate_joint,
    centred_limits,
    homing_offset,
    joints_block,
    limits_from_sweep,
    pose_impact,
    save_offsets_backup,
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


def _phase_centre(bus, joints, recorders, args) -> dict[str, int]:
    """Move each joint's zero to the middle of the range just swept.

    The centre comes from the sweep, not from a pose the operator has to hold.
    Holding all six joints at mid-travel at once is an awkward, unbalanced
    position, and eyeballing it is less accurate than the measurement already
    taken — so the arm can be left wherever it ended up.
    """
    print("\n=== Phase 2 of 2: centre the zeros ===")
    print(
        "Each joint's 0 rad is being moved to the middle of the range you just\n"
        "swept. Leave the arm where it is; this only changes what the encoders\n"
        "report, not where the arm is."
    )

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
    print(f"\n{'joint':<16}{'mid-travel':>11}{'offset':>8}{'->':>4}{'new':>7}")
    for joint in joints:
        centre = recorders[joint.name].result().measured_centre
        wanted[joint.name] = homing_offset(centre, existing[joint.name])
        print(
            f"{joint.name:<16}{centre:>11}"
            f"{existing[joint.name]:>8}{'->':>4}{wanted[joint.name]:>7}"
        )

    stale = {n: v for n, v in wanted.items() if v != existing[n]}
    if not stale:
        print("\nevery servo is already centred — nothing to write.")
        return wanted

    print(
        f"\nThis writes the position-correction register of {len(stale)} servo(s) "
        "— EEPROM,\na persistent hardware change. Afterwards the zero counts in "
        "robot.yaml are\nstale until you paste the block this prints at the end."
    )
    if not _confirm("write homing offsets? [y/N] ", args.yes):
        raise SystemExit("aborted; nothing written")

    # Snapshot before the first write. These values exist nowhere but the
    # servos, so without this a run that dies partway leaves an arm that cannot
    # be put back — which is exactly what happened once.
    backup = save_offsets_backup(
        existing,
        {j.name: j.id for j in joints},
        datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%SZ"),
    )
    print(f"previous offsets backed up to {backup}")
    print("  (`pixi run arm-offsets restore` puts them back)")

    written: list[str] = []
    for joint in joints:
        if joint.name not in stale:
            continue
        # Read again here rather than reusing the pre-prompt reading: the arm is
        # limp and may have sagged while the operator answered.
        was = bus.read_position(joint.id)
        ok = bus.write_homing_offset(joint.id, wanted[joint.name])
        if not ok:
            _abort_partial(written, backup, f"{joint.name}: write not verified")
        moved = _reading_moved_as_expected(
            bus, joint, was, existing[joint.name], wanted[joint.name]
        )
        if moved is not None:
            _abort_partial(written, backup, moved)
        written.append(joint.name)
        print(f"  {joint.name:<16} written, verified, and reading confirmed")

    print(
        f"\noffsets verified: {len(written)} reading(s) moved by exactly what "
        "was written."
    )
    return wanted


def _abort_partial(written: list[str], backup, why: str) -> None:
    """Stop the moment a write cannot be trusted, saying what state we are in."""
    raise SystemExit(
        f"\nSTOPPED: {why}\n"
        f"{len(written)} servo(s) were changed before this: {written or 'none'}.\n"
        f"The arm is part-way through a calibration. Put it back with:\n"
        f"    pixi run arm-offsets restore     # from {backup}\n"
        "then investigate before re-running."
    )


def _reading_moved_as_expected(bus, joint, was, existing, wanted) -> str | None:
    """None if the joint's reading shifted by the offset delta; else why not.

    Checked per joint, immediately after its write, so a servo that does not
    behave as assumed stops the run rather than being discovered at the end
    with five more already changed.
    """
    if was is None:
        return None
    time.sleep(0.1)
    now = bus.read_position(joint.id)
    if now is None:
        return f"{joint.name}: could not read its position back after writing"
    expected = (was - (wanted - existing)) % 4096
    error = min(abs(now - expected), 4096 - abs(now - expected))
    if error <= 25:
        return None
    return (
        f"{joint.name}: reading was {was}, expected {expected} after the offset "
        f"write, but reads {now}. Either the arm moved while limp, or this servo "
        "does not apply the correction register as assumed."
    )


def _phase_ranges(bus, joints, rate_hz: float) -> tuple[dict, int]:
    """Record every joint's range at once while the operator moves them."""
    print("\n=== Phase 1 of 2: record the ranges ===")
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


def _check_sweep(joint, sweep, args) -> None:
    """Raise if this sweep cannot become limits, before any EEPROM is written."""
    if args.skip_homing:
        limits_from_sweep(sweep, joint.zero_counts, joint.invert, args.margin)
    else:
        centred_limits(sweep, joint.invert, args.margin)


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

    recorders, misses = _phase_ranges(bus, selected, args.rate)
    if misses:
        print(f"\n{misses} reading(s) did not come back — bus contention or wiring")

    # Work out which sweeps are usable before touching EEPROM: a joint that
    # cannot be calibrated should not have its zero moved either.
    chosen = {j.name for j in selected}
    failures: dict[str, str] = {
        j.name: "not selected this run" for j in cfg.joints if j.name not in chosen
    }
    usable = []
    for joint in selected:
        rec = recorders[joint.name]
        if rec.samples == 0:
            failures[joint.name] = "no encoder readings"
            continue
        try:
            _check_sweep(joint, rec.result(), args)
        except CalibrationError as exc:
            failures[joint.name] = exc.reason
            print(f"\n{joint.name} NOT CALIBRATED: {exc}")
            continue
        usable.append(joint)

    if not usable:
        raise SystemExit("\nno joint produced a usable sweep — nothing to emit")

    offsets: dict[str, int] = {}
    calibrated: dict = {}
    if args.skip_homing:
        print("\n--skip-homing: keeping the zeros already in robot.yaml.")
        for joint in usable:
            calibrated[joint.name] = calibrate_joint(
                joint,
                recorders[joint.name].result(),
                joint.zero_counts,
                args.margin,
                "kept from robot.yaml",
            )
    else:
        offsets = _phase_centre(bus, usable, recorders, args)
        for joint in usable:
            calibrated[joint.name] = calibrate_centred(
                joint, recorders[joint.name].result(), args.margin
            )

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
