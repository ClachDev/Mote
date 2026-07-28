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

It writes the measured limits into ``mote_description/config/robot.yaml`` — the
shared hardware description in the repo — after showing a diff and asking. That
is NOT ``$MOTE_HOME/robot.yaml``, which holds this robot's fleet identity and is
unrelated to the arm.

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
import difflib
import os
import pathlib
import sys
import threading
import time
from datetime import datetime, timezone

import yaml

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
    splice_joints_block,
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
    print("\n=== 2 of 2: centre the zeros ===")
    print("Setting each joint's 0 rad to the middle of its swept range. Leave the")
    print("arm where it is — this changes what the encoders report, not the arm.")

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

    print(f"\nWrites servo EEPROM on {len(stale)} joint(s) — a persistent change.")
    if not _confirm("write? [y/N] ", args.yes):
        raise SystemExit("aborted; nothing written")

    # Snapshot before the first write. These values exist nowhere but the
    # servos, so without this a run that dies partway leaves an arm that cannot
    # be put back — which is exactly what happened once.
    backup = save_offsets_backup(
        existing,
        {j.name: j.id for j in joints},
        datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%SZ"),
    )
    print(f"backed up to {backup} (`pixi run arm-offsets restore` undoes this)")

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
        print(f"  {joint.name:<16} written and confirmed")

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
    # Let the EEPROM write settle before believing anything read off this bus,
    # then require two agreeing reads: a single one here returned the offset
    # register's own value, which looked exactly like a servo misbehaving.
    time.sleep(0.25)
    now = bus.read_position_settled(joint.id)
    if now is None:
        return (
            f"{joint.name}: could not get two agreeing position reads after "
            "writing, so the write could not be confirmed either way"
        )
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
    print("\n=== 1 of 2: sweep the joints ===")
    print(
        "Move each joint gently to both stops — the stop is where it resists, do\n"
        "not force it. Any order; all are recorded at once.\n"
        "\nPress Enter when every joint has been to both stops."
    )

    recorders = {j.name: SweepRecorder(j.name) for j in joints}
    table = _LiveTable("", f"  {'joint':<16}{'now':>6}{'swept':>13}")
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
    """One row, the same shape for every joint.

    Only the swept range is shown, not the raw encoder min/max. Those are
    meaningless for a joint whose travel crosses the encoder's zero — its raw
    range reads 17 to 4093 — and showing a blank for that joint alone made it
    look special when it is not: it gets centred like the others and ends up
    with an ordinary band. The range comes off the unwrapped stream, so it is
    the true travel for every joint, and it is the number that tells you
    whether you have reached both stops yet.
    """
    if rec.samples == 0:
        return f"  {name:<16}{'-':>6}   no readings"
    return f"  {name:<16}{now if now is not None else '':>6}{rec.unwrapped_span * RAD_PER_COUNT:>9.2f} rad"


def _invalidated_poses(cfg, calibrated) -> list[str]:
    """Taught poses that a changed zero has silently redefined.

    A pose is stored as radians about the zero, so moving the zero changes which
    physical position each stored number names. Pure query: the caller decides
    how loudly to say it.
    """
    moved = {
        name: shift
        for name, cal in calibrated.items()
        if (
            shift := zero_shift(
                cfg.joint(name).zero_counts, cal.zero_counts, cfg.joint(name).invert
            )
        )
    }
    return sorted(pose_impact(poses.load_poses(), moved))


def _robot_yaml_source(args) -> str | None:
    """The robot.yaml to edit: the source file, not the installed copy.

    `colcon --symlink-install` makes the installed config a symlink back into
    the source tree, so resolving it lands on the file worth editing. Without
    symlink-install it lands inside `install/`, where an edit would be silently
    thrown away by the next build — so that case is refused rather than written.
    """
    path = args.robot_yaml or config.default_robot_yaml()
    real = os.path.realpath(path)
    if f"{os.sep}install{os.sep}" in real:
        return None
    return real


def _apply_to_robot_yaml(block: str, args) -> bool:
    """Write the block into robot.yaml, showing the diff and asking first.

    Returns whether robot.yaml now matches the calibration. Printing a block for
    the operator to retype was the earlier behaviour and was wrong: phase 2 moves
    the servos' zeros, so until the file is updated it actively misdescribes the
    hardware. The block is still printed on any path that cannot write, so a
    calibration is never simply lost.

    This edits ``mote_description/config/robot.yaml`` — the shared hardware
    description in the repo. It is NOT ``$MOTE_HOME/robot.yaml``, which holds
    this robot's fleet identity and has nothing to do with the arm.
    """
    path = _robot_yaml_source(args)
    if path is None:
        print(
            "\nrobot.yaml resolves inside install/ (not a symlink-install), so an "
            "edit\nthere would be lost on the next build. Paste this instead:\n"
        )
        print(block)
        return False

    original = pathlib.Path(path).read_text()
    try:
        updated = splice_joints_block(original, block)
    except CalibrationError as exc:
        print(f"\n{exc}\n")
        print(block)
        return False

    # Never write a robot.yaml that would not load. This is the config every
    # other arm tool reads, including the soft limits that stop the arm.
    try:
        reparsed = config.ArmConfig.from_dict(yaml.safe_load(updated))
    except Exception as exc:  # noqa: BLE001 - any failure to reload is fatal
        raise SystemExit(
            f"refusing to write: the result would not load ({exc}). "
            "Paste the block by hand and check it."
        )

    diff = list(
        difflib.unified_diff(
            original.splitlines(),
            updated.splitlines(),
            fromfile=path,
            tofile=f"{path} (calibrated)",
            lineterm="",
            n=1,
        )
    )
    if not diff:
        print(f"\n{path} already matches this calibration — nothing to write.")
        return True

    print(f"\n{path}\n")
    for line in diff:
        print(f"  {line}")

    if not _confirm("\nwrite? [y/N] ", args.yes):
        print("not written. The block above can still be pasted by hand:\n")
        print(block)
        return False

    # Write via a temporary file in the same directory, so an interrupted write
    # cannot leave a half-updated robot.yaml behind.
    target = pathlib.Path(path)
    tmp = target.with_suffix(target.suffix + ".tmp")
    tmp.write_text(updated)
    os.replace(tmp, target)
    print(f"written — {len(reparsed.joints)} joints. Review with `git diff`.")
    return True


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
    missing = [j.name for j in selected if not bus.ping(j.id)]
    if missing:
        raise SystemExit(
            f"joint(s) {missing} did not respond — fix wiring/IDs before "
            "calibrating (see `pixi run arm-check`)."
        )

    print("\nThe arm will go LIMP so you can move it by hand — support it first,")
    print("an unsupported arm falls when torque is released.")
    if not _confirm("release torque? [y/N] ", args.yes):
        raise SystemExit("aborted; nothing changed")
    for joint in cfg.joints:
        try:
            bus.set_torque(joint.id, False)
        except BusError as exc:
            print(f"  {joint.name}: {exc}")
    print("torque off.")

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

    recorded = datetime.now(timezone.utc).strftime("measured %Y-%m-%d")
    block = joints_block(list(cfg.joints), calibrated, failures, recorded)

    skipped = {n: r for n, r in failures.items() if n in chosen}
    if skipped:
        print("\nnot calibrated, keeping their existing values:")
        for name, reason in sorted(skipped.items()):
            print(f"  {name:<16} {reason}")

    # The diff is the report: it shows every limit that changed, in the file
    # they will live in, so listing them again beforehand is noise.
    written = _apply_to_robot_yaml(block, args)

    if not args.no_record:
        save_record(calibrated, recorded, offsets=offsets)

    if not written and not args.skip_homing:
        print(
            "\nWARNING: the zeros moved but robot.yaml was not updated. Do not run"
            "\n`pixi run arm` until the block above is in place — every angle it"
            "\nreports would be measured against the old zero."
        )
    _next_steps(cfg, calibrated, written)


def _next_steps(cfg, calibrated, written) -> None:
    """What the operator has to do now, and nothing else."""
    steps = []
    if written:
        steps.append("git diff mote_description/config/robot.yaml   # review")
    stale = _invalidated_poses(cfg, calibrated)
    steps += [f"pixi run arm-pose save {name}" for name in stale]
    if not steps:
        return
    if stale:
        print(
            f"\n{len(stale)} taught pose(s) are now wrong — a pose is stored as "
            "radians\nfrom the zero, and the zeros moved. Re-teach them:"
        )
    print()
    for step in steps:
        print(f"  {step}")


if __name__ == "__main__":
    main()
