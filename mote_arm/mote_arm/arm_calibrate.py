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

It saves what it measured to ``$MOTE_HOME/arm.yaml`` — per-robot state, not the
repo. The zeros and limits describe one physical arm, so they do not belong in
``mote_description/config/robot.yaml``, which every robot shares and which is
read-only once the package is installed from a channel. That file keeps the
design (ids, names, direction, gains) and the defaults an uncalibrated arm
falls back to; ``mote_arm.config`` overlays this robot's measurements on top.

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
    calibration_document,
    centred_limits,
    homing_offset,
    limits_from_sweep,
    pose_impact,
    save_calibration,
    save_offsets_backup,
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

    # One line, not one per joint: every name here succeeded, and a joint that
    # did not has already stopped the run by name with the detail that matters.
    print(f"{len(written)} joint(s) centred and confirmed.")
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


def _go_limp(bus, cfg, args) -> None:
    """Make sure nothing is holding, warning only if something actually is.

    The arm is limp already in almost every case: this tool refuses to start
    while another process holds the port, so the driver is not running, and the
    driver leaves the arm limp on shutdown. The exception is a driver killed
    outright, which leaves torque enabled in servo RAM — so check rather than
    ask, and say something only when the arm is about to drop.
    """
    holding = [j.name for j in cfg.joints if bus.read_torque(j.id)]
    if holding:
        print(f"\n{', '.join(holding)} are holding and are about to go limp —")
        print("support the arm before continuing.")
        if not _confirm("ready? [y/N] ", args.yes):
            raise SystemExit("aborted; nothing changed")
    for joint in cfg.joints:
        try:
            bus.set_torque(joint.id, False)
        except BusError as exc:
            print(f"  {joint.name}: {exc}")


def _phase_ranges(bus, joints, rate_hz: float) -> tuple[dict, int]:
    """Record every joint's range at once while the operator moves them."""
    print("\n=== 1 of 2: sweep the joints ===")
    print(
        "Move each joint gently to both stops — the stop is where it resists, do\n"
        "not force it. Any order; all are recorded at once.\n"
        "\nPress Enter when every joint has been to both stops."
    )

    recorders = {j.name: SweepRecorder(j.name) for j in joints}
    table = _LiveTable(
        "", f"  {'joint':<16}{'now':>6}{'low':>7}{'high':>7}{'swept':>13}"
    )
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

    ``low`` and ``high`` are the two ends of *travel*, shown as well as the
    total because the total alone cannot tell you *which* end still needs
    reaching — you can see one end parked on a stop while the other has not got
    there yet. They come off the unwrapped stream, so they are real encoder
    positions even for a joint whose travel crosses zero, where the raw min and
    max would read 17 and 4093 and describe the encoder rather than the joint.
    Such a joint reads ``low`` numerically larger than ``high``, which is what
    running up through zero and out the other side looks like.
    """
    if rec.samples == 0:
        return f"  {name:<16}{'-':>6}{'-':>7}{'-':>7}   no readings"
    low, high = rec.travel_ends
    span = rec.unwrapped_span * RAD_PER_COUNT
    return (
        f"  {name:<16}{now if now is not None else '':>6}"
        f"{low:>7}{high:>7}{span:>9.2f} rad"
    )


def _save(cfg, calibrated, offsets, recorded) -> bool:
    """Write this robot's calibration.

    The numbers themselves are not reprinted: the swept ranges were on screen a
    moment ago, the limits are those pulled inward by ``--margin``, and the file
    is the record — it keeps each value next to the measurement it came from.
    """
    document = calibration_document(
        list(cfg.joints), calibrated, recorded, offsets=offsets
    )

    # Refuse to save anything the config layer would not load: this file
    # supplies the soft limits that stop the arm.
    try:
        config.apply_calibration(cfg, document)
    except Exception as exc:  # noqa: BLE001 - any validation failure is fatal
        raise SystemExit(f"refusing to save: {exc}")

    saved = save_calibration(document)
    print(f"\nsaved to {saved}")
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

    _go_limp(bus, cfg, args)

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

    skipped = {n: r for n, r in failures.items() if n in chosen}
    if skipped:
        print("\nnot calibrated, keeping the packaged defaults:")
        for name, reason in sorted(skipped.items()):
            print(f"  {name:<16} {reason}")

    written = _save(cfg, calibrated, offsets, recorded)

    if not written and not args.skip_homing:
        print(
            "\nWARNING: the servo zeros moved but were not saved, so the arm's"
            "\nconfig no longer matches its hardware. Do not run `pixi run arm`"
            "\nuntil this is re-run or the offsets are restored."
        )
    _migrate_poses(cfg, calibrated)
    _next_steps(written)


def _migrate_poses(cfg, calibrated) -> None:
    """Rewrite taught poses about the new zeros, so none has to be re-taught.

    The shift is exactly what the calibration already computed, and applying it
    keeps every pose pointing at the same physical position. A pose that would
    land outside the new soft limits is reported rather than silently clamped —
    that means it was taught against something the arm can no longer reach, and
    is the operator's call.
    """
    shifts = {
        name: shift
        for name, cal in calibrated.items()
        if (
            shift := zero_shift(
                cfg.joint(name).zero_counts, cal.zero_counts, cfg.joint(name).invert
            )
        )
    }
    taught = poses.load_poses()
    if not shifts or not taught:
        return

    affected = pose_impact(taught, shifts)
    if not affected:
        return

    migrated = poses.shift_poses(taught, shifts)
    limits = {name: (cal.min_rad, cal.max_rad) for name, cal in calibrated.items()}
    unreachable = {
        pose: [
            joint
            for joint, value in joints.items()
            if joint in limits and not limits[joint][0] <= value <= limits[joint][1]
        ]
        for pose, joints in migrated.items()
    }
    unreachable = {p: j for p, j in unreachable.items() if j}

    path = poses.save_poses(migrated)
    print(
        f"\n{len(affected)} taught pose(s) re-expressed about the new zeros "
        f"({path});\nthey still point where they always did. Previous file kept "
        "as .bak."
    )
    for pose in sorted(affected):
        print(f"  {pose}")
    if unreachable:
        print("\nbut these now sit outside the calibrated limits:")
        for pose, joints in sorted(unreachable.items()):
            print(f"  {pose:<16} {', '.join(joints)}")
        print("  They were taught somewhere the arm cannot reach — re-teach them.")


def _next_steps(written) -> None:
    if written:
        print("\n  pixi run arm-check          # rad reads ~0 at mid-travel")


if __name__ == "__main__":
    main()
