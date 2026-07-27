"""Guided range calibration: sweep each joint to its stops, emit robot.yaml.

Walks the six joints in servo-command order. For each, the operator moves the
limp joint gently to both mechanical stops by hand while the tool watches the
encoder live; it then emits an ``arm.joints`` block whose soft limits are the
measured travel pulled inward by a margin.

    pixi run arm-calibrate                 # home = the mid-point of each sweep
    pixi run arm-calibrate -- --home capture   # pose a zero per joint instead
    pixi run arm-calibrate -- --joints wrist_roll,gripper   # redo two joints

This replaces ``arm-pose limits`` as the way soft limits are set. That command
widens *outward* from poses a human has already vetted, which can only ever
describe where the arm has been — it never learns where the stops are, and a
joint that barely moved between two taught poses ends up with a near-zero band.
Calibration measures the stops directly and works inward from them.

**It opens the serial bus directly**, like ``arm_check`` and ``arm_gains``, so
run it with the driver stopped. It is a bus owner rather than an ``arm_driver``
client for two reasons: the driver reports radians about the very ``home`` this
tool exists to replace, so a client would be measuring against the offset under
test; and the arm must be limp and back-drivable throughout, which is the
opposite of what the driver is for. The one write it makes is torque *off* — it
never commands a goal.
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
    calibrate_joint,
    home_shift,
    joints_block,
    pose_impact,
    save_record,
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
    """Event set when the operator presses Enter, so the sweep can poll on."""
    done = threading.Event()

    def wait() -> None:
        try:
            input()
        except EOFError:
            pass
        done.set()

    threading.Thread(target=wait, daemon=True).start()
    return done


def _sweep(bus, joint, rate_hz: float) -> tuple[SweepRecorder, int]:
    recorder = SweepRecorder(joint.name)
    done = _wait_for_enter()
    period = 1.0 / rate_hz
    misses = 0
    while not done.is_set():
        counts = bus.read_position(joint.id)
        if counts is None:
            misses += 1
        else:
            recorder.add(counts)
            span = recorder.unwrapped_span
            flag = f"  WRAPPED x{recorder.wraps}" if recorder.wraps else ""
            print(
                f"\r  now {counts:>4}   min {recorder.min_counts:>4}   "
                f"max {recorder.max_counts:>4}   span {span:>4} counts "
                f"({span * RAD_PER_COUNT:5.3f} rad){flag}   ",
                end="",
                flush=True,
            )
        time.sleep(period)
    print()
    return recorder, misses


def _capture_home(bus, joint) -> int | None:
    print(
        f"  now move {joint.name} to the pose that should read 0 rad, then press Enter."
    )
    input()
    for _ in range(5):
        counts = bus.read_position(joint.id)
        if counts is not None:
            return counts
        time.sleep(0.1)
    return None


def _report_sweep(recorder: SweepRecorder, misses: int) -> None:
    span = recorder.unwrapped_span
    travel = f"{span} counts ({span * RAD_PER_COUNT:.3f} rad)"
    if recorder.wraps:
        print(
            f"  raw readings {recorder.min_counts}-{recorder.max_counts}, "
            f"true travel {travel}, from {recorder.samples} samples"
        )
        print(
            f"  WRAP: the reading jumped past 0/4095 {recorder.wraps} time(s), so "
            "the raw range is not this joint's range."
        )
    else:
        print(
            f"  swept {recorder.min_counts}-{recorder.max_counts}, {travel}, "
            f"from {recorder.samples} samples"
        )
    if misses:
        print(f"  {misses} reading(s) did not come back — bus contention or wiring")


def _warn_about_poses(cfg, calibrated) -> None:
    """Name the taught poses a re-home would invalidate, before emitting it."""
    shifts = {
        name: home_shift(
            cfg.joint(name).home_counts, cal.home_counts, cfg.joint(name).invert
        )
        for name, cal in calibrated.items()
    }
    moved = {n: s for n, s in shifts.items() if s != 0.0}
    if not moved:
        print(
            "\nhome is unchanged on every calibrated joint — taught poses still hold."
        )
        return

    print(f"\nhome MOVED on {len(moved)} joint(s):")
    for name, shift in sorted(moved.items()):
        old = cfg.joint(name).home_counts
        print(
            f"  {name:<16} {old} -> {calibrated[name].home_counts} counts "
            f"({shift:+.4f} rad)"
        )

    taught = poses.load_poses()
    impact = pose_impact(taught, moved)
    if not impact:
        print(
            f"  no taught poses are affected ({len(taught)} pose(s) in "
            f"{poses.poses_path()})"
        )
        return
    print(
        "\nPOSES INVALIDATED: poses are stored in radians about home, so after "
        "pasting\nthis block the poses below name different physical positions "
        "and must be\nre-taught (`pixi run arm-pose save <name>`). The shift per "
        "joint is shown."
    )
    for pose, affected in sorted(impact.items()):
        joints = ", ".join(f"{n} {s:+.4f} rad" for n, s in sorted(affected.items()))
        print(f"  {pose:<16} {joints}")


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
        description="Guided full-range arm calibration (sweep to the stops)"
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
        "--home",
        choices=("mid", "capture"),
        default="mid",
        help="'mid' derives home from the middle of each sweep; 'capture' asks "
        "you to pose a zero per joint (default: mid)",
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
        print("\ninterrupted — nothing was written", file=sys.stderr)
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

    calibrated: dict = {}
    chosen = {j.name for j in selected}
    failures: dict[str, str] = {
        j.name: "not selected this run" for j in cfg.joints if j.name not in chosen
    }
    for index, joint in enumerate(selected, 1):
        print(f"\n[{index}/{len(selected)}] {joint.name} (id {joint.id})")
        print(
            "  Move it gently to one mechanical stop, then to the other, then "
            "back to a\n  safe resting position. Do not force it — the stop is "
            "where it resists.\n  Press Enter when done."
        )
        recorder, misses = _sweep(bus, joint, args.rate)
        if recorder.samples == 0:
            failures[joint.name] = "no encoder readings"
            print("  no readings — skipping this joint")
            continue
        _report_sweep(recorder, misses)
        if recorder.wraps:
            # Say so before asking for a zero pose: no home can rescue a wrapped
            # sweep, and prompting for one would waste the operator's time.
            failures[joint.name] = "sweep crossed the encoder 0/4095 boundary"
            print(
                "  NOT CALIBRATED: re-home the servo so this joint's mid-range "
                "sits away\n  from the 0/4095 boundary (see mote_arm/BENCH.md), "
                "then sweep it again."
            )
            continue

        home_counts = None
        if args.home == "capture":
            home_counts = _capture_home(bus, joint)
            if home_counts is None:
                failures[joint.name] = "could not read the captured home pose"
                print("  could not read a position — skipping this joint")
                continue
            print(f"  home captured at {home_counts} counts")

        try:
            calibrated[joint.name] = calibrate_joint(
                joint, recorder.result(), home_counts, args.margin
            )
        except CalibrationError as exc:
            failures[joint.name] = exc.reason
            print(f"  NOT CALIBRATED: {exc}")
            continue
        cal = calibrated[joint.name]
        print(
            f"  limits {cal.min_rad:+.3f} to {cal.max_rad:+.3f} rad about "
            f"home {cal.home_counts} ({cal.home_source})"
        )

    if not calibrated:
        raise SystemExit("\nno joint was calibrated — nothing to emit")

    _warn_about_poses(cfg, calibrated)

    recorded = datetime.now(timezone.utc).strftime("measured %Y-%m-%d")
    print("\nPaste into robot.yaml's arm: section, then `pixi run build`:\n")
    print(joints_block(list(cfg.joints), calibrated, failures, recorded))

    if failures:
        print(f"\n{len(failures)} joint(s) kept their existing values:")
        for name, reason in sorted(failures.items()):
            print(f"  {name:<16} {reason}")
    if not args.no_record:
        path = save_record(calibrated, recorded)
        print(f"\nmeasurements recorded in {path}")


if __name__ == "__main__":
    main()
