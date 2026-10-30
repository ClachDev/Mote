"""Show or apply the arm servos' position-loop gains.

The gains live in each servo's EEPROM, which makes them invisible config: swap a
servo and the tuning silently reverts. ``robot.yaml``'s ``arm.gains`` is the
source of truth, and this tool reconciles the hardware with it.

    pixi run arm-setup gains show     # read-only comparison against robot.yaml
    pixi run arm-setup gains apply    # write robot.yaml's gains, verifying each servo
    pixi run arm-setup gains sweep    # measure a step response across candidate gains

Opens the bus directly, so run it with the driver stopped. ``apply`` and
``sweep`` write EEPROM — a persistent change — so they ask first unless
``--yes`` is given, and report success only for servos whose read-back confirms
the new values.

``sweep`` is how the committed gains were chosen rather than guessed: it drives
one joint through the same step under each candidate gain set, scores what the
joint did (``step_response.py``), and restores the gains it started with. It
moves the arm, so it is a bench tool — clear the arm's path first.
"""

from __future__ import annotations

import json
import time
from datetime import datetime, timezone
from pathlib import Path

from mote_arm.bus import BusError
from mote_arm.poses import mote_home
from mote_arm.step_response import Sample, StepMetrics, droop_verdict, summarise

# A joint whose servo is this hot stops the sweep: the gains being measured are
# the ones that heat it, so pressing on would both risk the servo and score the
# later trials against a different machine than the earlier ones.
MAX_TEMP_C = 55


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


def _parse_ints(text: str, what: str) -> list[int]:
    values = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        try:
            value = int(part)
        except ValueError:
            raise SystemExit(f"{what}: {part!r} is not an integer")
        if not 0 <= value <= 254:
            raise SystemExit(f"{what}: {value} outside the servo range 0-254")
        values.append(value)
    if not values:
        raise SystemExit(f"{what}: no values given")
    return values


def _read_rad(bus, joint) -> float | None:
    counts = bus.read_position(joint.id)
    return None if counts is None else joint.counts_to_rad(counts)


def _drive_to(
    bus, cfg, joint, rad: float, settle_s: float, speed=None, acc=None
) -> None:
    bus.write_goal(
        joint.id,
        joint.rad_to_counts(rad),
        cfg.moving_speed if speed is None else speed,
        cfg.moving_acc if acc is None else acc,
    )
    time.sleep(settle_s)


def _apply_gains_for_trial(bus, cfg, joint, kp: int, kd: int, ki: int) -> bool:
    """Put the servo on the trial's gains, torque cycled so they are in force.

    The gains are EEPROM registers, and whether a servo already holding torque
    picks up a change mid-hold is exactly the kind of thing this tool exists to
    stop us assuming: a servo that latched its gains at torque-enable would run
    every trial at the same gain and report a droop that does not respond to kp.
    So each trial drops torque, writes, and re-enables against the joint's
    *present* position — seeded first, or the servo drives to whatever stale
    goal its register holds.

    The joint is briefly limp, which is the same condition the driver starts in:
    the arm must be resting in a pose it holds unsupported before a sweep.
    """
    bus.set_torque(joint.id, False)
    if not bus.write_gains(joint.id, kp, kd, ki):
        return False
    counts = bus.read_position(joint.id)
    if counts is None:
        return False
    bus.write_goal(joint.id, counts, cfg.moving_speed, cfg.moving_acc)
    bus.set_torque(joint.id, True)
    return True


def _run_trial(
    bus, cfg, joint, start_rad, goal_rad, hold_s, rate_hz, speed=None, acc=None
) -> list[Sample]:
    """Command one step and sample the joint until the hold ends.

    Sampling is paced against a monotonic clock rather than by sleeping a fixed
    interval, so a slow bus transaction shortens the next sleep instead of
    stretching the whole trace and quietly changing what a timestamp means.

    The return to the start uses the config's gentle speed whatever the trial
    runs at: only the measured step should be aggressive.
    """
    _drive_to(bus, cfg, joint, start_rad, settle_s=1.5)

    period = 1.0 / rate_hz
    samples: list[Sample] = []
    bus.write_goal(
        joint.id,
        joint.rad_to_counts(goal_rad),
        cfg.moving_speed if speed is None else speed,
        cfg.moving_acc if acc is None else acc,
    )
    t0 = time.monotonic()
    next_sample = t0
    while True:
        now = time.monotonic()
        elapsed = now - t0
        if elapsed > hold_s:
            break
        reading = bus.read_position_load(joint.id)
        if reading is not None:
            counts, load = reading
            samples.append(
                Sample(t=elapsed, rad=joint.counts_to_rad(counts), load=load)
            )
        next_sample += period
        remaining = next_sample - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)
    return samples


def _cmd_sweep(cfg, bus, args) -> None:
    try:
        joint = cfg.joint(args.joint)
    except KeyError:
        raise SystemExit(f"no arm joint named {args.joint!r} — joints are {cfg.names}")

    original = bus.read_gains(joint.id)
    if original is None:
        raise SystemExit(
            f"joint {joint.name!r} (id {joint.id}) did not answer a gain read — "
            "check the arm is attached and powered (`pixi run arm-setup check`)"
        )

    # The driver is the only thing that normally fixes a servo's mode, and this
    # tool runs with the driver stopped. A servo left in wheel mode — a re-IDed
    # ex-wheel servo is exactly the case robot.yaml's ID guard exists for — obeys
    # a position goal as a *speed*, so the step would become continuous rotation.
    if not bus.ensure_position_mode(joint.id):
        raise SystemExit(
            f"joint {joint.name!r} (id {joint.id}) is not confirmed in position "
            "mode, so a position goal could spin it continuously instead of "
            "stepping. Check it with `pixi run arm-setup check`; a servo that cannot "
            "be read is left untouched rather than blind-written."
        )

    start_rad = _read_rad(bus, joint)
    if start_rad is None:
        raise SystemExit(f"joint {joint.name!r}: cannot read its position")

    goal_rad = joint.clamp_rad(start_rad + args.step)
    achievable = goal_rad - start_rad
    if abs(achievable) < 0.5 * abs(args.step):
        raise SystemExit(
            f"joint {joint.name!r} is at {start_rad:+.3f} rad, so a step of "
            f"{args.step:+.3f} clamps to {achievable:+.3f} against the soft "
            f"limits [{joint.min_rad:+.3f}, {joint.max_rad:+.3f}]. Move the "
            "joint away from its limit, or sweep with a smaller --step: trials "
            "are only comparable if every one commands the same travel."
        )

    grid = [
        (kp, kd, ki)
        for kp in _parse_ints(args.kp, "--kp")
        for ki in _parse_ints(args.ki, "--ki")
        for kd in _parse_ints(args.kd if args.kd else str(cfg.gains.kd), "--kd")
    ]

    print(f"joint {joint.name} (id {joint.id}) on {cfg.port}")
    print(f"gains now: kp={original[0]} kd={original[1]} ki={original[2]}")
    print(
        f"step: {start_rad:+.3f} -> {goal_rad:+.3f} rad "
        f"({achievable:+.3f}), holding {args.hold:.1f}s at {args.rate:.0f} Hz"
    )
    print(
        f"{len(grid)} trial(s): "
        + ", ".join(f"kp={p}/kd={d}/ki={i}" for p, d, i in grid)
    )
    print(
        "\nTHIS MOVES THE ARM and writes servo EEPROM. Only this joint is "
        "torqued; the rest stay limp, and this one goes briefly limp between "
        "trials — so the arm must already be resting in a pose it holds "
        "unsupported. Clear its path first."
    )
    if not args.yes:
        if input("proceed? [y/N] ").strip().lower() not in ("y", "yes"):
            print("aborted; nothing written, nothing moved")
            return

    results = []
    aborted = None
    try:
        for kp, kd, ki in grid:
            if not _apply_gains_for_trial(bus, cfg, joint, kp, kd, ki):
                aborted = f"could not verify kp={kp} kd={kd} ki={ki} on the servo"
                break

            samples = _run_trial(
                bus,
                cfg,
                joint,
                start_rad,
                goal_rad,
                args.hold,
                args.rate,
                speed=args.speed,
                acc=args.acc,
            )
            if not samples:
                aborted = f"kp={kp} kd={kd} ki={ki}: the joint returned no readings"
                break

            metrics = summarise(samples, start_rad, goal_rad)
            health = bus.read_health(joint.id)
            temp = health.temperature if health else None
            volts = health.voltage if health else None
            results.append(
                {
                    "kp": kp,
                    "kd": kd,
                    "ki": ki,
                    "metrics": metrics.as_dict(),
                    "temperature_c": temp,
                    "voltage": volts,
                    "trace": [
                        {"t": round(s.t, 4), "rad": round(s.rad, 5), "load": s.load}
                        for s in samples
                    ],
                }
            )
            print(
                f"  kp={kp:<4} kd={kd:<4} ki={ki:<4} "
                f"error {metrics.abs_error:.3f} rad  "
                f"reached {100 * metrics.travel_fraction:5.1f}%  "
                f"load {metrics.hold_load:4.0f}  "
                f"ripple {metrics.ripple_counts:4.1f} counts  "
                f"reversals {metrics.reversals:<3} "
                f"{temp if temp is not None else '?'}C"
            )

            if temp is not None and temp >= args.max_temp:
                aborted = f"servo reached {temp}C (limit {args.max_temp}C)"
                break

            _drive_to(bus, cfg, joint, start_rad, settle_s=1.5)
    except KeyboardInterrupt:
        aborted = "interrupted by the operator"
    finally:
        # Whatever happened, the joint must not be left torqued at a swept gain:
        # put the position back, the EEPROM back, and the torque off. Each step
        # is guarded on its own — a failed drive-back must not cost the torque
        # drop, which is the one that decides how the arm is left.
        try:
            _drive_to(bus, cfg, joint, start_rad, settle_s=1.0)
        except BusError as exc:
            print(f"\nWARNING: could not drive back to the start: {exc}")
        try:
            bus.set_torque(joint.id, False)
        except BusError as exc:
            print(
                f"\nWARNING: the joint is still holding torque ({exc}) — "
                "power-cycle the servo bus before handling the arm"
            )
        restored = bus.write_gains(joint.id, *original)
        print(
            f"\nrestored kp={original[0]} kd={original[1]} ki={original[2]}"
            if restored
            else f"\nWARNING: could not restore the original gains {original} — "
            "run `arm-setup gains apply` before using the arm"
        )

    if results:
        _report_sweep(cfg, joint, results, args, start_rad, goal_rad)
    if aborted:
        raise SystemExit(f"sweep stopped: {aborted}")


def _report_sweep(cfg, joint, results, args, start_rad, goal_rad) -> None:
    print(
        f"\n{'kp':>4}{'kd':>4}{'ki':>4}{'error':>9}{'kp*err':>8}{'reached':>9}"
        f"{'load':>7}{'settle':>8}{'ripple':>8}{'rev':>5}"
    )
    for row in results:
        m = row["metrics"]
        settle = m["settling_time"]
        print(
            f"{row['kp']:>4}{row['kd']:>4}{row['ki']:>4}"
            f"{m['abs_error']:>9.3f}"
            f"{row['kp'] * m['abs_error']:>8.2f}"
            f"{100 * m['travel_fraction']:>8.1f}%"
            f"{m['hold_load']:>7.0f}"
            f"{(f'{settle:.2f}s' if settle is not None else '  --'):>8}"
            f"{m['ripple_counts']:>7.1f}c"
            f"{m['reversals']:>5}"
        )

    by_kp = [
        (row["kp"], row["metrics"])
        for row in results
        if row["ki"] == results[0]["ki"] and row["kd"] == results[0]["kd"]
    ]
    if len(by_kp) > 1:
        print("\n" + droop_verdict([(kp, StepMetrics(**m)) for kp, m in by_kp]))

    path = Path(args.out) if args.out else _default_sweep_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(
            {
                "joint": joint.name,
                "servo_id": joint.id,
                "port": cfg.port,
                "moving_speed": args.speed if args.speed else cfg.moving_speed,
                "moving_acc": args.acc if args.acc else cfg.moving_acc,
                "start_rad": start_rad,
                "goal_rad": goal_rad,
                "hold_s": args.hold,
                "rate_hz": args.rate,
                "recorded_utc": datetime.now(timezone.utc).isoformat(
                    timespec="seconds"
                ),
                "trials": results,
            },
            indent=2,
        )
    )
    print(f"\ntrace written to {path}")


def _default_sweep_path() -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return mote_home() / "arm_gain_sweeps" / f"{stamp}.json"


def add_subparser(sub) -> None:
    parser = sub.add_parser("gains", help="the servos' position-loop gains (EEPROM)")
    sub = parser.add_subparsers(dest="action", required=True)

    p_show = sub.add_parser("show", help="read gains and compare to robot.yaml")
    p_show.set_defaults(func=_cmd_show)

    p_apply = sub.add_parser("apply", help="write robot.yaml's gains to the servos")
    p_apply.set_defaults(func=_cmd_apply)

    p_sweep = sub.add_parser(
        "sweep",
        help="measure one joint's step response under candidate gains (moves the arm)",
    )
    p_sweep.add_argument(
        "--joint",
        required=True,
        help="joint to move — pick one with real travel in its soft limits",
    )
    p_sweep.add_argument(
        "--step",
        type=float,
        default=-0.2,
        help="commanded travel in rad from the joint's current position "
        "(default: %(default)s, the step the droop was first measured on)",
    )
    p_sweep.add_argument("--kp", default="16,32,64,128", help="kp values to try")
    p_sweep.add_argument("--ki", default="0", help="ki values to try")
    p_sweep.add_argument(
        "--kd", default="", help="kd values to try (default: robot.yaml)"
    )
    p_sweep.add_argument(
        "--speed",
        type=int,
        help="servo speed for the measured step, steps/s (default: robot.yaml's "
        "moving_speed). A gain that holds a slow move can still overshoot a fast "
        "one, so raising this is how the top of a kp range is tested",
    )
    p_sweep.add_argument(
        "--acc", type=int, help="servo acceleration for the step (default: robot.yaml)"
    )
    p_sweep.add_argument(
        "--hold",
        type=float,
        default=3.0,
        help="seconds to sample after commanding the step (default: %(default)s)",
    )
    p_sweep.add_argument(
        "--rate",
        type=float,
        default=50.0,
        help="sampling rate in Hz (default: %(default)s)",
    )
    p_sweep.add_argument(
        "--max-temp",
        type=int,
        default=MAX_TEMP_C,
        help="stop if the servo reaches this temperature (default: %(default)s C)",
    )
    p_sweep.add_argument("--out", help="where to write the JSON trace")
    p_sweep.set_defaults(func=_cmd_sweep)
