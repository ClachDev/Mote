"""Step-response metrics for a single servo joint.

A position-loop gain is only meaningful against a measurement, so the gain sweep
commands a step and scores what the joint actually did. The scoring lives here,
apart from the bus I/O, so the maths that decides "this gain is better" is
unit-tested against synthetic traces rather than judged by eye on hardware.

Two numbers carry the droop argument. ``steady_error`` is how far short the
joint settles, and ``hold_load`` is the effort it is spending to sit there:
proportional droop holds ``kp * error`` roughly constant while load stays well
below the 1000 limit, whereas torque saturation pins load near 1000 and error
stops responding to gain. ``ripple`` and ``reversals`` are the counter-check on
raising gain — a servo that hunts around its goal is buzzing, not holding.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass

# STS3215 encoders report 4096 counts per turn; one count is the finest real
# motion, so position changes below it are quantisation, not movement.
RAD_PER_COUNT = 2.0 * math.pi / 4096


@dataclass(frozen=True)
class Sample:
    """One reading during a step: seconds since the goal was commanded."""

    t: float
    rad: float
    load: int


@dataclass(frozen=True)
class StepMetrics:
    """What one commanded step did, in the units the gain decision is made in."""

    start_rad: float
    goal_rad: float
    final_rad: float
    # Signed goal - final: positive means it stopped short in the + direction.
    steady_error: float
    # |steady_error|, the figure droop is compared on.
    abs_error: float
    # Fraction of the commanded travel actually covered (1.0 = arrived).
    travel_fraction: float
    # Fraction of the commanded travel overshot past the goal; 0.0 if it never
    # passed the goal.
    overshoot: float
    # Seconds until the joint stays inside the settle band for the rest of the
    # hold, or None if it never does.
    settling_time: float | None
    # Peak-to-peak motion while holding, in radians and encoder counts: a servo
    # that cannot sit still reports ripple over a count or two.
    ripple: float
    ripple_counts: float
    # Direction changes larger than one count while holding — hunting/buzz.
    reversals: int
    # Effort while holding: mean and peak |load|, where 1000 is the servo's max.
    hold_load: float
    peak_load: int
    samples: int

    def as_dict(self) -> dict:
        return asdict(self)


def settle_band(commanded: float, floor: float = 2 * RAD_PER_COUNT) -> float:
    """Tolerance a joint must stay inside to count as settled.

    2% of the commanded travel, but never tighter than two encoder counts —
    below that the band would be quantisation noise and nothing would ever
    settle.
    """
    return max(floor, 0.02 * abs(commanded))


def summarise(
    samples: list[Sample],
    start_rad: float,
    goal_rad: float,
    hold_window: float = 0.5,
) -> StepMetrics:
    """Score a step from its trace.

    ``hold_window`` is the trailing slice of the trace treated as "holding" —
    steady error, ripple and load are all read from it, so it must be long
    enough to contain the settled state and short enough to exclude the move.
    """
    if not samples:
        raise ValueError("no samples: the step recorded nothing to score")

    commanded = goal_rad - start_rad
    if commanded == 0.0:
        raise ValueError("step of zero: nothing to measure")

    end_t = samples[-1].t
    hold = [s for s in samples if s.t >= end_t - hold_window] or samples[-1:]

    final_rad = sum(s.rad for s in hold) / len(hold)
    steady_error = goal_rad - final_rad
    travel_fraction = (final_rad - start_rad) / commanded

    direction = 1.0 if commanded > 0 else -1.0
    peak_rad = max(samples, key=lambda s: direction * s.rad).rad
    past_goal = direction * (peak_rad - goal_rad)
    overshoot = past_goal / abs(commanded) if past_goal > 0 else 0.0

    band = settle_band(commanded)
    settling_time: float | None = None
    for i, s in enumerate(samples):
        if all(abs(later.rad - final_rad) <= band for later in samples[i:]):
            settling_time = s.t
            break

    positions = [s.rad for s in hold]
    ripple = max(positions) - min(positions)

    # Count direction changes over motion bigger than one count, so encoder
    # quantisation on a perfectly still joint does not read as hunting.
    reversals = 0
    last_dir = 0
    for previous, current in zip(hold, hold[1:]):
        delta = current.rad - previous.rad
        if abs(delta) <= RAD_PER_COUNT:
            continue
        step_dir = 1 if delta > 0 else -1
        if last_dir and step_dir != last_dir:
            reversals += 1
        last_dir = step_dir

    loads = [abs(s.load) for s in hold]
    return StepMetrics(
        start_rad=start_rad,
        goal_rad=goal_rad,
        final_rad=final_rad,
        steady_error=steady_error,
        abs_error=abs(steady_error),
        travel_fraction=travel_fraction,
        overshoot=overshoot,
        settling_time=settling_time,
        ripple=ripple,
        ripple_counts=ripple / RAD_PER_COUNT,
        reversals=reversals,
        hold_load=sum(loads) / len(loads),
        peak_load=max(abs(s.load) for s in samples),
        samples=len(samples),
    )


def droop_verdict(trials: list[tuple[int, StepMetrics]]) -> str:
    """Read a kp sweep as droop or as saturation, in one sentence.

    Under proportional droop the servo settles where ``kp * error`` balances the
    holding torque, so that product stays roughly constant as kp rises while
    load barely moves. Under torque saturation the servo is already giving
    everything it has: load sits near the 1000 limit and error stops falling.
    """
    scored = sorted((kp, m) for kp, m in trials if kp > 0)
    if len(scored) < 2:
        return "not enough gains swept to tell droop from saturation"

    products = [kp * m.abs_error for kp, m in scored]
    loads = [m.hold_load for _, m in scored]
    lowest, highest = scored[0], scored[-1]

    if max(loads) >= 900:
        return (
            f"load reaches {max(loads):.0f}/1000 — at or near torque "
            "saturation, so the servo is out of effort, not out of gain"
        )

    spread = max(products) / min(products) if min(products) > 0 else math.inf
    error_ratio = (
        lowest[1].abs_error / highest[1].abs_error
        if highest[1].abs_error > 0
        else math.inf
    )
    gain_ratio = highest[0] / lowest[0]

    if spread <= 1.5 and error_ratio > 1.2:
        return (
            f"proportional droop: kp*error held within {spread:.2f}x across "
            f"kp {lowest[0]}-{highest[0]} while load stayed at "
            f"{min(loads):.0f}-{max(loads):.0f}/1000, and error fell "
            f"{error_ratio:.1f}x for a {gain_ratio:.0f}x gain rise"
        )
    if error_ratio <= 1.2:
        return (
            f"error barely moved ({error_ratio:.2f}x) across kp "
            f"{lowest[0]}-{highest[0]} at load "
            f"{min(loads):.0f}-{max(loads):.0f}/1000 — something other than "
            "proportional gain is the limit (friction, backlash, or a goal "
            "already reached)"
        )
    return (
        f"error fell {error_ratio:.1f}x over a {gain_ratio:.0f}x gain rise but "
        f"kp*error varied {spread:.2f}x — not clean proportional droop; read "
        "the per-trial rows"
    )
