"""Range calibration: swept encoder counts -> home offset and soft limits.

The arm's soft limits should describe where the joint can physically go, which
means measuring the mechanical stops. An operator sweeps each limp joint to both
stops by hand while the tool watches the encoder; this module turns that stream
of raw counts into a ``robot.yaml`` ``arm.joints`` block.

Two things make that more than a min/max:

* **The 12-bit encoder wraps.** A joint whose travel crosses the 0/4095 boundary
  reports a raw min and max that say nothing about its real span, and no
  ``home``/limit pair expressed in that scheme can describe it — ``rad_to_counts``
  clamps at the encoder edge. Wraps are detected and refused rather than turned
  into plausible-looking numbers.
* **Limits sit *inside* the stops.** A hard stop is where the operator stopped
  pushing, so the emitted band is the swept range pulled *inward* by a margin.
  This is the opposite of ``poses.envelope``, which widens outward from poses a
  human has already vetted as safe.

ROS-free and SDK-free, like ``config``: every rule above is a plain function so
it is unit-tested without an arm on the bench.
"""

from __future__ import annotations

import textwrap
from dataclasses import dataclass
from pathlib import Path

import yaml

from mote_arm.config import COUNTS_PER_REV, RAD_PER_COUNT, JointSpec
from mote_arm.poses import mote_home

# Radians of headroom kept between a soft limit and the measured hard stop.
DEFAULT_MARGIN = 0.05

# A hand-moved joint cannot travel half a revolution between consecutive
# samples, so a jump that large is the encoder wrapping past 0/4095.
WRAP_THRESHOLD = COUNTS_PER_REV // 2


class CalibrationError(ValueError):
    """A sweep that cannot be turned into limits, with the reason why.

    Carries a short ``reason`` alongside the full message: the message explains
    the fix to an operator at the bench, while the reason is what fits on the
    joint's line in the emitted YAML.
    """

    def __init__(self, message: str, reason: str = "not calibrated"):
        super().__init__(message)
        self.reason = reason


@dataclass(frozen=True)
class Sweep:
    """One joint's recorded travel between its mechanical stops."""

    name: str
    samples: int
    min_counts: int
    max_counts: int
    # Times the raw reading jumped more than half a revolution between samples.
    wraps: int
    # Span measured on the unwrapped stream, so it stays true across a wrap.
    unwrapped_span: int

    @property
    def wrapped(self) -> bool:
        return self.wraps > 0

    @property
    def raw_span(self) -> int:
        """max - min of the raw readings; meaningless once ``wrapped``."""
        return self.max_counts - self.min_counts

    @property
    def span_rad(self) -> float:
        return self.unwrapped_span * RAD_PER_COUNT

    @property
    def mid_counts(self) -> int:
        """The raw mid-point of the sweep; meaningless once ``wrapped``."""
        return round((self.min_counts + self.max_counts) / 2)


class SweepRecorder:
    """Accumulates one joint's encoder readings as the operator sweeps it.

    Fed a sample at a time so the interactive tool can show the running range
    live. It tracks the raw min/max *and* an unwrapped position, so a sweep that
    crosses 0/4095 still reports its true span while being flagged as wrapped.

    A single garbled reading registers as two wraps that cancel in the unwrapped
    span. That is deliberate: the wrap count is reported rather than corrected,
    because a tool that silently repaired encoder readings would also silently
    hide a joint genuinely sitting on the boundary.
    """

    def __init__(self, name: str):
        self.name = name
        self._samples = 0
        self._min = COUNTS_PER_REV
        self._max = -1
        self._prev: int | None = None
        self._unwrapped = 0
        self._u_min = 0
        self._u_max = 0
        self._wraps = 0

    def add(self, counts: int) -> None:
        if not 0 <= counts < COUNTS_PER_REV:
            raise ValueError(
                f"{self.name}: encoder reading {counts} outside 0-{COUNTS_PER_REV - 1}"
            )
        if self._prev is None:
            self._unwrapped = counts
            self._u_min = self._u_max = counts
        else:
            delta = counts - self._prev
            if delta > WRAP_THRESHOLD:
                delta -= COUNTS_PER_REV
                self._wraps += 1
            elif delta < -WRAP_THRESHOLD:
                delta += COUNTS_PER_REV
                self._wraps += 1
            self._unwrapped += delta
            self._u_min = min(self._u_min, self._unwrapped)
            self._u_max = max(self._u_max, self._unwrapped)
        self._prev = counts
        self._min = min(self._min, counts)
        self._max = max(self._max, counts)
        self._samples += 1

    @property
    def samples(self) -> int:
        return self._samples

    @property
    def min_counts(self) -> int:
        return self._min

    @property
    def max_counts(self) -> int:
        return self._max

    @property
    def wraps(self) -> int:
        return self._wraps

    @property
    def unwrapped_span(self) -> int:
        """Live span across the sweep so far, true even once it has wrapped."""
        return self._u_max - self._u_min

    def result(self) -> Sweep:
        if self._samples == 0:
            raise CalibrationError(f"{self.name}: no encoder readings recorded")
        return Sweep(
            name=self.name,
            samples=self._samples,
            min_counts=self._min,
            max_counts=self._max,
            wraps=self._wraps,
            unwrapped_span=self._u_max - self._u_min,
        )


@dataclass(frozen=True)
class JointCalibration:
    """One joint's calibrated home and soft limits, and where they came from."""

    name: str
    id: int
    invert: bool
    home_counts: int
    min_rad: float
    max_rad: float
    # "mid-range" (derived from the sweep) or "captured" (operator-posed zero).
    home_source: str
    margin: float
    sweep: Sweep


def limits_from_sweep(
    sweep: Sweep,
    home_counts: int,
    invert: bool = False,
    margin: float = DEFAULT_MARGIN,
) -> tuple[float, float]:
    """Soft limits in radians about ``home_counts``, pulled inward by ``margin``.

    The stops themselves are where the operator stopped pushing, so they are the
    outer bound of what is known-reachable, not a target. Everything the caller
    must not silently get wrong raises instead: a wrapped sweep, a zero outside
    the swept range, and a range too narrow to survive the margin.
    """
    if margin < 0:
        raise ValueError("margin must not be negative")
    if sweep.wrapped:
        raise CalibrationError(
            f"{sweep.name}: the sweep crossed the encoder's 0/{COUNTS_PER_REV - 1} "
            f"boundary {sweep.wraps} time(s), so its raw range "
            f"({sweep.min_counts}-{sweep.max_counts}) does not describe its travel. "
            "Re-home the servo so the joint's mid-range sits away from the "
            "boundary (see mote_arm/BENCH.md), then sweep it again.",
            reason="sweep crossed the encoder 0/4095 boundary",
        )
    if not sweep.min_counts <= home_counts <= sweep.max_counts:
        raise CalibrationError(
            f"{sweep.name}: home {home_counts} lies outside the swept range "
            f"{sweep.min_counts}-{sweep.max_counts} — the zero pose must be one "
            "the joint can actually reach.",
            reason="home lies outside the swept range",
        )

    sign = -1 if invert else 1
    ends = (
        sign * (sweep.min_counts - home_counts) * RAD_PER_COUNT,
        sign * (sweep.max_counts - home_counts) * RAD_PER_COUNT,
    )
    lo, hi = min(ends), max(ends)
    if hi - lo <= 2 * margin:
        raise CalibrationError(
            f"{sweep.name}: swept only {hi - lo:.3f} rad, which a {margin:.3f} rad "
            "margin at each end would erase. Sweep the joint to both stops, or "
            "lower --margin if the joint really is that short.",
            reason=f"swept only {hi - lo:.3f} rad, too short for the margin",
        )

    lo, hi = lo + margin, hi - margin
    if not lo <= 0.0 <= hi:
        raise CalibrationError(
            f"{sweep.name}: zero would sit outside the limits [{lo:+.3f}, "
            f"{hi:+.3f}] — the home pose is within {margin:.3f} rad of a stop, so "
            "the joint could never be commanded to 0 rad. Re-capture home further "
            "from the stops.",
            reason="home too close to a stop; zero would be unreachable",
        )
    return lo, hi


def calibrate_joint(
    spec: JointSpec,
    sweep: Sweep,
    home_counts: int | None = None,
    margin: float = DEFAULT_MARGIN,
) -> JointCalibration:
    """Turn one sweep into a calibrated joint.

    ``home_counts`` is an operator-captured zero pose; without one, home is the
    mid-point of the sweep. Which of the two was used is carried on the result so
    the emitted YAML can say so rather than leaving it to be inferred.
    """
    if sweep.wrapped:
        # Reported by limits_from_sweep, but mid_counts would be nonsense first.
        limits_from_sweep(sweep, spec.home_counts, spec.invert, margin)
    source = "captured" if home_counts is not None else "mid-range"
    home = sweep.mid_counts if home_counts is None else int(home_counts)
    lo, hi = limits_from_sweep(sweep, home, spec.invert, margin)
    return JointCalibration(
        name=spec.name,
        id=spec.id,
        invert=spec.invert,
        home_counts=home,
        min_rad=lo,
        max_rad=hi,
        home_source=source,
        margin=margin,
        sweep=sweep,
    )


def home_shift(old_home: int, new_home: int, invert: bool = False) -> float:
    """Radians a stored pose value moves by when ``home`` changes.

    Poses are recorded in radians about home, so re-homing silently redefines
    every one of them: the same stored number now names a different physical
    position, off by exactly this much.
    """
    sign = -1 if invert else 1
    return sign * (old_home - new_home) * RAD_PER_COUNT


def pose_impact(
    taught: dict[str, dict[str, float]],
    shifts: dict[str, float],
) -> dict[str, dict[str, float]]:
    """Which taught poses a set of home changes invalidates, and by how much.

    Returns ``{pose: {joint: shift_rad}}`` for poses touching a moved joint,
    omitting joints whose home did not move. Empty when nothing is affected.
    """
    moved = {name: shift for name, shift in shifts.items() if shift != 0.0}
    impact: dict[str, dict[str, float]] = {}
    for pose, joints in taught.items():
        affected = {n: moved[n] for n in joints if n in moved}
        if affected:
            impact[pose] = affected
    return impact


def _joint_line(
    name: str, joint_id: int, lo: float, hi: float, home: int, invert: bool
) -> str:
    return (
        f"    - {{name: {name + ',':<16} id: {joint_id}, "
        f"min: {lo:>7.3f}, max: {hi:>7.3f}, "
        f"home: {home:>4}, invert: {str(invert).lower()}}}"
    )


def joints_block(
    joints: list[JointSpec],
    calibrated: dict[str, JointCalibration],
    failures: dict[str, str] | None = None,
    recorded: str | None = None,
) -> str:
    """Render a ready-to-paste ``arm.joints`` block, in servo-command order.

    Joints that were not calibrated (skipped, or a sweep that could not be used)
    keep their existing values and say why, so pasting the block never silently
    reverts a joint to a guess.
    """
    failures = failures or {}
    lines: list[str] = []
    done = list(calibrated.values())
    if done:
        margins = "/".join(f"{m:.3f}" for m in sorted({c.margin for c in done}))
        sources = ", ".join(sorted({c.home_source for c in done}))
        lines += textwrap.wrap(
            f"Soft limits measured by sweeping each joint to its mechanical stops "
            f"({len(done)} joint(s)"
            + (f", {recorded}" if recorded else "")
            + f"). The band is the swept range pulled INWARD by {margins} rad, so a "
            "soft limit always stops short of the stop it was measured from. "
            f"home: {sources}.",
            width=78,
            initial_indent="  # ",
            subsequent_indent="  # ",
            break_on_hyphens=False,
        )
    lines.append("  joints:")
    for joint in joints:
        cal = calibrated.get(joint.name)
        if cal is not None:
            lines.append(
                _joint_line(
                    cal.name,
                    cal.id,
                    cal.min_rad,
                    cal.max_rad,
                    cal.home_counts,
                    cal.invert,
                )
                + f"  # swept {cal.sweep.min_counts}-{cal.sweep.max_counts}"
            )
            continue
        note = failures.get(joint.name, "not calibrated")
        lines.append(f"    # unchanged, {note}:")
        lines.append(
            _joint_line(
                joint.name,
                joint.id,
                joint.min_rad,
                joint.max_rad,
                joint.home_counts,
                joint.invert,
            )
        )
    return "\n".join(lines)


def record_path() -> Path:
    return mote_home() / "arm_calibration.yaml"


def save_record(
    calibrated: dict[str, JointCalibration],
    recorded: str,
    path: Path | str | None = None,
) -> Path:
    """Write what was measured, so the limits in robot.yaml have a provenance.

    Per-robot state (``MOTE_HOME``), like poses: the numbers describe one
    physical arm's stops on one day, not the repo's shared configuration.
    """
    p = Path(path) if path is not None else record_path()
    joints = {
        cal.name: {
            "id": cal.id,
            "min_counts": cal.sweep.min_counts,
            "max_counts": cal.sweep.max_counts,
            "samples": cal.sweep.samples,
            "span_rad": round(cal.sweep.span_rad, 4),
            "home": cal.home_counts,
            "home_source": cal.home_source,
            "margin": cal.margin,
            "min": round(cal.min_rad, 4),
            "max": round(cal.max_rad, 4),
        }
        for cal in calibrated.values()
    }
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(
        yaml.safe_dump({"recorded": recorded, "joints": joints}, sort_keys=True)
    )
    return p
