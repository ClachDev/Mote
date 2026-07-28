"""Range calibration: swept encoder counts -> zero offset and soft limits.

The arm's soft limits should describe where the joint can physically go, which
means measuring the mechanical stops. This module turns a stream of raw encoder
counts into a ``robot.yaml`` ``arm.joints`` block.

The flow it serves is LeRobot's, in two phases:

1. **Set the homing offsets.** The operator parks the arm with every joint near
   the middle of its travel, and each servo's position-correction register
   (EEPROM, ``SMS_STS_OFS_L/H``) is written so that pose reads 2048. The servo
   reports ``present = actual - offset``, so this re-centres the joint's whole
   travel within the 0-4095 encoder frame.
2. **Record the ranges.** Every joint is swept and recorded *together* in one
   pass, not one at a time.

Phase 1 is what makes phase 2 trustworthy. Without it, a joint whose travel
straddles the encoder's 0/4095 boundary reports a raw min and max that say
nothing about its real span, and no zero/limit pair in that frame can describe
it — ``rad_to_counts`` clamps at the encoder edge. On the real arm two of six
joints did exactly that. The wrap check below is kept as the safety net that
proves phase 1 did its job, not as a problem to hand back to the operator.

The other rule: **limits sit *inside* the stops.** A hard stop is where the
operator stopped pushing, so the emitted band is the swept range pulled *inward*
by a margin — the opposite of ``poses.envelope``, which widens outward from
poses a human has already vetted as safe.

ROS-free and SDK-free, like ``config``: every rule above is a plain function, so
it is unit-tested without an arm on the bench.
"""

from __future__ import annotations

import textwrap
from dataclasses import dataclass
from pathlib import Path

import yaml

from mote_arm.bus import OFFSET_MAX
from mote_arm.config import COUNTS_PER_REV, RAD_PER_COUNT, JointSpec
from mote_arm.poses import mote_home

# Radians of headroom kept between a soft limit and the measured hard stop.
DEFAULT_MARGIN = 0.05

# Where phase 1 puts each joint's zero: the middle of the encoder frame, so the
# travel has as much room as possible either side before it reaches a wrap.
CENTRE_COUNTS = COUNTS_PER_REV // 2

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
    """One joint's calibrated zero and soft limits, and where they came from."""

    name: str
    id: int
    invert: bool
    zero_counts: int
    min_rad: float
    max_rad: float
    # How the zero was arrived at: "centred" (phase 1 wrote a homing offset),
    # "kept" (--skip-homing reused robot.yaml), or "sweep mid-point".
    zero_source: str
    margin: float
    sweep: Sweep


def limits_from_sweep(
    sweep: Sweep,
    zero_counts: int,
    invert: bool = False,
    margin: float = DEFAULT_MARGIN,
) -> tuple[float, float]:
    """Soft limits in radians about ``zero_counts``, pulled inward by ``margin``.

    The stops themselves are where the operator stopped pushing, so they are the
    outer bound of what is known-reachable, not a target. Everything the caller
    must not silently get wrong raises instead: a wrapped sweep, a zero outside
    the swept range, and a range too narrow to survive the margin.
    """
    if margin < 0:
        raise ValueError("margin must not be negative")
    if sweep.unwrapped_span >= COUNTS_PER_REV:
        # No homing offset can rescue this: the joint simply does not fit in a
        # single-turn frame. Distinguished from an ordinary wrap because the
        # remedy is different — there isn't one, short of excluding the joint.
        raise CalibrationError(
            f"{sweep.name}: swept {sweep.span_rad:.2f} rad, more than the one "
            f"full revolution the encoder can express. A continuously-rotating "
            "joint has no mechanical stops to calibrate against — leave it out "
            "with --joints, and drive it in relative terms instead.",
            reason="travel exceeds one revolution; joint is continuous",
        )
    if sweep.wrapped:
        raise CalibrationError(
            f"{sweep.name}: the sweep crossed the encoder's 0/{COUNTS_PER_REV - 1} "
            f"boundary {sweep.wraps} time(s), so its raw range "
            f"({sweep.min_counts}-{sweep.max_counts}) does not describe its travel. "
            "Re-run without --skip-homing so phase 1 re-centres this joint, and "
            "park it nearer the middle of its travel when asked.",
            reason="sweep crossed the encoder 0/4095 boundary",
        )
    if not sweep.min_counts <= zero_counts <= sweep.max_counts:
        raise CalibrationError(
            f"{sweep.name}: home {zero_counts} lies outside the swept range "
            f"{sweep.min_counts}-{sweep.max_counts} — the zero pose must be one "
            "the joint can actually reach.",
            reason="home lies outside the swept range",
        )

    sign = -1 if invert else 1
    ends = (
        sign * (sweep.min_counts - zero_counts) * RAD_PER_COUNT,
        sign * (sweep.max_counts - zero_counts) * RAD_PER_COUNT,
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
    zero_counts: int | None = None,
    margin: float = DEFAULT_MARGIN,
    zero_source: str = "",
) -> JointCalibration:
    """Turn one sweep into a calibrated joint.

    ``zero_counts`` is where 0 rad sits — normally ``CENTRE_COUNTS``, since
    phase 1 has just written the homing offsets that put it there. Without one
    it falls back to the mid-point of the sweep. ``zero_source`` records which,
    so the emitted YAML says it rather than leaving it to be inferred.
    """
    if sweep.wrapped:
        # Reported by limits_from_sweep, but mid_counts would be nonsense first.
        limits_from_sweep(sweep, spec.zero_counts, spec.invert, margin)
    zero = sweep.mid_counts if zero_counts is None else int(zero_counts)
    source = zero_source or ("sweep mid-point" if zero_counts is None else "measured")
    lo, hi = limits_from_sweep(sweep, zero, spec.invert, margin)
    return JointCalibration(
        name=spec.name,
        id=spec.id,
        invert=spec.invert,
        zero_counts=zero,
        min_rad=lo,
        max_rad=hi,
        zero_source=source,
        margin=margin,
        sweep=sweep,
    )


def homing_offset(
    present_counts: int,
    existing_offset: int,
    centre: int = CENTRE_COUNTS,
) -> int:
    """The offset that makes the joint's current pose read ``centre``.

    The servo reports ``present = actual - offset``, so the actual encoder angle
    is ``present + existing_offset`` and the offset wanted is that minus the
    target. ``existing_offset`` must be the value currently in the register, not
    assumed zero: re-running calibration on an already-offset servo would
    otherwise double-count it and move the zero by the old offset again.
    """
    offset = present_counts + existing_offset - centre
    if abs(offset) > OFFSET_MAX:
        raise CalibrationError(
            f"the offset needed ({offset}) exceeds the servo's +-{OFFSET_MAX} "
            "correction range. The joint is more than half a revolution from "
            "centre — move it nearer the middle of its travel and retry.",
            reason="needed offset exceeds the servo's correction range",
        )
    return offset


def zero_shift(old_zero: int, new_zero: int, invert: bool = False) -> float:
    """Radians a stored pose value moves by when a joint's zero changes.

    Poses are recorded in radians about the zero, so re-zeroing silently
    redefines every one of them: the same stored number now names a different
    physical position, off by exactly this much.
    """
    sign = -1 if invert else 1
    return sign * (old_zero - new_zero) * RAD_PER_COUNT


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
    name: str, joint_id: int, lo: float, hi: float, zero: int, invert: bool
) -> str:
    return (
        f"    - {{name: {name + ',':<16} id: {joint_id}, "
        f"min: {lo:>7.3f}, max: {hi:>7.3f}, "
        f"zero: {zero:>4}, invert: {str(invert).lower()}}}"
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
        sources = ", ".join(sorted({c.zero_source for c in done}))
        lines += textwrap.wrap(
            f"Soft limits measured by sweeping each joint to its mechanical stops "
            f"({len(done)} joint(s)"
            + (f", {recorded}" if recorded else "")
            + f"). The band is the swept range pulled INWARD by {margins} rad, so a "
            "soft limit always stops short of the stop it was measured from. "
            f"zero: {sources}. NOTE zero is the middle of each joint's travel, "
            "not the arm's rest pose — the rest pose is a taught pose named "
            "'home' in arm_poses.yaml.",
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
                    cal.zero_counts,
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
                joint.zero_counts,
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
    offsets: dict[str, int] | None = None,
) -> Path:
    """Write what was measured, so the limits in robot.yaml have a provenance.

    Per-robot state (``MOTE_HOME``), like poses: the numbers describe one
    physical arm's stops on one day, not the repo's shared configuration.
    """
    p = Path(path) if path is not None else record_path()
    offsets = offsets or {}
    joints = {}
    for cal in calibrated.values():
        entry = {
            "id": cal.id,
            "min_counts": cal.sweep.min_counts,
            "max_counts": cal.sweep.max_counts,
            "samples": cal.sweep.samples,
            "span_rad": round(cal.sweep.span_rad, 4),
            "zero": cal.zero_counts,
            "zero_source": cal.zero_source,
            "margin": cal.margin,
            "min": round(cal.min_rad, 4),
            "max": round(cal.max_rad, 4),
        }
        # The offset lives in servo EEPROM, where nothing else records it; if
        # a servo is swapped this is the only note of what the old one carried.
        if cal.name in offsets:
            entry["homing_offset"] = offsets[cal.name]
        joints[cal.name] = entry
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(
        yaml.safe_dump({"recorded": recorded, "joints": joints}, sort_keys=True)
    )
    return p
