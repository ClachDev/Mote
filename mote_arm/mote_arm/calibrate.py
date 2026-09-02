"""Range calibration: swept encoder counts -> zero offset and soft limits.

The arm's soft limits should describe where the joint can physically go, which
means measuring the mechanical stops. This module turns a stream of raw encoder
counts into a ``robot.yaml`` ``arm.joints`` block.

The flow it serves has two phases:

1. **Record the ranges.** Every joint is swept and recorded *together* in one
   pass, not one at a time.
2. **Centre the zeros.** Each joint's zero moves to the measured middle of its
   sweep, by writing the servo's position-correction register (EEPROM,
   ``SMS_STS_OFS_L/H``). The servo reports ``present = actual - offset``, so
   this re-centres the joint's whole travel within the 0-4095 encoder frame.

Centring is what makes the limits expressible at all: a joint whose travel
straddles the 0/4095 boundary has a raw min and max that say nothing about its
real span, and no zero/limit pair in that frame can describe it —
``rad_to_counts`` clamps at the encoder edge. On the real arm two of six joints
did exactly that.

LeRobot runs these the other way round, taking the zero from a pose the operator
holds at mid-travel before sweeping. That ordering is load-bearing for them:
their range recording is a plain min/max with no unwrapping, so centring first
is what keeps the sweep off the boundary. Sweeping first removes an awkward step
but means the sweep can cross it, which is why ``SweepRecorder`` unwraps and the
centre is recovered from the unwrapped stream.

The other rule: **limits sit *inside* the stops.** A hard stop is where the
operator stopped pushing, so the emitted band is the swept range pulled *inward*
by a margin — the opposite of ``poses.envelope``, which widens outward from
poses a human has already vetted as safe.

ROS-free and SDK-free, like ``config``: every rule above is a plain function, so
it is unit-tested without an arm on the bench.
"""

from __future__ import annotations

import os
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
    # The travel on the unwrapped stream, anchored so that the first sample's
    # unwrapped value equals its raw value. These stay true across a wrap, which
    # is what lets the measured mid-travel be recovered from a wrapped sweep.
    unwrapped_min: int
    unwrapped_max: int

    @property
    def wrapped(self) -> bool:
        return self.wraps > 0

    @property
    def unwrapped_span(self) -> int:
        return self.unwrapped_max - self.unwrapped_min

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

    @property
    def travel_ends(self) -> tuple[int, int]:
        """The encoder counts at the two ends of the travel.

        Taken from the unwrapped stream and mapped back, so they are the real
        endpoints even for a joint whose travel crosses 0/4095 — where the raw
        min and max are 17 and 4093 and describe the encoder rather than the
        joint. For a sweep that never wrapped these are exactly the raw min and
        max; for one that did, the first number is larger than the second,
        which is precisely what "runs up through zero and out the other side"
        looks like.
        """
        return (
            self.unwrapped_min % COUNTS_PER_REV,
            self.unwrapped_max % COUNTS_PER_REV,
        )

    @property
    def measured_centre(self) -> int:
        """The raw encoder count at the middle of the *measured* travel.

        Recovered through the unwrapping, so it is correct even for a sweep that
        crossed 0/4095 — which is the whole reason the centre is taken from the
        sweep rather than from a pose the operator has to hold.
        """
        mid = round((self.unwrapped_min + self.unwrapped_max) / 2)
        return mid % COUNTS_PER_REV


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

    @property
    def travel_ends(self) -> tuple[int, int]:
        """Live equivalent of ``Sweep.travel_ends``, for the running display."""
        return (self._u_min % COUNTS_PER_REV, self._u_max % COUNTS_PER_REV)

    def result(self) -> Sweep:
        if self._samples == 0:
            raise CalibrationError(f"{self.name}: no encoder readings recorded")
        return Sweep(
            name=self.name,
            samples=self._samples,
            min_counts=self._min,
            max_counts=self._max,
            wraps=self._wraps,
            unwrapped_min=self._u_min,
            unwrapped_max=self._u_max,
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
    _reject_continuous(sweep)
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


def normalise_offset(offset: int) -> int:
    """Fold an offset into the range the servo's register can hold.

    The servo computes ``present = (actual - offset) mod 4096``, so offsets are
    modular: 3056 and -1040 command exactly the same thing. Folding is therefore
    always available, and an offset is never "out of range" — treating the raw
    arithmetic result as a hard failure was a bug that rejected perfectly good
    calibrations.

    The register holds a sign and an 11-bit magnitude, i.e. -2047..2047: 4095 of
    the 4096 residues. The one that cannot be named exactly is +-2048, which is
    clamped, costing one encoder count (0.0015 rad).
    """
    folded = (offset + CENTRE_COUNTS) % COUNTS_PER_REV - CENTRE_COUNTS
    return max(-OFFSET_MAX, min(OFFSET_MAX, folded))


def homing_offset(
    target_counts: int,
    existing_offset: int,
    centre: int = CENTRE_COUNTS,
) -> int:
    """The offset that makes ``target_counts`` read ``centre``.

    The servo reports ``present = actual - offset``, so the actual encoder angle
    at the target is ``target_counts + existing_offset`` and the offset wanted is
    that minus where we want it to read. ``existing_offset`` must be the value
    currently in the register, not assumed zero: re-running calibration on an
    already-offset servo would otherwise double-count it.
    """
    return normalise_offset(target_counts + existing_offset - centre)


def centred_limits(
    sweep: Sweep,
    invert: bool = False,
    margin: float = DEFAULT_MARGIN,
) -> tuple[float, float]:
    """Soft limits about the middle of the measured travel.

    Symmetric by construction — the zero *is* the mid-point of what was swept —
    so unlike ``limits_from_sweep`` this can never produce a band that excludes
    its own zero. Wrapping is irrelevant here because the span comes from the
    unwrapped stream.
    """
    if margin < 0:
        raise ValueError("margin must not be negative")
    _reject_continuous(sweep)
    half = sweep.unwrapped_span / 2.0 * RAD_PER_COUNT
    if 2 * half <= 2 * margin:
        raise CalibrationError(
            f"{sweep.name}: swept only {2 * half:.3f} rad, which a {margin:.3f} "
            "rad margin at each end would erase. Sweep the joint to both stops, "
            "or lower --margin if the joint really is that short.",
            reason=f"swept only {2 * half:.3f} rad, too short for the margin",
        )
    lo, hi = -half + margin, half - margin
    return (lo, hi) if not invert else (-hi, -lo)


def _reject_continuous(sweep: Sweep) -> None:
    if sweep.unwrapped_span >= COUNTS_PER_REV:
        # No homing offset can rescue this: the joint simply does not fit in a
        # single-turn frame. Distinguished from an ordinary wrap because the
        # remedy is different — there isn't one, short of excluding the joint.
        raise CalibrationError(
            f"{sweep.name}: swept {sweep.span_rad:.2f} rad, more than the one "
            f"full revolution the encoder can express. A continuously-rotating "
            "joint has no mechanical stops to calibrate against — leave it out "
            "with --joints, and drive it in relative terms instead. (Note this "
            "catches only a joint rotated past a whole turn: one that spins "
            "freely but was rotated less looks exactly like a joint with stops, "
            "and nothing in a sweep can tell them apart.)",
            reason="travel exceeds one revolution; joint is continuous",
        )


def calibrate_centred(
    spec: JointSpec,
    sweep: Sweep,
    margin: float = DEFAULT_MARGIN,
) -> JointCalibration:
    """Calibrate a joint whose zero will be moved to its measured mid-travel."""
    lo, hi = centred_limits(sweep, spec.invert, margin)
    return JointCalibration(
        name=spec.name,
        id=spec.id,
        invert=spec.invert,
        zero_counts=CENTRE_COUNTS,
        min_rad=lo,
        max_rad=hi,
        zero_source="the middle of the measured travel",
        margin=margin,
        sweep=sweep,
    )


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


def calibration_document(
    joints: list[JointSpec],
    calibrated: dict[str, JointCalibration],
    recorded: str,
    offsets: dict[str, int] | None = None,
) -> dict:
    """The per-robot calibration file's contents.

    Measurements *and* the values derived from them in one document: the zero
    and limits the arm runs on, plus the sweep they came from and the homing
    offset written to the servo — which exists nowhere else, so if a servo is
    swapped this is the only record of what the old one carried.

    Only calibrated joints appear. A joint that was skipped or refused is simply
    absent, and keeps the packaged default, rather than being written out with
    stale numbers that would look measured.
    """
    offsets = offsets or {}
    out: dict[str, dict] = {}
    for spec in joints:
        cal = calibrated.get(spec.name)
        if cal is None:
            continue
        entry = {
            "id": cal.id,
            "zero": cal.zero_counts,
            "min": round(cal.min_rad, 4),
            "max": round(cal.max_rad, 4),
            "swept_rad": round(cal.sweep.span_rad, 4),
            # The ends of travel, not the raw min/max: for a joint that crossed
            # 0/4095 those would read 17 and 4093 and describe the encoder.
            "swept_counts": list(cal.sweep.travel_ends),
            "samples": cal.sweep.samples,
            "margin": cal.margin,
            "zero_source": cal.zero_source,
        }
        if spec.name in offsets:
            entry["homing_offset"] = offsets[spec.name]
        out[spec.name] = entry
    return {"recorded": recorded, "joints": out}


def calibration_header(recorded: str) -> str:
    """The comment written above the calibration, for whoever opens the file."""
    return (
        "# This robot's measured arm calibration — written by "
        "`pixi run arm-calibrate`.\n"
        "#\n"
        "# Per-robot state, deliberately not in the repo: these are measurements "
        "of one\n"
        "# physical arm. mote_arm.config overlays `zero`/`min`/`max` onto the "
        "packaged\n"
        "# defaults in mote_description/config/robot.yaml; everything else "
        "(ids, gains,\n"
        "# direction) stays with the package. Delete this file to fall back to "
        "those\n"
        "# defaults.\n"
        "#\n"
        "# `zero` is the encoder count reading 0 rad — the middle of each joint's\n"
        "# travel, NOT the arm's rest pose (that is a taught pose in "
        "arm_poses.yaml).\n"
        "# `homing_offset` is what was written to the servo's EEPROM; it lives\n"
        f"# nowhere else. {recorded}.\n"
    )


def save_calibration(document: dict, path: Path | str | None = None) -> Path:
    """Write the calibration atomically, so an interrupt cannot truncate it."""
    from mote_arm.config import calibration_path

    p = Path(path) if path is not None else calibration_path()
    p.parent.mkdir(parents=True, exist_ok=True)
    text = calibration_header(str(document.get("recorded", ""))) + yaml.safe_dump(
        document, sort_keys=True
    )
    tmp = p.with_suffix(p.suffix + ".tmp")
    tmp.write_text(text)
    os.replace(tmp, p)
    return p


def offsets_backup_path() -> Path:
    return mote_home() / "arm_offsets_backup.yaml"


def save_offsets_backup(
    offsets: dict[str, int],
    ids: dict[str, int],
    when: str,
    path: Path | str | None = None,
) -> Path:
    """Record the offsets currently in EEPROM, *before* any are overwritten.

    The offset register is the one piece of arm state with no other copy: it
    lives only in the servo, and once overwritten the previous value is gone. A
    calibration run that dies partway would otherwise leave an arm nobody can
    put back. Written before the first write, so it always describes the state
    to return to.
    """
    p = Path(path) if path is not None else offsets_backup_path()
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(
        yaml.safe_dump(
            {
                "saved": when,
                "offsets": {
                    name: {"id": ids[name], "offset": value}
                    for name, value in sorted(offsets.items())
                },
            },
            sort_keys=True,
        )
    )
    return p


def load_offsets_backup(path: Path | str | None = None) -> dict[str, int]:
    """Return {joint: offset} from the backup, or empty if there is none."""
    p = Path(path) if path is not None else offsets_backup_path()
    if not p.exists():
        return {}
    data = yaml.safe_load(p.read_text()) or {}
    return {
        str(name): int(entry["offset"])
        for name, entry in (data.get("offsets") or {}).items()
    }


def limits_backup_path() -> Path:
    return mote_home() / "arm_limits_backup.yaml"


def save_limits_backup(
    limits: dict[str, tuple[int, int]],
    ids: dict[str, int],
    when: str,
    path: Path | str | None = None,
) -> Path:
    """Record the goal-range registers as found, *before* any are overwritten.

    Same rule as ``save_offsets_backup``, for the same reason: registers 9 and
    11 live only in the servo. This arm arrived with five of six joints fenced
    to a band narrower than their travel, which nothing here had ever read, so
    the value being overwritten may be the only record of how a servo shipped.
    """
    p = Path(path) if path is not None else limits_backup_path()
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(
        yaml.safe_dump(
            {
                "saved": when,
                "limits": {
                    name: {"id": ids[name], "min": int(low), "max": int(high)}
                    for name, (low, high) in sorted(limits.items())
                },
            },
            sort_keys=True,
        )
    )
    return p


def load_limits_backup(path: Path | str | None = None) -> dict[str, tuple[int, int]]:
    """Return {joint: (min, max)} from the backup, or empty if there is none."""
    p = Path(path) if path is not None else limits_backup_path()
    if not p.exists():
        return {}
    data = yaml.safe_load(p.read_text()) or {}
    return {
        str(name): (int(entry["min"]), int(entry["max"]))
        for name, entry in (data.get("limits") or {}).items()
    }
