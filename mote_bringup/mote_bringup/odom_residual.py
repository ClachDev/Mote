"""Wheel-vs-lidar odometry residual: the maths, with no ROS in it.

Mote carries two independent motion sources — wheel odometry, and kinematic_icp's
scan-matched pose, which *takes* the wheel odom as its prior and corrects it. The
correction is therefore already a measurement of how wrong the wheels were, and
needs no extra sensor to read.

This module holds the estimator both consumers run: the live ``slip_monitor``
node, and ``tools/slip_replay.py``, which scores it over recorded bags. Sharing
it is the point — a threshold calibrated offline is only meaningful if the robot
computes the same number.

Two things are deliberate:

**The residual is a windowed displacement difference, not a per-interval one.**
Each source's relative motion is taken over a window of ``window`` seconds and
divided by it. Per-interval differencing at the 10 Hz ICP rate is dominated by
scan-match jitter — measured p50 |yaw| residual ~3 deg/s on quiet bags, which
falls to ~1 deg/s at a 1 s window and ~0.5 at 2 s. Any zero-mean per-sample
disagreement averages down as the window grows; a real slip does not, because it
accumulates displacement.

**Both streams are interpolated to the window's own endpoints**, rather than one
being resampled onto the other's stamps. The two arrive at different rates (wheel
~50-100 Hz, ICP ~10 Hz), so comparing them at either one's sample times leaves a
sampling-grid error that a fast turn converts into apparent disagreement.

Only the *translation* residual is thresholded. The yaw residual is computed and
published for logging, but on real bags it is an order of magnitude noisier
relative to its own signal (p99 up to 1.1x the yaw rate itself, against 0.13x for
translation on the same bags), so no yaw threshold could be set that a hard turn
would not trip. See ``docs/tuning/2026-07-28-slip-detection.md``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

# Verdicts, in the order they are reported. OK and UNKNOWN are quiet states.
OK = "ok"
UNKNOWN = "unknown"
SLIP = "slip"
STUCK = "stuck"
ICP_FAULT = "icp_fault"


def yaw_of_quat(x, y, z, w):
    """Yaw of a quaternion, the only component a planar robot moves in."""
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def rel_motion(x0, y0, a0, x1, y1, a1):
    """Motion from pose0 to pose1, expressed in pose0's body frame."""
    dx, dy = x1 - x0, y1 - y0
    c, s = math.cos(-a0), math.sin(-a0)
    return (
        c * dx - s * dy,
        s * dx + c * dy,
        math.atan2(math.sin(a1 - a0), math.cos(a1 - a0)),
    )


class PoseTrack:
    """A time-ordered buffer of planar poses, interpolable at any time inside it.

    Yaw is accumulated unwrapped as samples arrive, so interpolation across the
    +/-pi branch cut is an ordinary average rather than a half-turn error.
    """

    def __init__(self, horizon=5.0):
        self.horizon = horizon
        self._samples = []  # (t, x, y, unwrapped yaw)

    def __len__(self):
        return len(self._samples)

    def add(self, t, x, y, yaw):
        """Append a sample. Out-of-order and duplicate stamps are dropped."""
        if self._samples:
            prev_t, _, _, prev_yaw = self._samples[-1]
            if t <= prev_t:
                return
            yaw = prev_yaw + math.atan2(
                math.sin(yaw - prev_yaw), math.cos(yaw - prev_yaw)
            )
        self._samples.append((t, x, y, yaw))
        self._trim(t - self.horizon)

    def _trim(self, before):
        keep = 0
        while keep + 1 < len(self._samples) and self._samples[keep + 1][0] < before:
            keep += 1
        if keep:
            del self._samples[:keep]

    def reset(self):
        self._samples.clear()

    @property
    def span(self):
        """(oldest, newest) sample time, or None when empty."""
        if not self._samples:
            return None
        return self._samples[0][0], self._samples[-1][0]

    def at(self, t):
        """Pose interpolated at time ``t``, or None if ``t`` is outside the buffer.

        Extrapolation is refused rather than clamped: a clamped endpoint silently
        reports zero motion for the part of the window that has no data, which
        reads exactly like the robot having stopped.
        """
        samples = self._samples
        if len(samples) < 2 or t < samples[0][0] or t > samples[-1][0]:
            return None
        lo, hi = 0, len(samples) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if samples[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        t0, x0, y0, a0 = samples[lo]
        t1, x1, y1, a1 = samples[hi]
        if t1 <= t0:
            return x0, y0, a0
        f = (t - t0) / (t1 - t0)
        return x0 + f * (x1 - x0), y0 + f * (y1 - y0), a0 + f * (a1 - a0)


@dataclass(frozen=True)
class Residual:
    """One window's comparison of the two motion sources."""

    t_start: float
    t_end: float
    wheel_dist: float  # metres travelled, wheel odometry
    icp_dist: float  # metres travelled, scan-matched pose
    wheel_yaw: float  # radians turned, wheel odometry
    icp_yaw: float  # radians turned, scan-matched pose

    @property
    def dt(self):
        return self.t_end - self.t_start

    @property
    def wheel_speed(self):
        return self.wheel_dist / self.dt

    @property
    def icp_speed(self):
        return self.icp_dist / self.dt

    @property
    def wheel_yaw_rate(self):
        return self.wheel_yaw / self.dt

    @property
    def icp_yaw_rate(self):
        return self.icp_yaw / self.dt

    @property
    def speed_residual(self):
        """Positive when the wheels claim more travel than the lidar saw."""
        return (self.wheel_dist - self.icp_dist) / self.dt

    @property
    def yaw_rate_residual(self):
        """Reported for logging only — never thresholded. See the module docstring."""
        return (self.wheel_yaw - self.icp_yaw) / self.dt

    @property
    def scale(self):
        """The larger of the two reported speeds: what the residual is relative to."""
        return max(self.wheel_dist, self.icp_dist) / self.dt

    @property
    def relative(self):
        """Signed residual as a fraction of the motion actually reported."""
        return self.speed_residual / self.scale if self.scale > 0 else 0.0


@dataclass(frozen=True)
class Thresholds:
    """Detection thresholds. Defaults are calibrated in the tuning note."""

    window: float = 1.0
    # Below this reported speed neither source resolves enough travel to compare.
    min_speed: float = 0.03
    # A verdict needs the residual to clear an absolute floor *and* a fraction of
    # the motion reported. The floor rejects noise at low speed; the fraction
    # rejects a large residual that is merely a large motion measured slightly
    # differently.
    slip_speed: float = 0.030
    slip_fraction: float = 0.25
    icp_speed: float = 0.030
    icp_fraction: float = 0.25
    # A body speed the drive cannot produce is a scan-match excursion: wheel slip
    # makes the wheels over-read, never the lidar.
    max_body_speed: float = 0.218 * 1.15
    # Stuck: commanded, but neither source reports motion.
    stuck_command_speed: float = 0.05
    stuck_command_yaw_rate: float = 0.20
    stuck_speed: float = 0.01
    stuck_yaw_rate: float = 0.05
    # A verdict must hold continuously for this long before it is reported, and
    # must be absent this long before it is withdrawn. Windows overlap, so a
    # count of evaluations would depend on the evaluation rate; a duration does
    # not.
    hold: float = 0.5
    release: float = 2.0
    # A source older than this cannot contribute to a window ending now. Without
    # this, a stalled lidar freezes the window at its last value while the wheels
    # keep turning, which reads as an ever-growing slip — the one failure mode
    # that would make this node blame the wheels for a sensor dropout.
    max_lag: float = 1.0

    def with_max_wheel_speed(self, max_wheel_speed, tolerance=1.15):
        return replace(self, max_body_speed=max_wheel_speed * tolerance)


@dataclass(frozen=True)
class Verdict:
    state: str
    detail: str


def classify(residual, thresholds, command=None, reason="no odometry yet"):
    """Raw per-window verdict, before any hold/release filtering.

    ``command`` is the commanded (linear, angular) body velocity, or None when
    nothing is being published — in which case stuck cannot be distinguished from
    deliberately parked, and is not reported. ``reason`` explains a missing
    residual, and comes from :attr:`ResidualEstimator.reason`.
    """
    t = thresholds
    if residual is None:
        return Verdict(UNKNOWN, reason)

    icp_speed = residual.icp_speed
    if icp_speed > t.max_body_speed:
        return Verdict(
            ICP_FAULT,
            f"lidar odometry reports {icp_speed:.3f} m/s, "
            f"above the {t.max_body_speed:.3f} m/s the drive can produce",
        )
    # Slip makes the *wheels* over-read, never the lidar, so the two directions of
    # disagreement are different faults and are reported as such.

    if command is not None:
        cmd_v, cmd_w = command
        commanded = (
            abs(cmd_v) > t.stuck_command_speed or abs(cmd_w) > t.stuck_command_yaw_rate
        )
        still = (
            residual.wheel_speed < t.stuck_speed
            and icp_speed < t.stuck_speed
            and abs(residual.wheel_yaw_rate) < t.stuck_yaw_rate
            and abs(residual.icp_yaw_rate) < t.stuck_yaw_rate
        )
        if commanded and still:
            return Verdict(
                STUCK,
                f"commanded {cmd_v:+.2f} m/s {cmd_w:+.2f} rad/s, "
                f"neither source reports motion",
            )

    if residual.scale < t.min_speed:
        return Verdict(UNKNOWN, "stationary or too slow to compare")

    resid = residual.speed_residual
    if resid > t.slip_speed and residual.relative > t.slip_fraction:
        return Verdict(
            SLIP,
            f"wheels report {residual.wheel_speed:.3f} m/s, lidar "
            f"{icp_speed:.3f} m/s ({100 * residual.relative:.0f}% over)",
        )
    if -resid > t.icp_speed and -residual.relative > t.icp_fraction:
        return Verdict(
            ICP_FAULT,
            f"lidar odometry reports {icp_speed:.3f} m/s, wheels "
            f"{residual.wheel_speed:.3f} m/s ({-100 * residual.relative:.0f}% over)",
        )
    return Verdict(OK, "wheel and lidar odometry agree")


class ResidualEstimator:
    """Sliding-window comparison of the two odometry sources.

    Feed both streams with :meth:`add_wheel` / :meth:`add_icp` and call
    :meth:`evaluate` as often as convenient; the window always ends at the newest
    time *both* sources cover, so a stalled stream produces no verdict rather
    than a fabricated one.
    """

    def __init__(self, thresholds=None):
        self.thresholds = thresholds or Thresholds()
        horizon = max(2.0 * self.thresholds.window, 2.0)
        self.wheel = PoseTrack(horizon)
        self.icp = PoseTrack(horizon)
        self.reason = "no odometry yet"

    def add_wheel(self, t, x, y, yaw):
        self.wheel.add(t, x, y, yaw)

    def add_icp(self, t, x, y, yaw):
        self.icp.add(t, x, y, yaw)

    def reset(self):
        self.wheel.reset()
        self.icp.reset()

    def residual(self, now=None):
        """Residual over the most recent full window, or None if unavailable.

        ``now`` is the current time; passing it enables the staleness guard,
        without which a stalled source freezes the window rather than reporting
        that it has stopped. :attr:`reason` explains any None.
        """
        wheel_span, icp_span = self.wheel.span, self.icp.span
        if wheel_span is None or icp_span is None:
            missing = "wheel" if wheel_span is None else "lidar"
            self.reason = f"no {missing} odometry yet"
            return None
        if now is not None:
            for label, span in (("wheel", wheel_span), ("lidar", icp_span)):
                lag = now - span[1]
                if lag > self.thresholds.max_lag:
                    self.reason = f"{label} odometry stale ({lag:.1f}s)"
                    return None
        t_end = min(wheel_span[1], icp_span[1])
        t_start = t_end - self.thresholds.window
        if t_start < max(wheel_span[0], icp_span[0]):
            self.reason = "not yet a full window of both sources"
            return None
        w0, w1 = self.wheel.at(t_start), self.wheel.at(t_end)
        i0, i1 = self.icp.at(t_start), self.icp.at(t_end)
        if None in (w0, w1, i0, i1):
            self.reason = "window not covered by both sources"
            return None
        self.reason = ""
        wdx, wdy, wda = rel_motion(*w0, *w1)
        idx, idy, ida = rel_motion(*i0, *i1)
        return Residual(
            t_start=t_start,
            t_end=t_end,
            wheel_dist=math.hypot(wdx, wdy),
            icp_dist=math.hypot(idx, idy),
            wheel_yaw=wda,
            icp_yaw=ida,
        )


class VerdictFilter:
    """Hold/release filter turning per-window verdicts into reported events.

    A verdict is reported only once it has held continuously for ``hold``
    seconds, and is withdrawn only after ``release`` seconds without it. Windows
    overlap heavily, so a single bad window is not an event, and a flapping one
    is a single event rather than several.
    """

    def __init__(self, thresholds):
        self.thresholds = thresholds
        self.state = OK
        self.detail = "no data yet"
        self._candidate = None
        self._since = None
        self._clear_since = None

    def update(self, now, verdict):
        """Feed one window's verdict; returns the currently reported verdict."""
        t = self.thresholds
        raised = verdict.state not in (OK, UNKNOWN)

        if raised:
            self._clear_since = None
            if verdict.state != self._candidate:
                self._candidate = verdict.state
                self._since = now
            if now - self._since >= t.hold or self.state == verdict.state:
                self.state = verdict.state
                self.detail = verdict.detail
        else:
            self._candidate = None
            self._since = None
            if self.state not in (OK, UNKNOWN):
                if self._clear_since is None:
                    self._clear_since = now
                elif now - self._clear_since >= t.release:
                    self.state = OK
                    self.detail = verdict.detail
                    self._clear_since = None
            else:
                self.state = verdict.state
                self.detail = verdict.detail
        return Verdict(self.state, self.detail)
