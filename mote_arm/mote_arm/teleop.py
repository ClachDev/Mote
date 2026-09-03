"""The follow rule, with no ROS and no hardware attached.

Teleoperation here has no leader arm: the operator moves a *commanded pose* held
in software and the arm follows it. This module is the following itself —
everything that decides whether the real arm may move, and how far, in one place
that a unit test can drive:

  * **clamping** — a commanded pose outside the joint's soft limits is clamped
    before it ever becomes a goal (the driver clamps again; this one exists so
    the operator sees the limit rather than discovering it downstream),
  * **rate limiting** — the goal advances towards the commanded pose by at most
    ``max_velocity * dt``, so a command that jumps (a slider dragged, a frontend
    restarted at a different pose) produces a ramp, never a lunge,
  * **the deadman** — the command's *liveness* is the deadman. A frontend offers
    a pose only while the operator is actually driving it, so input that stops —
    a released key, a closed window, an SSH session dropped mid-move — all
    arrive as the same thing: no fresh pose. Motion then halts,
  * **the panic latch** — an engaged e-stop suppresses every goal until it is
    explicitly cleared, so torque coming back on cannot restart the move.

Resuming after a hold re-seeds the commanded pose from where the arm *actually*
is. Without that, a pause would bank up the difference and pay it out as a jump
on resume.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass

from mote_arm.config import JointSpec
from mote_arm.motion import advance


@dataclass(frozen=True)
class FollowLimits:
    """How fast the arm may chase the commanded pose, and when it stops trying."""

    # Radians per second the goal may advance. Deliberately at or above the
    # frontend's own speed, so the arm is never left with a backlog of commanded
    # motion to work through after the operator stops.
    max_velocity: float = 0.5
    # Seconds without a fresh pose before motion halts. Long enough to cover a
    # terminal's key-repeat gap, short enough that a released key stops the arm
    # while it is still obviously connected to the key.
    deadman_timeout: float = 0.4
    # Radians the commanded pose may lead the measured one. A position servo
    # that cannot reach its target -- gravity on the lifting joint, a hand on
    # the arm, something in the way -- does not say so; it simply sits there
    # while the command runs away, straining harder every tick against a target
    # it will never meet, and every re-seed then snaps the command back. So the
    # command is not allowed to run away: it waits for the arm. Well above the
    # 0.01-0.03 rad of ordinary proportional droop, so normal following is
    # untouched.
    max_lag: float = 0.15

    def __post_init__(self) -> None:
        if self.max_velocity <= 0:
            raise ValueError("max_velocity must be positive")
        if self.deadman_timeout <= 0:
            raise ValueError("deadman_timeout must be positive")
        if self.max_lag <= 0:
            raise ValueError("max_lag must be positive")


# What the follower is doing, for logging and for tests to assert on.
TRACKING = "tracking"
HOLDING = "holding"  # deadman: no fresh commanded pose
ESTOPPED = "estopped"
WAITING = "waiting"  # no measured state yet, so nothing is safe to command


class PoseFollower:
    """Turns commanded poses into rate-limited, clamped goals for the arm."""

    def __init__(
        self,
        joints: Sequence[JointSpec],
        limits: FollowLimits | None = None,
    ):
        self._joints = {j.name: j for j in joints}
        self.limits = limits or FollowLimits()
        self._measured: dict[str, float] = {}
        self._command: dict[str, float] = {}
        self._command_stamp: float | None = None
        self._commanded: dict[str, float] = {}
        self._estop = False
        # True when the commanded pose is not trustworthy as a starting point —
        # at startup, and after any hold — so the next goal re-seeds from the
        # follower's measured pose instead of resuming from a stale command.
        self._reseed = True
        # Set when a hold begins, so the mirror can issue one goal at the
        # arm's present position: that halts the residual travel towards the
        # last setpoint instead of letting it coast there.
        self._halt_pending = False
        # Joints whose command is being held back because the arm is not
        # following. Empty is the normal case.
        self.stalled: list[str] = []
        self.state = WAITING

    @property
    def estopped(self) -> bool:
        return self._estop

    @property
    def commanded(self) -> dict[str, float]:
        """The pose being asked for — read-only, for diagnostics."""
        return dict(self._commanded)

    @property
    def measured(self) -> dict[str, float]:
        """The pose last reported by the arm — read-only, for diagnostics."""
        return dict(self._measured)

    def on_command(self, pose: Mapping[str, float], now: float) -> None:
        """Record a commanded pose. Unknown joint names are ignored."""
        self._command = {n: v for n, v in pose.items() if n in self._joints}
        if self._command:
            self._command_stamp = now

    def on_measured(self, pose: Mapping[str, float]) -> None:
        """Record where the arm actually is."""
        for name, value in pose.items():
            if name in self._joints:
                self._measured[name] = value

    def set_estop(self, engaged: bool, now: float) -> None:
        """Engage or clear the panic latch.

        Clearing does not resume motion by itself: the commanded pose is marked
        for re-seeding, so the arm picks up from where it is rather than from
        where it was heading when the operator hit the panic key.
        """
        if engaged and not self._estop:
            self._commanded = {}
        if not engaged and self._estop:
            self._reseed = True
        self._estop = engaged

    def update(self, now: float, dt: float) -> dict[str, float] | None:
        """Advance one tick; returns the goal to publish, or None to send nothing.

        Sending nothing is how the arm stops: the driver holds its last goal, so
        an absent command is a hold, not a drift.
        """
        if self._estop:
            self.state = ESTOPPED
            return None
        if not self._measured:
            self.state = WAITING
            return None

        stale = (
            self._command_stamp is None
            or (now - self._command_stamp) > self.limits.deadman_timeout
        )
        if stale:
            if self.state == TRACKING:
                self._halt_pending = True
            self._reseed = True
            self.state = HOLDING
            if self._halt_pending:
                self._halt_pending = False
                # One goal at the present position: stop here, don't coast on
                # to the setpoint the arm was still travelling towards.
                return dict(self._measured)
            return None

        self._halt_pending = False
        if self._reseed or not self._commanded:
            self._commanded = dict(self._measured)
            self._reseed = False

        max_step = self.limits.max_velocity * max(0.0, dt)
        goal: dict[str, float] = {}
        stalled: list[str] = []
        for name, target in self._command.items():
            joint = self._joints[name]
            start = self._commanded.get(name, self._measured.get(name))
            if start is None:
                continue
            stepped = advance(start, joint.clamp_rad(target), max_step)
            # Never lead the arm by more than max_lag: if it is not following,
            # the command stops advancing and waits rather than running away.
            here = self._measured.get(name)
            if here is not None:
                stepped = max(
                    here - self.limits.max_lag, min(here + self.limits.max_lag, stepped)
                )
                if abs(stepped - here) >= self.limits.max_lag - 1e-9:
                    stalled.append(name)
            self._commanded[name] = stepped
            goal[name] = stepped

        self.stalled = sorted(stalled)
        self.state = TRACKING
        return goal or None


def sync_pose(
    measured: Mapping[str, float], joints: Sequence[JointSpec]
) -> dict[str, float]:
    """The commanded pose when it re-syncs to the arm: measured, then clamped.

    Re-syncing to an arm sitting fractionally outside its soft band (limits are
    taught, and a servo droops) would otherwise hand back a pose the follower
    immediately clamps, showing a command that cannot be where it says.
    """
    return {j.name: j.clamp_rad(measured[j.name]) for j in joints if j.name in measured}
