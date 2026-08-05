"""The virtual leader's follow rule, with no ROS and no hardware attached.

Teleoperation here is leader-follower without a leader arm: a *virtual leader*
is a pose held in software that an operator moves, and the follower mirrors it.
This module is the mirroring itself — everything that decides whether the real
arm may move, and how far, in one place that a unit test can drive:

  * **clamping** — a leader pose outside the joint's soft limits is clamped
    before it ever becomes a goal (the driver clamps again; this one exists so
    the operator sees the limit rather than discovering it downstream),
  * **rate limiting** — the commanded pose advances towards the leader by at
    most ``max_velocity * dt``, so a leader that jumps (a slider dragged, a
    frontend restarted at a different pose) produces a ramp, never a lunge,
  * **the deadman** — the leader's *liveness* is the deadman. A frontend
    publishes only while the operator is actually driving it, so input that
    stops — a released key, a closed window, an SSH session dropped mid-move —
    all arrive as the same thing: no fresh leader pose. Motion then halts,
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
class MirrorLimits:
    """How fast the follower may chase the leader, and when it stops trying."""

    # Radians per second the commanded pose may advance. Deliberately at or
    # above the virtual leader's own speed, so the follower is never left with
    # a backlog of leader motion to work through after the operator stops.
    max_velocity: float = 0.5
    # Seconds without a leader pose before motion halts. Long enough to cover a
    # terminal's key-repeat gap, short enough that a released key stops the arm
    # while it is still obviously connected to the key.
    deadman_timeout: float = 0.4

    def __post_init__(self) -> None:
        if self.max_velocity <= 0:
            raise ValueError("max_velocity must be positive")
        if self.deadman_timeout <= 0:
            raise ValueError("deadman_timeout must be positive")


# What the mirror is doing, for logging and for tests to assert on.
TRACKING = "tracking"
HOLDING = "holding"  # deadman: no fresh leader pose
ESTOPPED = "estopped"
WAITING = "waiting"  # no follower state yet, so nothing is safe to command


class LeaderMirror:
    """Turns virtual-leader poses into rate-limited, clamped follower goals."""

    def __init__(
        self,
        joints: Sequence[JointSpec],
        limits: MirrorLimits | None = None,
    ):
        self._joints = {j.name: j for j in joints}
        self.limits = limits or MirrorLimits()
        self._measured: dict[str, float] = {}
        self._leader: dict[str, float] = {}
        self._leader_stamp: float | None = None
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
        self.state = WAITING

    @property
    def estopped(self) -> bool:
        return self._estop

    def on_leader(self, pose: Mapping[str, float], now: float) -> None:
        """Record a virtual-leader pose. Unknown joint names are ignored."""
        self._leader = {n: v for n, v in pose.items() if n in self._joints}
        if self._leader:
            self._leader_stamp = now

    def on_measured(self, pose: Mapping[str, float]) -> None:
        """Record where the follower actually is."""
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
            self._leader_stamp is None
            or (now - self._leader_stamp) > self.limits.deadman_timeout
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
        for name, target in self._leader.items():
            joint = self._joints[name]
            start = self._commanded.get(name, self._measured.get(name))
            if start is None:
                continue
            stepped = advance(start, joint.clamp_rad(target), max_step)
            self._commanded[name] = stepped
            goal[name] = stepped

        self.state = TRACKING
        return goal or None


def sync_pose(
    measured: Mapping[str, float], joints: Sequence[JointSpec]
) -> dict[str, float]:
    """The virtual leader's pose when it re-syncs to the arm: measured, clamped.

    A leader re-synced to a follower sitting fractionally outside its soft band
    (limits are taught, and a servo droops) would otherwise hand back a pose the
    mirror immediately clamps, showing a leader that cannot be where it says.
    """
    return {j.name: j.clamp_rad(measured[j.name]) for j in joints if j.name in measured}
