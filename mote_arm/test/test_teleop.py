"""The follow rule: clamping, rate limiting, the deadman, and the panic latch.

Every safety property of virtual-leader teleop is decided in ``LeaderMirror``,
so it is all checked here — with no bus, no driver and no terminal.
"""

import pytest

from mote_arm.config import JointSpec
from mote_arm.teleop import (
    ESTOPPED,
    HOLDING,
    TRACKING,
    WAITING,
    LeaderMirror,
    MirrorLimits,
    sync_pose,
)

JOINTS = (
    JointSpec(name="elbow_flex", id=3, min_rad=-1.0, max_rad=1.0),
    JointSpec(name="wrist_roll", id=5, min_rad=-0.1, max_rad=0.1),
)
LIMITS = MirrorLimits(max_velocity=1.0, deadman_timeout=0.4)
DT = 0.05


def mirror(**kwargs) -> LeaderMirror:
    return LeaderMirror(JOINTS, MirrorLimits(**{**LIMITS.__dict__, **kwargs}))


def drive(
    m: LeaderMirror, leader: dict, seconds: float, start: float = 0.0
) -> dict | None:
    """Feed a steady leader pose for ``seconds`` and return the last goal."""
    goal = None
    ticks = int(round(seconds / DT))
    for i in range(ticks):
        now = start + i * DT
        m.on_leader(leader, now)
        goal = m.update(now, DT)
    return goal


def test_nothing_is_commanded_before_the_arm_reports():
    m = mirror()
    m.on_leader({"elbow_flex": 0.5}, 0.0)
    assert m.update(0.0, DT) is None
    assert m.state == WAITING


def test_goal_advances_at_the_rate_limit():
    m = mirror(max_velocity=1.0)
    m.on_measured({"elbow_flex": 0.0, "wrist_roll": 0.0})
    m.on_leader({"elbow_flex": 1.0}, 0.0)
    # One tick of 50 ms at 1 rad/s is 0.05 rad, however far away the leader is.
    assert m.update(0.0, DT)["elbow_flex"] == pytest.approx(0.05)
    assert m.update(DT, DT)["elbow_flex"] == pytest.approx(0.10)


def test_a_leader_jump_becomes_a_ramp_not_a_lunge():
    m = mirror(max_velocity=0.5)
    m.on_measured({"elbow_flex": 0.0})
    # A slider dragged to the far end, or a frontend restarted at a different
    # pose, is exactly this: one enormous step in the leader's position.
    goal = drive(m, {"elbow_flex": 1.0}, seconds=0.2)
    assert goal["elbow_flex"] == pytest.approx(0.1, abs=1e-9)
    assert m.state == TRACKING


def test_goals_are_clamped_to_the_soft_limits():
    m = mirror()
    m.on_measured({"wrist_roll": 0.0})
    goal = drive(m, {"wrist_roll": 5.0}, seconds=2.0)
    assert goal["wrist_roll"] == pytest.approx(0.1)


def test_deadman_halts_at_the_arms_position_then_sends_nothing():
    m = mirror(deadman_timeout=0.2)
    m.on_measured({"elbow_flex": 0.0})
    drive(m, {"elbow_flex": 1.0}, seconds=0.2)
    m.on_measured({"elbow_flex": 0.15})

    # First tick past the deadman: one goal at where the arm *is*, so it stops
    # there instead of coasting on to the setpoint it was travelling towards.
    halt = m.update(1.0, DT)
    assert m.state == HOLDING
    assert halt == pytest.approx({"elbow_flex": 0.15})
    # And then silence: an absent goal is a hold, because the driver keeps the
    # last one it was given.
    assert m.update(1.05, DT) is None
    assert m.update(1.10, DT) is None


def test_resuming_starts_from_the_arm_not_from_the_stale_command():
    m = mirror(max_velocity=1.0, deadman_timeout=0.2)
    m.on_measured({"elbow_flex": 0.0})
    drive(m, {"elbow_flex": 1.0}, seconds=0.5)  # commanded is now ~0.5
    m.update(2.0, DT)  # deadman

    # The arm settled short of the last command, as a real servo does.
    m.on_measured({"elbow_flex": 0.42})
    resumed = drive(m, {"elbow_flex": 1.0}, seconds=DT, start=3.0)
    # One tick past 0.42, not a jump back to the 0.5 it had banked up.
    assert resumed["elbow_flex"] == pytest.approx(0.47)


def test_estop_suppresses_goals_and_latches():
    m = mirror()
    m.on_measured({"elbow_flex": 0.0})
    drive(m, {"elbow_flex": 1.0}, seconds=0.2)

    m.set_estop(True, 1.0)
    assert m.estopped
    # Input keeps arriving — the whole point of a latch is that this changes
    # nothing until someone clears it.
    assert drive(m, {"elbow_flex": 1.0}, seconds=1.0, start=1.0) is None
    assert m.state == ESTOPPED


def test_clearing_the_estop_resumes_from_the_arms_position():
    m = mirror(max_velocity=1.0)
    m.on_measured({"elbow_flex": 0.0})
    drive(m, {"elbow_flex": 1.0}, seconds=0.5)
    m.set_estop(True, 1.0)
    m.on_measured({"elbow_flex": 0.3})
    m.set_estop(False, 2.0)

    resumed = drive(m, {"elbow_flex": 1.0}, seconds=DT, start=2.0)
    assert resumed["elbow_flex"] == pytest.approx(0.35)


def test_unknown_leader_joints_are_ignored():
    m = mirror()
    m.on_measured({"elbow_flex": 0.0})
    goal = drive(m, {"elbow_flex": 0.5, "not_a_joint": 9.9}, seconds=0.1)
    assert set(goal) == {"elbow_flex"}


def test_a_leader_pose_of_only_unknown_joints_commands_nothing():
    m = mirror()
    m.on_measured({"elbow_flex": 0.0})
    assert drive(m, {"gripper_of_another_robot": 1.0}, seconds=0.1) is None


def test_sync_pose_clamps_a_drooping_arm_into_the_band():
    # Limits are taught and servos droop, so the arm can sit fractionally
    # outside its own band; a leader synced to that would show a pose the
    # mirror immediately clamps.
    assert sync_pose({"wrist_roll": 0.15}, JOINTS) == pytest.approx({"wrist_roll": 0.1})


def test_limits_must_be_positive():
    with pytest.raises(ValueError):
        MirrorLimits(max_velocity=0.0)
    with pytest.raises(ValueError):
        MirrorLimits(deadman_timeout=-1.0)
