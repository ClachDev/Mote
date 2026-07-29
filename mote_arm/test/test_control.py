"""What the arm's command clients put on the wire.

`mote_arm.control` is the one place that knows how to talk to `arm_controller`,
so the message shape is worth pinning: a single-point trajectory, named joints,
and a start time of zero — which is what tells the controller "start now" on the
robot's clock rather than on the operator's.
"""

from mote_arm.config import JointSpec
from mote_arm.control import ARM_CONTROLLER, TRAJECTORY_TOPIC, duration_msg, trajectory
from mote_arm.jog import MIN_MOVE_TIME_S, move_time, next_target


def test_trajectory_names_the_joints_it_moves():
    msg = trajectory({"elbow_flex": -1.0, "shoulder_pan": 0.1}, 2.0)
    assert set(msg.joint_names) == {"elbow_flex", "shoulder_pan"}
    assert len(msg.points) == 1
    # Positions must line up with joint_names, not with any other ordering.
    for name, position in zip(msg.joint_names, msg.points[0].positions):
        assert position == {"elbow_flex": -1.0, "shoulder_pan": 0.1}[name]


def test_trajectory_starts_now():
    msg = trajectory({"gripper": 0.0}, 1.0)
    # A zero header stamp means "start now" on the robot's clock; stamping it
    # here would put the start of an arm move on the operator's clock.
    assert msg.header.stamp.sec == 0
    assert msg.header.stamp.nanosec == 0


def test_duration_splits_seconds_and_nanoseconds():
    d = duration_msg(1.25)
    assert d.sec == 1
    assert d.nanosec == 250_000_000
    assert duration_msg(0.5).sec == 0


def test_trajectory_carries_the_move_time():
    msg = trajectory({"gripper": 0.0}, 2.5)
    assert msg.points[0].time_from_start.sec == 2
    assert msg.points[0].time_from_start.nanosec == 500_000_000


def test_topic_is_the_controller_s_own():
    assert TRAJECTORY_TOPIC == f"{ARM_CONTROLLER}/joint_trajectory"


def test_move_time_scales_with_distance():
    # A jog must not ask the arm to travel further in the same time; the servos
    # have their own speed cap and a trajectory faster than it just runs ahead.
    assert move_time(2.0) > move_time(0.5)


def test_move_time_has_a_floor():
    assert move_time(0.0) == MIN_MOVE_TIME_S
    assert move_time(-0.001) == MIN_MOVE_TIME_S


def test_jog_step_is_still_clamped_before_it_is_sent():
    j = JointSpec("j", 1, min_rad=-1.0, max_rad=1.0)
    assert next_target(0.98, 0.05, j) == 1.0
