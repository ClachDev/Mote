"""Keyboard teleop end to end, against a control stack that isn't there.

``mock_arm`` presents the interface ros2_control does — a trajectory topic and
``switch_controller`` — with no bus behind it, so the whole path (commanded pose
-> safety rules -> arm_controller -> arm) runs in one process and every safety
behaviour can be checked before anyone stands at the bench.

The mirror is driven the way it is in production: the executor spins on a
worker thread and ``tick()`` is called from this one. That is not a test
convenience — taking hold of the arm is a ``switch_controller`` call, and a
service call made from inside an executor callback can never complete, because
the future is resolved by the executor the callback is blocking.

The keyboard is the one thing stubbed: these set ``pose`` where a held key
would, which is exactly what ``ArmTeleop.step`` does.

A random ROS_DOMAIN_ID keeps these nodes off a live robot's graph (they command
``arm_controller``, which moves a real arm), and a per-process namespace keeps
them off sibling test sessions colcon runs in parallel.
"""

import os
import random
import threading
import time
from argparse import Namespace

os.environ["ROS_DOMAIN_ID"] = str(random.randint(60, 100))

import pytest  # noqa: E402
import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from mote_arm import arm_teleop as teleop_mod  # noqa: E402
from mote_arm import config, mock_arm as mock_mod  # noqa: E402

CFG = config.ArmConfig.from_dict(
    {
        "arm": {
            "port": "/dev/fake",
            "baud_rate": 1000000,
            "joints": [
                {"name": "elbow_flex", "id": 3, "min": -1.0, "max": 1.0, "zero": 2048},
                {"name": "wrist_roll", "id": 5, "min": -0.1, "max": 0.1, "zero": 2048},
            ],
        }
    }
)

MOCK_ARGS = Namespace(
    rate=50.0, speed=2.0, droop=0.0, camera=False, camera_rate=10.0, camera_size=(8, 8)
)


class Stack:
    """The teleop node, its follower, and the two loops that drive them."""

    def __init__(self, mock, teleop, executor):
        self.mock = mock
        self.teleop = teleop
        self._executor = executor
        self.pose: dict[str, float] | None = None

    def run(self, seconds: float) -> None:
        """Advance both loops, as `main` runs them on two threads."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if self.pose is not None:
                # Where a held key would put it. Offered every pass, because a
                # pose offered once and then not again is the operator letting
                # go, which is what the deadman is for.
                self.teleop.pose = dict(self.pose)
                self.teleop.offer()
            self.teleop.tick()
            time.sleep(self.teleop.period)

    def panic(self, engaged: bool) -> None:
        self.teleop.set_estop(engaged)

    def at(self, joint: str) -> float:
        return self.mock.position[joint]


@pytest.fixture
def stack(monkeypatch):
    monkeypatch.setattr(config, "load", lambda: CFG)
    rclpy.init(args=["--ros-args", "-r", f"__ns:=/test_{os.getpid()}"])
    mock = mock_mod.MockArm(MOCK_ARGS)
    teleop = teleop_mod.ArmTeleop(speed=0.25, key_timeout=0.35)

    executor = SingleThreadedExecutor()
    for node in (mock, teleop):
        executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    built = Stack(mock, teleop, executor)
    # Let the mock's first joint_states arrive: the mirror refuses to command an
    # arm it has not heard from.
    time.sleep(0.3)
    yield built

    executor.shutdown()
    spinner.join(timeout=2.0)
    for node in (mock, teleop):
        node.destroy_node()
    rclpy.shutdown()


def test_the_arm_follows_the_keyboard(stack):
    start = stack.at("elbow_flex")
    stack.pose = {"elbow_flex": 0.6}
    stack.run(0.6)
    assert stack.at("elbow_flex") > start + 0.1


def test_commanding_takes_hold_of_a_limp_arm(stack):
    # The mock starts with arm_controller inactive, exactly as the real stack
    # spawns it; the first command is what makes the hardware take hold.
    assert stack.mock.holding is False
    stack.pose = {"elbow_flex": 0.4}
    stack.run(0.3)
    assert stack.mock.holding is True


def test_following_is_rate_limited_not_instant(stack):
    # A command that jumps must not become an arm that jumps: the default 0.5
    # rad/s over ~0.5 s is a few tenths of a radian, nowhere near the target.
    stack.pose = {"elbow_flex": 1.0}
    stack.run(0.5)
    assert stack.at("elbow_flex") < 0.45


def test_releasing_the_input_halts_the_arm(stack):
    stack.pose = {"elbow_flex": 1.0}
    stack.run(0.6)
    stack.pose = None  # the operator let go
    stack.run(0.6)

    halted = stack.at("elbow_flex")
    stack.run(0.6)
    assert stack.at("elbow_flex") == pytest.approx(halted, abs=1e-6)


def test_goals_are_clamped_to_the_soft_limits(stack):
    stack.pose = {"wrist_roll": 5.0}
    stack.run(1.2)
    assert stack.at("wrist_roll") == pytest.approx(0.1, abs=1e-3)


def test_panic_drops_torque_and_the_arm_stops_even_while_driven(stack):
    stack.pose = {"elbow_flex": 1.0}
    stack.run(0.4)
    stack.panic(True)
    stack.run(0.4)
    # Torque is controller activation, so dropping it means deactivating.
    assert stack.mock.holding is False

    # The pose keeps being offered throughout: the latch, not the absence of
    # input, is what holds the arm.
    stopped = stack.at("elbow_flex")
    stack.run(0.6)
    assert stack.at("elbow_flex") == pytest.approx(stopped, abs=1e-6)


def test_clearing_panic_lets_the_arm_move_again(stack):
    stack.pose = {"elbow_flex": 1.0}
    stack.run(0.3)
    stack.panic(True)
    stack.run(0.3)
    stopped = stack.at("elbow_flex")

    stack.panic(False)
    stack.run(0.6)
    assert stack.at("elbow_flex") > stopped + 0.05


# --- step mode: what `arm-jog` was for, on the path that has the safety rules


def test_a_press_advances_the_pose_by_exactly_one_step(stack):
    stack.teleop.sync()
    before = stack.teleop.pose["elbow_flex"]
    assert stack.teleop.nudge("elbow_flex", +1.0, 0.05, 100.0) is True
    assert stack.teleop.pose["elbow_flex"] == pytest.approx(before + 0.05)


def test_a_key_repeat_does_not_step_again(stack):
    """A terminal cannot tell a repeat from a press, so holding steps once."""
    stack.teleop.sync()
    assert stack.teleop.nudge("elbow_flex", +1.0, 0.05, 100.0) is True
    stepped = stack.teleop.pose["elbow_flex"]
    for repeat in (100.03, 100.1, 100.3):
        assert stack.teleop.nudge("elbow_flex", +1.0, 0.05, repeat) is False
    assert stack.teleop.pose["elbow_flex"] == pytest.approx(stepped)


def test_releasing_and_pressing_again_steps_again(stack):
    stack.teleop.sync()
    stack.teleop.nudge("elbow_flex", +1.0, 0.05, 100.0)
    later = 100.0 + stack.teleop.key_timeout + 0.01
    assert stack.teleop.nudge("elbow_flex", +1.0, 0.05, later) is True


def test_a_step_is_clamped_like_any_other_command(stack):
    stack.teleop.sync()
    for press in range(50):
        stack.teleop.nudge("wrist_roll", +1.0, 0.05, 100.0 + press)
    assert stack.teleop.pose["wrist_roll"] == pytest.approx(0.1)


def test_a_step_is_offered_long_enough_for_the_arm_to_walk_it(stack):
    """Offered once, the deadman would stop the arm short of the increment."""
    limit = stack.teleop.mirror.limits.max_velocity
    assert stack.teleop.settle_time(0.05) == pytest.approx(0.05 / limit)
    assert stack.teleop.settle_time(0.5) > stack.teleop.key_timeout


def test_stepping_moves_the_arm_by_about_the_step(stack):
    start = stack.at("elbow_flex")
    stack.teleop.sync()
    stack.teleop.nudge("elbow_flex", +1.0, 0.1, time.monotonic())
    stack.pose = dict(stack.teleop.pose)
    stack.run(stack.teleop.settle_time(0.1) + 0.4)
    assert stack.at("elbow_flex") == pytest.approx(start + 0.1, abs=0.03)
