"""End-to-end tick of the goto tree against a mock navigate_to_pose server.

No Gazebo or Nav2 required: the mock server accepts every goal and succeeds
immediately, so the test exercises command parsing, zone lookup, the DriveTo
behaviour, and outcome reporting. It shares a task_server with the fetch tree,
so it also checks the command dispatch (goto vs fetch vs unknown).
"""

import os
import random
import time

import pytest
import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

from mote_tasks.task_server import TaskServer

ZONES = """\
frame_id: map
zones:
  pickup: {x: 1.0, y: 2.0}
  kitchen: {x: -1.5, y: 0.5, yaw: 3.14, radius: 1.5}
  lab: {x: 4.0, y: -2.0}
"""


class MockNav(Node):
    def __init__(self):
        super().__init__("mock_nav")
        self.goals = []
        self.statuses = []
        self.server = ActionServer(
            self, NavigateToPose, "navigate_to_pose", self.execute
        )
        self.command_pub = self.create_publisher(String, "task/command", 1)
        self.create_subscription(
            String, "task/status", lambda m: self.statuses.append(m.data), 10
        )

    def execute(self, goal_handle):
        self.goals.append(goal_handle.request.pose)
        goal_handle.succeed()
        return NavigateToPose.Result()


@pytest.fixture
def ros():
    # Nothing outside this test may reach its task_server and mock nav server:
    # a high DDS domain keeps a live robot/sim session on this machine out, and
    # a per-process namespace keeps sibling test sessions out — colcon runs
    # package tests in parallel, and mote_fleet drives the same topic names.
    os.environ["ROS_DOMAIN_ID"] = str(random.randint(60, 100))
    rclpy.init(args=["--ros-args", "-r", f"__ns:=/test_{os.getpid()}"])
    yield
    rclpy.shutdown()


def spin_until(executor, condition, timeout=30.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return True
        executor.spin_once(timeout_sec=0.05)
    return condition()


def test_goto_round_trip(ros, tmp_path):
    zones_file = tmp_path / "zones.yaml"
    zones_file.write_text(ZONES)
    server = TaskServer(
        parameter_overrides=[Parameter("zones_file", value=str(zones_file))]
    )
    mock = MockNav()
    executor = SingleThreadedExecutor()
    executor.add_node(server)
    executor.add_node(mock)

    assert spin_until(
        executor, lambda: mock.command_pub.get_subscription_count() > 0
    ), "task_server never subscribed to task/command"

    mock.command_pub.publish(String(data="goto nowhere"))
    assert spin_until(
        executor, lambda: any(s.startswith("rejected") for s in mock.statuses)
    ), mock.statuses

    mock.command_pub.publish(String(data="wander kitchen"))
    assert spin_until(
        executor,
        lambda: sum(s.startswith("rejected") for s in mock.statuses) >= 2,
    ), mock.statuses

    mock.command_pub.publish(String(data="goto kitchen"))
    assert spin_until(
        executor, lambda: any(s.startswith("succeeded") for s in mock.statuses)
    ), mock.statuses

    assert len(mock.goals) == 1, mock.goals
    assert mock.goals[0].pose.position.x == pytest.approx(-1.5)
    assert mock.goals[0].pose.position.y == pytest.approx(0.5)
    assert mock.goals[0].header.frame_id == "map"

    server.destroy_node()
    mock.destroy_node()


def test_idle_tick_rate_does_not_delay_the_mission(ros, tmp_path):
    """The tree ticks slowly between missions, and instantly once given one.

    Between missions the tree ticks WaitForTask and nothing else, so it runs at
    ``idle_tick_period``. The saving is only free if accepting a command
    restores the mission rate *and* resets the timer, because setting a period
    does not move the expiry already pending: an idling timer switched to the
    mission rate still has the rest of its idle period to wait, and the first
    tick of the accepted tree — the one that sends the Nav2 goal — is what
    would wait. So the command is deliberately sent just *after* an idle tick,
    with a whole idle period pending, which is the case the reset exists for.
    """
    zones_file = tmp_path / "zones.yaml"
    zones_file.write_text(ZONES)
    server = TaskServer(
        parameter_overrides=[
            Parameter("zones_file", value=str(zones_file)),
            Parameter("tick_period", value=0.05),
            Parameter("idle_tick_period", value=2.0),
        ]
    )
    mock = MockNav()
    executor = SingleThreadedExecutor()
    executor.add_node(server)
    executor.add_node(mock)

    def period_s():
        return server.tick_timer.timer_period_ns / 1e9

    def next_call_s():
        return server.tick_timer.time_until_next_call() / 1e9

    assert period_s() == pytest.approx(2.0), "an idle tree is ticking at mission rate"

    assert spin_until(
        executor, lambda: mock.command_pub.get_subscription_count() > 0
    ), "task_server never subscribed to task/command"

    # An idle tick has just fired, so a full idle period is pending.
    assert spin_until(executor, lambda: next_call_s() > 1.5), "no idle tick fired"

    mock.command_pub.publish(String(data="goto kitchen"))
    assert spin_until(
        executor, lambda: any(s.startswith("accepted") for s in mock.statuses)
    ), mock.statuses
    accepted_at = time.monotonic()
    assert period_s() == pytest.approx(0.05), "an accepted task is ticking at idle rate"
    assert next_call_s() <= 0.1, (
        f"the accepted tree waits {next_call_s():.2f}s for its first tick — "
        "the period changed but the pending idle expiry did not"
    )

    assert spin_until(
        executor, lambda: any(s.startswith("succeeded") for s in mock.statuses)
    ), mock.statuses
    elapsed = time.monotonic() - accepted_at
    assert elapsed < 1.5, f"the mission took {elapsed:.2f}s at the mission rate"

    # A finished mission hands the idle rate back, or the saving lasts one task.
    assert period_s() == pytest.approx(2.0), "the tree kept ticking after the task"

    server.destroy_node()
    mock.destroy_node()


def test_idle_rate_is_floored_at_the_mission_rate(ros, tmp_path):
    """An idle rate faster than the mission rate is a contradiction, not a config."""
    zones_file = tmp_path / "zones.yaml"
    zones_file.write_text(ZONES)
    server = TaskServer(
        parameter_overrides=[
            Parameter("zones_file", value=str(zones_file)),
            Parameter("tick_period", value=0.5),
            Parameter("idle_tick_period", value=0.1),
        ]
    )
    assert server.idle_tick_period == pytest.approx(0.5)
    assert server.tick_timer.timer_period_ns / 1e9 == pytest.approx(0.5)
    server.destroy_node()
