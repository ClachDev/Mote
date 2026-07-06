"""End-to-end tick of the fetch tree against a mock navigate_to_pose server.

No Gazebo or Nav2 required: the mock server accepts every goal and succeeds
immediately, so the test exercises command parsing, blackboard wiring, both
DriveTo behaviours, the pick/place stubs, and outcome reporting.
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
  dropoff: {x: -1.0, y: 0.5, yaw: 3.14}
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
    # A private DDS domain so a live robot/sim session on this machine can't
    # cross-talk with the test's task_server and mock nav server.
    os.environ["ROS_DOMAIN_ID"] = str(random.randint(60, 100))
    rclpy.init()
    yield
    rclpy.shutdown()


def spin_until(executor, condition, timeout=30.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return True
        executor.spin_once(timeout_sec=0.05)
    return condition()


def test_fetch_round_trip(ros, tmp_path):
    zones_file = tmp_path / "zones.yaml"
    zones_file.write_text(ZONES)
    server = TaskServer(
        parameter_overrides=[
            Parameter("zones_file", value=str(zones_file)),
            Parameter("pick_duration", value=0.1),
            Parameter("place_duration", value=0.1),
        ]
    )
    mock = MockNav()
    executor = SingleThreadedExecutor()
    executor.add_node(server)
    executor.add_node(mock)

    assert spin_until(
        executor, lambda: mock.command_pub.get_subscription_count() > 0
    ), "task_server never subscribed to task/command"

    mock.command_pub.publish(String(data="fetch pickup nowhere"))
    assert spin_until(
        executor, lambda: any(s.startswith("rejected") for s in mock.statuses)
    ), mock.statuses

    mock.command_pub.publish(String(data="fetch pickup dropoff"))
    assert spin_until(
        executor, lambda: any(s.startswith("succeeded") for s in mock.statuses)
    ), mock.statuses

    assert len(mock.goals) == 2, mock.goals
    assert mock.goals[0].pose.position.x == pytest.approx(1.0)
    assert mock.goals[0].pose.position.y == pytest.approx(2.0)
    assert mock.goals[1].pose.position.x == pytest.approx(-1.0)
    assert mock.goals[0].header.frame_id == "map"

    server.destroy_node()
    mock.destroy_node()
