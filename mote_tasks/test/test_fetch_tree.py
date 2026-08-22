"""End-to-end tick of the fetch tree against a mock navigate_to_pose server.

No Gazebo or Nav2 required: the mock server accepts every goal and succeeds
immediately, so the test exercises input validation, blackboard wiring, both
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

import mission_harness as harness
from mote_bringup.spec import mission

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
        harness.collect(self, self.statuses)
        harness.localise(self)

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


def test_fetch_round_trip(ros, tmp_path):
    zones_file = tmp_path / "zones.yaml"
    zones_file.write_text(ZONES)
    server = TaskServer(
        parameter_overrides=[
            Parameter("zones_file", value=str(zones_file)),
            Parameter("platform_id", value=harness.PLATFORM),
            Parameter("pick_duration", value=0.1),
            Parameter("place_duration", value=0.1),
        ]
    )
    mock = MockNav()
    executor = SingleThreadedExecutor()
    executor.add_node(server)
    executor.add_node(mock)

    harness.ready(executor, server, mock.command_pub)

    harness.send(
        mock.command_pub, "fetch", {"target": "pickup", "destination": "nowhere"}
    )
    assert spin_until(executor, lambda: harness.failures(mock.statuses)), mock.statuses
    assert harness.failures(mock.statuses)[-1] == (
        mission.REJECTED,
        mission.UNRESOLVED_ZONE,
    )

    harness.send(
        mock.command_pub, "fetch", {"target": "pickup", "destination": "dropoff"}
    )
    assert spin_until(
        executor, lambda: mission.SUCCEEDED in harness.states(mock.statuses)
    ), mock.statuses

    assert len(mock.goals) == 2, mock.goals
    assert mock.goals[0].pose.position.x == pytest.approx(1.0)
    assert mock.goals[0].pose.position.y == pytest.approx(2.0)
    assert mock.goals[1].pose.position.x == pytest.approx(-1.0)
    assert mock.goals[0].header.frame_id == "map"

    server.destroy_node()
    mock.destroy_node()
