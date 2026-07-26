"""Fetch-by-label round trip against mock Nav2 and a mock detector.

No Gazebo, Nav2, or detection server: the mock detector answers any label with
a detection at a fixed map point, so the test exercises the label grammar,
AcquireObject's publish/wait/standoff logic, label clearing, and the unchanged
zone path driving to the drop.
"""

import math
import os
import random
import time

import pytest
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from mote_tasks.behaviours.perception import LABELS_QOS
from mote_tasks.task_server import TaskServer

ZONES = """\
frame_id: map
zones:
  dropoff: {x: -1.0, y: 0.5, yaw: 3.14}
"""

OBJECT_XY = (2.0, 0.0)


class MockWorld(Node):
    """Mock Nav2 + mock detector + a static map->base_footprint at the origin."""

    def __init__(self):
        super().__init__("mock_world")
        self.goals = []
        self.statuses = []
        self.labels = []
        self.server = ActionServer(
            self, NavigateToPose, "navigate_to_pose", self.execute
        )
        self.command_pub = self.create_publisher(String, "task/command", 1)
        self.create_subscription(
            String, "task/status", lambda m: self.statuses.append(m.data), 10
        )
        self.create_subscription(String, "detect/labels", self._on_labels, LABELS_QOS)
        self.detections_pub = self.create_publisher(
            Detection3DArray, "detected_objects", 5
        )
        self.create_timer(0.05, self._tick_detector)

        tf = TransformStamped()
        tf.header.frame_id = "map"
        tf.child_frame_id = "base_footprint"
        tf.transform.rotation.w = 1.0
        tf2_ros.StaticTransformBroadcaster(self).sendTransform(tf)

    def execute(self, goal_handle):
        self.goals.append(goal_handle.request.pose)
        goal_handle.succeed()
        return NavigateToPose.Result()

    def _on_labels(self, msg):
        self.labels.append(msg.data)

    def _tick_detector(self):
        if not self.labels or not self.labels[-1]:
            return
        det = Detection3D()
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = self.labels[-1]
        hyp.hypothesis.score = 0.9
        hyp.pose.pose.position.x = OBJECT_XY[0]
        hyp.pose.pose.position.y = OBJECT_XY[1]
        hyp.pose.pose.orientation.w = 1.0
        msg = Detection3DArray()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        det.header = msg.header
        det.results.append(hyp)
        msg.detections.append(det)
        self.detections_pub.publish(msg)


@pytest.fixture
def ros():
    # Nothing outside this test may reach its task_server and mock nodes: a high
    # DDS domain keeps a live robot/sim session on this machine out, and a
    # per-process namespace keeps sibling test sessions out — colcon runs
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


def test_fetch_by_label(ros, tmp_path):
    zones_file = tmp_path / "zones.yaml"
    zones_file.write_text(ZONES)
    server = TaskServer(
        parameter_overrides=[
            Parameter("zones_file", value=str(zones_file)),
            Parameter("pick_duration", value=0.1),
            Parameter("place_duration", value=0.1),
        ]
    )
    mock = MockWorld()
    executor = SingleThreadedExecutor()
    executor.add_node(server)
    executor.add_node(mock)

    assert spin_until(
        executor, lambda: mock.command_pub.get_subscription_count() > 0
    ), "task_server never subscribed to task/command"

    mock.command_pub.publish(String(data="fetch red_box dropoff"))
    assert spin_until(
        executor, lambda: any(s.startswith("succeeded") for s in mock.statuses)
    ), mock.statuses

    # The detector was asked for the label (underscores as spaces) and then idled.
    assert mock.labels[0] == "red box"
    assert mock.labels[-1] == ""

    # Robot at the origin, object at (2, 0): the first goal is the standoff
    # 0.4 m short of the object, facing it; the second is the drop zone.
    assert len(mock.goals) == 2, mock.goals
    standoff = mock.goals[0]
    assert standoff.header.frame_id == "map"
    assert standoff.pose.position.x == pytest.approx(1.6, abs=1e-6)
    assert standoff.pose.position.y == pytest.approx(0.0, abs=1e-6)
    yaw = 2.0 * math.atan2(standoff.pose.orientation.z, standoff.pose.orientation.w)
    assert yaw == pytest.approx(0.0, abs=1e-6)
    assert mock.goals[1].pose.position.x == pytest.approx(-1.0)

    server.destroy_node()
    mock.destroy_node()
