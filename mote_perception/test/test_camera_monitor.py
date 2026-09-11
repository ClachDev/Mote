"""camera_monitor against a stand-in camera, over a real graph.

The frames and camera_info go through DDS rather than into the callbacks, so the
one-shot read is exercised the way it runs: a subscription destroyed from inside
its own callback. Reports are called directly rather than waited for.

A random ROS_DOMAIN_ID and a per-process namespace keep this off a live robot's
graph and off sibling test sessions.
"""

import os
import random
import time

os.environ["ROS_DOMAIN_ID"] = str(random.randint(60, 100))

import pytest  # noqa: E402
import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import CameraInfo, CompressedImage  # noqa: E402

from mote_perception.camera_monitor import CameraMonitor  # noqa: E402

FORMAT = "rgb8; jpeg compressed bgr8"


class Log:
    def __init__(self):
        self.lines = []

    def info(self, msg):
        self.lines.append(("info", msg))

    def warn(self, msg):
        self.lines.append(("warn", msg))


@pytest.fixture
def graph():
    rclpy.init(args=["--ros-args", "-r", f"__ns:=/test_{os.getpid()}"])
    monitor = CameraMonitor()
    log = Log()
    monitor.get_logger = lambda: log
    camera = Node("camera")
    frames = camera.create_publisher(CompressedImage, "image/compressed", 10)
    info = camera.create_publisher(CameraInfo, "camera_info", 10)
    executor = SingleThreadedExecutor()
    executor.add_node(monitor)
    executor.add_node(camera)

    def spin_until(cond, timeout=10.0, each=None):
        deadline = time.monotonic() + timeout
        while not cond():
            assert time.monotonic() < deadline, "timed out"
            if each:
                each()
            executor.spin_once(timeout_sec=0.05)

    def publish(n=1, size=(640, 480), with_info=True):
        # Discovery is asynchronous; a message sent before the monitor matches is lost.
        spin_until(lambda: frames.get_subscription_count() > 0)
        if with_info:
            spin_until(lambda: info.get_subscription_count() > 0)
        for _ in range(n):
            frames.publish(CompressedImage(format=FORMAT, data=bytes(64)))
            if with_info:
                info.publish(CameraInfo(width=size[0], height=size[1]))

    yield monitor, log, publish, spin_until
    executor.shutdown()
    camera.destroy_node()
    monitor.destroy_node()
    rclpy.shutdown()


def test_reports_rate_resolution_and_format(graph):
    monitor, log, publish, spin_until = graph
    publish(3)
    spin_until(lambda: monitor.count == 3 and monitor.resolution)
    monitor._report()
    level, line = log.lines[-1]
    assert level == "info"
    assert f"640x480, format={FORMAT}" in line
    assert line.startswith(f"{3 / monitor.report_period:.1f} FPS")


def test_camera_info_is_read_once(graph):
    monitor, _, publish, spin_until = graph
    publish(1)
    spin_until(lambda: monitor.resolution == (640, 480))
    assert monitor._info_sub is None
    assert all(
        sub.topic_name.endswith("image/compressed") for sub in monitor.subscriptions
    )


def test_dropout_rearms_the_read(graph):
    monitor, log, publish, spin_until = graph
    publish(1)
    spin_until(lambda: monitor.resolution == (640, 480))
    monitor._report()

    monitor._report()
    assert log.lines[-1][0] == "warn"
    assert monitor.resolution is None
    assert monitor._info_sub is not None

    # The re-armed subscription is new to the camera, and the camera may still be
    # matched to the one it replaced, so keep publishing until one lands.
    spin_until(
        lambda: monitor.resolution == (1280, 720),
        each=lambda: publish(1, size=(1280, 720)),
    )
    monitor._report()
    assert "1280x720" in log.lines[-1][1]


def test_frames_without_camera_info(graph):
    monitor, log, publish, spin_until = graph
    publish(2, with_info=False)
    spin_until(lambda: monitor.count == 2)
    monitor._report()
    level, line = log.lines[-1]
    assert level == "info"
    assert "no camera_info" in line
