"""The slip monitor as a running node: real subscriptions, real TF, real output.

``test_odom_residual.py`` covers every decision the detector makes. What it
cannot cover is the plumbing around them, which is where the silent failures
live: a wrong topic name, a TF frame that never resolves, a stamp read from the
wrong clock. All three would leave a node that runs, publishes a cheerful
``unknown``, and never reports anything — so this drives the real node with two
odometry sources and asserts on what comes out of ``/diagnostics``.

The robot is synthesised rather than replayed from a bag because bags live in
``$MOTE_HOME``, outside the repo, and a test that skips when they are absent
would never run in CI. The distributions those bags produced are what set the
thresholds; that calibration is asserted in ``test_odom_residual.py`` and
recorded in ``docs/tuning/2026-07-28-slip-detection.md``.
"""

import math
import os
import random

# A stray ROS_DOMAIN_ID here would put this test on the same graph as a real
# robot, whose wheels take /diff_drive_controller commands. Claim an unused
# domain and stay on localhost before rclpy is imported.
os.environ["ROS_DOMAIN_ID"] = str(random.randint(64, 200))
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"

import pytest  # noqa: E402
import rclpy  # noqa: E402
from diagnostic_msgs.msg import DiagnosticArray  # noqa: E402
from geometry_msgs.msg import TransformStamped, TwistStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402

import tf2_ros  # noqa: E402

from mote_bringup.slip_monitor import STATUS_NAME, SlipMonitor  # noqa: E402

ODOM_RATE = 50.0
ICP_RATE = 10.0


class _Robot(Node):
    """Publishes both odometry sources for a robot travelling in a straight line.

    ``wheel_speed`` is what the encoders report and ``icp_speed`` what the scan
    match sees; making them differ is the whole experiment.
    """

    def __init__(self, wheel_speed, icp_speed, command=None):
        super().__init__("fake_robot")
        self.wheel_speed = wheel_speed
        self.icp_speed = icp_speed
        self.command = command
        self.odom_pub = self.create_publisher(
            Odometry, "/diff_drive_controller/odom", 20
        )
        self.cmd_pub = self.create_publisher(
            TwistStamped, "/diff_drive_controller/cmd_vel", 10
        )
        self.tf = tf2_ros.TransformBroadcaster(self)
        self.start = self._now()
        self.create_timer(1.0 / ODOM_RATE, self._publish_wheel)
        self.create_timer(1.0 / ICP_RATE, self._publish_icp)

    def _now(self):
        return self.get_clock().now()

    def _elapsed(self, stamp):
        return (stamp - self.start).nanoseconds / 1e9

    def _publish_wheel(self):
        now = self._now()
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_footprint"
        msg.pose.pose.position.x = self.wheel_speed * self._elapsed(now)
        msg.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(msg)

        if self.command is not None:
            cmd = TwistStamped()
            cmd.header.stamp = now.to_msg()
            cmd.twist.linear.x = self.command[0]
            cmd.twist.angular.z = self.command[1]
            self.cmd_pub.publish(cmd)

    def _publish_icp(self):
        now = self._now()
        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_footprint"
        tf.transform.translation.x = self.icp_speed * self._elapsed(now)
        tf.transform.rotation.w = 1.0
        self.tf.sendTransform(tf)


class _Listener(Node):
    """Collects the slip status off the shared /diagnostics topic."""

    def __init__(self):
        super().__init__("diagnostics_listener")
        self.statuses = []
        self.create_subscription(DiagnosticArray, "diagnostics", self._on_msg, 20)

    def _on_msg(self, msg):
        for status in msg.status:
            if status.name == STATUS_NAME:
                self.statuses.append(status)

    def states(self):
        return [
            dict((kv.key, kv.value) for kv in s.values).get("state")
            for s in self.statuses
        ]


def _run(wheel_speed, icp_speed, command=None, seconds=4.0):
    """Spin the real monitor against a synthetic robot; return its statuses."""
    rclpy.init()
    monitor = SlipMonitor()
    robot = _Robot(wheel_speed, icp_speed, command)
    listener = _Listener()
    executor = SingleThreadedExecutor()
    for node in (monitor, robot, listener):
        executor.add_node(node)
    try:
        deadline = monitor.get_clock().now().nanoseconds / 1e9 + seconds
        while monitor.get_clock().now().nanoseconds / 1e9 < deadline:
            executor.spin_once(timeout_sec=0.05)
    finally:
        for node in (monitor, robot, listener):
            node.destroy_node()
        rclpy.try_shutdown()
    return listener.statuses, listener.states()


@pytest.fixture(autouse=True)
def _isolated_home(tmp_path, monkeypatch):
    """No per-robot slip.yaml, so the packaged thresholds are what is tested."""
    monkeypatch.setenv("MOTE_HOME", str(tmp_path))


def test_agreeing_sources_stay_quiet():
    statuses, states = _run(wheel_speed=0.20, icp_speed=0.199)
    assert statuses, "the monitor published no slip status at all"
    assert all(s.level == s.OK for s in statuses)
    assert "slip" not in states


def test_wheels_over_reading_is_reported_as_slip():
    """The acceptance case: wheels claim 0.20 m/s, the lidar sees 0.08 m/s."""
    statuses, states = _run(wheel_speed=0.20, icp_speed=0.08)
    assert "slip" in states, f"never reported slip; saw {sorted(set(states))}"
    slipping = [s for s in statuses if s.message.startswith("wheels report")]
    assert slipping
    assert slipping[-1].level == slipping[-1].WARN
    values = dict((kv.key, kv.value) for kv in slipping[-1].values)
    assert float(values["speed_residual"]) == pytest.approx(0.12, abs=0.02)
    assert float(values["relative"]) == pytest.approx(0.6, abs=0.1)


def test_commanded_but_motionless_is_reported_as_stuck():
    _, states = _run(wheel_speed=0.0, icp_speed=0.0, command=(0.2, 0.0))
    assert "stuck" in states, f"never reported stuck; saw {sorted(set(states))}"


def test_parked_without_a_command_is_not_stuck():
    """Nothing commanded means parked, and a parked robot is not a fault."""
    _, states = _run(wheel_speed=0.0, icp_speed=0.0, command=None)
    assert "stuck" not in states


def test_lidar_over_reading_is_not_blamed_on_the_wheels():
    statuses, states = _run(wheel_speed=0.08, icp_speed=0.20)
    assert "icp_fault" in states, f"saw {sorted(set(states))}"
    assert "slip" not in states


def test_residual_is_published_for_logging():
    """The raw residual has to be recordable, not just summarised in a message."""
    rclpy.init()
    monitor = SlipMonitor()
    robot = _Robot(0.20, 0.10)
    received = []

    listener = rclpy.create_node("residual_listener")
    listener.create_subscription(
        TwistStamped, "slip/residual", lambda m: received.append(m), 20
    )
    executor = SingleThreadedExecutor()
    for node in (monitor, robot, listener):
        executor.add_node(node)
    try:
        deadline = monitor.get_clock().now().nanoseconds / 1e9 + 3.0
        while monitor.get_clock().now().nanoseconds / 1e9 < deadline:
            executor.spin_once(timeout_sec=0.05)
    finally:
        for node in (monitor, robot, listener):
            node.destroy_node()
        rclpy.try_shutdown()

    assert received
    last = received[-1]
    assert last.twist.linear.x == pytest.approx(0.10, abs=0.02)
    assert not math.isnan(last.twist.angular.z)
