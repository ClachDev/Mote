"""The health monitor as a running node: raw subscriptions still report.

``test_health_monitor.py`` covers the roll-up decisions as plain function calls.
What it cannot cover is the delivery underneath them. The watched topics are
subscribed raw — the callback only counts arrivals, so nothing is gained by
building a Python message first — and ``raw=True`` is exactly the kind of change
that fails silently: a subscription that delivers nothing leaves a node which
still runs, still publishes on time, and reports every subsystem as missing. So
this drives the real node with a real publisher and asserts both halves: that
the topics are subscribed raw, and that a raw arrival still reaches the summary.
"""

import os
import random

# A stray ROS_DOMAIN_ID here would put this test on the same graph as a real
# robot. Claim an unused domain and stay on localhost before rclpy is imported.
os.environ["ROS_DOMAIN_ID"] = str(random.randint(64, 200))
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"

import pytest  # noqa: E402
import rclpy  # noqa: E402
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from mote_bringup.health_monitor import HealthMonitor  # noqa: E402

CONFIG = """\
period: 0.2
topics:
  - name: scan
    topic: /scan
    type: sensor_msgs/msg/LaserScan
    min_rate: 5.0
    timeout: 2.0
    severity: critical
tf: []
subscribe_diagnostics: true
"""

SCAN_RATE = 20.0


class _Lidar(Node):
    """Publishes scans at a rate comfortably above the configured floor."""

    def __init__(self):
        super().__init__("fake_lidar")
        self.published = 0
        self.pub = self.create_publisher(LaserScan, "/scan", 10)
        self.create_timer(1.0 / SCAN_RATE, self._publish)

    def _publish(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser"
        msg.ranges = [1.0] * 360
        self.pub.publish(msg)
        self.published += 1


class _Listener(Node):
    def __init__(self):
        super().__init__("health_listener")
        self.summaries = []
        self.aggregates = []
        self.create_subscription(String, "health", self._on_health, 10)
        self.create_subscription(
            DiagnosticArray, "diagnostics_agg", self.aggregates.append, 10
        )

    def _on_health(self, msg):
        self.summaries.append(msg.data)


@pytest.fixture
def monitor(tmp_path, monkeypatch):
    monkeypatch.setenv("MOTE_HOME", str(tmp_path))
    (tmp_path / "health.yaml").write_text(CONFIG)
    rclpy.init()
    node = HealthMonitor()
    yield node
    node.destroy_node()
    rclpy.try_shutdown()


def _spin(nodes, seconds):
    executor = SingleThreadedExecutor()
    for node in nodes:
        executor.add_node(node)
    deadline = nodes[0].get_clock().now().nanoseconds / 1e9 + seconds
    while nodes[0].get_clock().now().nanoseconds / 1e9 < deadline:
        executor.spin_once(timeout_sec=0.02)


def _subscriptions(node):
    return {sub.topic_name.lstrip("/"): sub for sub in node.subscriptions}


def test_watched_topics_are_subscribed_raw(monitor):
    """Freshness watches take bytes; the roll-up's own input does not.

    The /diagnostics subscription reads status.name and status.level, so it is
    the one that must stay deserialized — subscribing it raw would hand the
    callback bytes it would index as a message, and the forwarded statuses
    would vanish from the summary rather than raise.
    """
    subs = _subscriptions(monitor)
    assert subs["scan"].raw is True
    assert subs["diagnostics"].raw is False


def test_a_raw_scan_still_reaches_the_summary(monitor):
    """A counted arrival is worth nothing if the count never moves."""
    lidar = _Lidar()
    listener = _Listener()
    try:
        _spin([monitor, lidar, listener], seconds=2.0)
    finally:
        lidar.destroy_node()
        listener.destroy_node()

    assert lidar.published > 10, "the publisher itself never ran"
    watch = monitor.topics[0]
    assert watch.count or watch.last_stamp, "no raw message ever arrived"

    assert listener.summaries, "nothing published on /health"
    assert listener.summaries[-1] == "OK", listener.summaries
    # The aggregate keeps its shape: the mote roll-up first, then one status per
    # subsystem, and the measured rate is a real one rather than zero.
    last = listener.aggregates[-1]
    assert [s.name for s in last.status] == ["mote", "scan"]
    assert last.status[0].level == DiagnosticStatus.OK
    values = {kv.key: kv.value for kv in last.status[1].values}
    assert float(values["rate_hz"]) >= 5.0, values
