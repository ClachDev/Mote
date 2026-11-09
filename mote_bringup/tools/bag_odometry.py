"""Read both odometry sources out of a recorded bag.

Shared by ``odom_health.py`` (per-interval scoring of a whole session) and
``slip_replay.py`` (the live detector, replayed). Both need exactly the same
extraction, and it is fiddly enough — an inverted TF leaf, two possible sources
for the wheel pose — to be worth having once.

The two sources are in a mapping bag's ``/tf``:

    base_footprint -> odom_icp    kinematic_icp's scan-matched pose, inverted (~10 Hz)
    odom -> base_footprint        the same, before the gate owned this edge
    base_footprint -> odom_wheel  the inverted wheel-odom leaf (~50-100 Hz)

Since the gate (2026-07-28) ``odom -> base_footprint`` is the gated edge, and
since 2026-11-04 it is published at the wheel rate, carried between scans on the
wheels; scoring it as the lidar would compare the wheels with themselves. So the
``odom_icp`` leaf is read when the bag has one, and the edge only when it does
not (bags recorded before the gate).

``mote_nav::OdomTfRelay`` writes that leaf straight from
``/diff_drive_controller/odom`` with the stamp and pose unchanged, so inverting
it recovers the very numbers the topic carried. The topic is preferred when the
bag has it (the ``lite`` stream records it; ``mapping`` does not).
"""

from __future__ import annotations

import math
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message

from mote_bringup.odom_residual import yaw_of_quat

ICP_EDGE = ("odom", "base_footprint")
ICP_LEAF = ("base_footprint", "odom_icp")
WHEEL_LEAF = ("base_footprint", "odom_wheel")
ODOM_TOPIC = "/diff_drive_controller/odom"
CMD_TOPIC = "/diff_drive_controller/cmd_vel"


def _stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def _uninvert(t, p, q):
    """``(t, x, y, yaw)`` of the pose an inverted leaf was made from."""
    a = yaw_of_quat(q.x, q.y, q.z, q.w)
    c, s = math.cos(-a), math.sin(-a)
    return (t, -(c * p.x - s * p.y), -(s * p.x + c * p.y), -a)


def read_samples(bag: Path):
    """``(wheel, icp, command)`` sample lists, each sorted by stamp.

    ``wheel`` and ``icp`` are ``(t, x, y, yaw)``; ``command`` is
    ``(t, linear.x, angular.z)`` and is empty unless the bag recorded cmd_vel.
    """
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    available = {t.name for t in reader.get_all_topics_and_types()}
    reader.set_filter(
        rosbag2_py.StorageFilter(
            topics=[t for t in ("/tf", ODOM_TOPIC, CMD_TOPIC) if t in available]
        )
    )

    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
    from tf2_msgs.msg import TFMessage

    wheel, icp, icp_leaf, command = [], [], [], []
    have_odom_topic = ODOM_TOPIC in available
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == "/tf":
            for tr in deserialize_message(data, TFMessage).transforms:
                t = _stamp(tr.header)
                p, q = tr.transform.translation, tr.transform.rotation
                pair = (tr.header.frame_id, tr.child_frame_id)
                if pair == ICP_EDGE:
                    icp.append((t, p.x, p.y, yaw_of_quat(q.x, q.y, q.z, q.w)))
                elif pair == ICP_LEAF:
                    icp_leaf.append(_uninvert(t, p, q))
                elif pair == WHEEL_LEAF and not have_odom_topic:
                    wheel.append(_uninvert(t, p, q))
        elif topic == ODOM_TOPIC:
            msg = deserialize_message(data, Odometry)
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            wheel.append(
                (_stamp(msg.header), p.x, p.y, yaw_of_quat(q.x, q.y, q.z, q.w))
            )
        elif topic == CMD_TOPIC:
            msg = deserialize_message(data, TwistStamped)
            command.append(
                (_stamp(msg.header), msg.twist.linear.x, msg.twist.angular.z)
            )

    if icp_leaf:
        icp = icp_leaf
    for series in (wheel, icp, command):
        series.sort(key=lambda s: s[0])
    return wheel, icp, command
