#!/usr/bin/env python3
"""Replay a mapping bag's odometry through the real `icp_odom_gate` node.

`odom_health.py` scores a bag; this produces the bag to score. It reads the two
streams a mapping bag already carries -- kinematic_icp's ``odom -> base`` and
the inverted wheel leaf -- feeds them to the *compiled* gate exactly as the
robot does, and records what the gate produces. Running `odom_health.py` over
the result is then a before/after on the same data, through the same code that
runs on the robot rather than an offline model of it.

The gate is a separate process reached over ROS, so nothing here reimplements
the decision it makes. Wheel poses go to it as odometry messages, as
``/diff_drive_controller/odom`` carries them, and each scan's ICP pose follows
the wheel poses up to one controller period past its stamp: on the robot the
scan match answers after the wheels have moved on.

Two outputs:

* ``<out>``: the gated pose at each scan stamp (the gate's ``~/odometry``) as
  ``odom -> base_footprint``, plus the wheel leaf. This is the track the gate
  accumulates, at the scan rate, so `odom_health.py` scores it exactly as it
  scored the gate's output before odom->base was published at wheel rate.
* ``<out>-steps.csv``: the wheel-rate broadcasts scored by `odom_steps.py`, the
  step each scan's re-anchoring puts into odom->base. Its summary is printed.

The gate's log, with one line per rejected increment, is ``<out>-gate.log``.

    pixi run -- python mote_bringup/tools/icp_gate_replay.py \\
        ~/.mote/bags/mapping/<run> /tmp/<run>-gated
    pixi run -- python mote_bringup/tools/odom_health.py /tmp/<run>-gated
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import rclpy
import rosbag2_py
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message, serialize_message
from tf2_msgs.msg import TFMessage

sys.path.insert(0, str(Path(__file__).resolve().parent))

from mote_bringup.odom_residual import yaw_of_quat  # noqa: E402
from mote_bringup.sweep_orphans import reap_group, spawn_reapable  # noqa: E402
from odom_steps import residuals, summarise, write_csv  # noqa: E402

ICP_EDGE = ("odom", "base_footprint")
WHEEL_LEAF = ("base_footprint", "odom_wheel")
ICP_TOPIC = "/icp_odom"
WHEEL_TOPIC = "/replay/wheel_odom"
ANCHOR_TOPIC = "/icp_odom_gate/odometry"

# How far the wheel stream runs ahead of each scan when its ICP pose arrives:
# one controller period, the gate's lag on the robot at the least. Rejection
# needs only a wheel sample at or after the scan stamp.
WHEEL_LOOKAHEAD = 0.05


def robot_config() -> dict:
    with open(
        Path(get_package_share_directory("mote_description")) / "config" / "robot.yaml"
    ) as f:
        return yaml.safe_load(f)


def _t(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def read_source(bag: Path):
    """The ICP poses, and the wheel-leaf transforms verbatim for re-recording."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/tf"]))
    icp, wheel = [], []
    while reader.has_next():
        _, data, _ = reader.read_next()
        for tr in deserialize_message(data, TFMessage).transforms:
            pair = (tr.header.frame_id, tr.child_frame_id)
            t = _t(tr.header.stamp)
            if pair == ICP_EDGE:
                icp.append((t, tr))
            elif pair == WHEEL_LEAF:
                wheel.append((t, tr))
    icp.sort(key=lambda r: r[0])
    wheel.sort(key=lambda r: r[0])
    return icp, wheel


def as_odometry(tr: TransformStamped, frame: str) -> Odometry:
    """kinematic_icp's odometry message, rebuilt from the transform it recorded.

    The bag holds the broadcast rather than the topic, and the two carry the
    same pose at the same stamp, so this is a re-framing and not a conversion.
    """
    msg = Odometry()
    msg.header.stamp = tr.header.stamp
    msg.header.frame_id = frame
    msg.child_frame_id = tr.child_frame_id
    msg.pose.pose.position.x = tr.transform.translation.x
    msg.pose.pose.position.y = tr.transform.translation.y
    msg.pose.pose.position.z = tr.transform.translation.z
    msg.pose.pose.orientation = tr.transform.rotation
    return msg


def wheel_pose(leaf: TransformStamped) -> tuple[float, float, float]:
    """The wheel pose the leaf was inverted from (`OdomTfRelay` undone)."""
    p, q = leaf.transform.translation, leaf.transform.rotation
    a = yaw_of_quat(q.x, q.y, q.z, q.w)
    c, s = math.cos(-a), math.sin(-a)
    return -(c * p.x - s * p.y), -(s * p.x + c * p.y), -a


def wheel_odometry(leaf: TransformStamped) -> Odometry:
    """``/diff_drive_controller/odom`` as it was before the relay inverted it."""
    x, y, yaw = wheel_pose(leaf)
    msg = Odometry()
    msg.header.stamp = leaf.header.stamp
    msg.header.frame_id = "odom"
    msg.child_frame_id = leaf.header.frame_id
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.pose.orientation.w = math.cos(yaw / 2)
    return msg


def as_transform(msg: Odometry) -> TransformStamped:
    tr = TransformStamped()
    tr.header.stamp = msg.header.stamp
    tr.header.frame_id = msg.header.frame_id
    tr.child_frame_id = msg.child_frame_id
    tr.transform.translation.x = msg.pose.pose.position.x
    tr.transform.translation.y = msg.pose.pose.position.y
    tr.transform.translation.z = msg.pose.pose.position.z
    tr.transform.rotation = msg.pose.pose.orientation
    return tr


class Replay(Node):
    def __init__(self):
        super().__init__("icp_gate_replay")
        tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self.icp_pub = self.create_publisher(Odometry, ICP_TOPIC, reliable)
        self.wheel_pub = self.create_publisher(Odometry, WHEEL_TOPIC, reliable)
        self.create_subscription(TFMessage, "/tf", self.on_tf, tf_qos)
        self.create_subscription(Odometry, ANCHOR_TOPIC, self.on_anchor, reliable)
        self.broadcast: list[TransformStamped] = []
        self.anchors: list[Odometry] = []

    def on_tf(self, msg: TFMessage):
        for tr in msg.transforms:
            if (tr.header.frame_id, tr.child_frame_id) == ICP_EDGE:
                self.broadcast.append(tr)

    def on_anchor(self, msg: Odometry):
        self.anchors.append(msg)

    def settle(self, seconds: float):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.001)


def write_bag(out: Path, gated, wheel):
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(out), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            id=0, name="/tf", type="tf2_msgs/msg/TFMessage", serialization_format="cdr"
        )
    )
    rows = [(_t(tr.header.stamp), tr) for tr in gated]
    rows += [(t, tr) for t, tr in wheel]
    rows.sort(key=lambda r: r[0])
    for t, tr in rows:
        msg = TFMessage()
        msg.transforms = [tr]
        writer.write("/tf", serialize_message(msg), int(t * 1e9))
    del writer


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("bag", type=Path)
    ap.add_argument("out", type=Path)
    ap.add_argument("--tolerance", type=float, default=1.15)
    ap.add_argument(
        "--pace",
        type=float,
        default=0.004,
        help="seconds between scans; the gate must keep up, and the run asserts it did",
    )
    args = ap.parse_args()

    icp, wheel = read_source(args.bag)
    print(f"{args.bag.name}: {len(icp)} icp poses, {len(wheel)} wheel transforms")
    if not icp or not wheel:
        raise SystemExit("bag carries neither edge; nothing to replay")

    robot = robot_config()
    log_path = Path(f"{args.out}-gate.log")
    log = open(log_path, "w")
    gate = spawn_reapable(
        [
            "ros2",
            "run",
            "mote_nav",
            "icp_odom_gate",
            "--ros-args",
            "-r",
            f"odom_in:={ICP_TOPIC}",
            "-r",
            f"wheel_odom_in:={WHEEL_TOPIC}",
            "-p",
            f"max_wheel_speed:={float(robot['max_wheel_speed'])}",
            "-p",
            f"wheel_separation:={float(robot['wheel_separation'])}",
            "-p",
            f"tolerance:={args.tolerance}",
        ],
        stdout=log,
        stderr=log,
    )

    rclpy.init()
    node = Replay()
    try:
        # Wait for the gate's subscriptions rather than guessing how long
        # `ros2 run` takes to come up: anything published before the match is
        # silently lost, and the run would then be scoring a shorter track.
        deadline = time.monotonic() + 30.0
        while (
            node.icp_pub.get_subscription_count() == 0
            or node.wheel_pub.get_subscription_count() == 0
            or node.count_publishers(ANCHOR_TOPIC) == 0
        ):
            if time.monotonic() > deadline:
                raise SystemExit(
                    "the gate never subscribed; is mote_nav built and sourced?"
                )
            rclpy.spin_once(node, timeout_sec=0.05)
        node.settle(0.5)

        wi = 0
        for n, (t, tr) in enumerate(icp):
            sent = False
            while wi < len(wheel) and wheel[wi][0] <= t + WHEEL_LOOKAHEAD:
                node.wheel_pub.publish(wheel_odometry(wheel[wi][1]))
                wi += 1
                sent = True
            if sent:
                # The two topics carry no ordering between them; give the
                # wheel poses time to land before the scan that follows them.
                node.settle(args.pace / 2)
            node.icp_pub.publish(as_odometry(tr, "odom_icp"))
            node.settle(args.pace)
            if n % 2000 == 0:
                print(f"  {n}/{len(icp)}  anchors={len(node.anchors)}")

        node.settle(1.0)
    finally:
        reap_group(gate)
        log.close()

    print(f"gate answered {len(node.anchors)} of {len(icp)} scans")
    if len(node.anchors) < len(icp):
        raise SystemExit(
            f"the gate answered {len(icp) - len(node.anchors)} fewer scans than it "
            "was given -- messages were dropped, so the output is not comparable; "
            "re-run with a larger --pace"
        )
    rejected = sum("rejected icp increment" in line for line in open(log_path))
    print(f"gate rejected {rejected} increments (log: {log_path})")

    write_bag(args.out, [as_transform(m) for m in node.anchors], wheel)
    print(f"wrote {args.out}")

    poses = [
        (
            _t(tr.header.stamp),
            tr.transform.translation.x,
            tr.transform.translation.y,
            yaw_of_quat(
                tr.transform.rotation.x,
                tr.transform.rotation.y,
                tr.transform.rotation.z,
                tr.transform.rotation.w,
            ),
        )
        for tr in sorted(node.broadcast, key=lambda tr: _t(tr.header.stamp))
    ]
    wheel_poses = [(t, *wheel_pose(tr)) for t, tr in wheel]
    rows = residuals(poses, wheel_poses)
    summarise(rows, "odom->base broadcasts: ")
    steps_path = Path(f"{args.out}-steps.csv")
    write_csv(steps_path, rows)
    print(f"wrote {steps_path}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
