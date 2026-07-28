#!/usr/bin/env python3
"""Replay a mapping bag's odometry through the real `icp_odom_gate` node.

`odom_health.py` scores a bag; this produces the bag to score. It reads the two
streams a mapping bag already carries -- kinematic_icp's ``odom -> base`` and
the inverted wheel leaf -- feeds them to the *compiled* gate exactly as the
robot does, and records what the gate broadcasts. Running `odom_health.py` over
the result is then a before/after on the same data, through the same code that
runs on the robot rather than an offline model of it.

The gate is a separate process reached over ROS, so nothing here reimplements
the decision it makes. Time is simulated: the clock is stepped to each scan
stamp, so the TF interpolation the gate does for its wheel prior sees exactly
the timing the live system sees, at whatever wall-clock rate the replay runs at.

    pixi run -- python mote_bringup/tools/icp_gate_replay.py \\
        ~/.mote/bags/mapping/<run> /tmp/<run>-gated
    pixi run -- python mote_bringup/tools/odom_health.py /tmp/<run>-gated
"""

from __future__ import annotations

import argparse
import subprocess
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
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage

ICP_EDGE = ("odom", "base_footprint")
WHEEL_LEAF = ("base_footprint", "odom_wheel")
GATED_TOPIC = "/icp_odom"

# The gate needs a wheel sample at or after each scan stamp to interpolate its
# prior; the wheel stream runs at the controller's 50 Hz, so one period of
# lookahead is enough and any more would let it see the future.
WHEEL_LOOKAHEAD = 0.05


def robot_config() -> dict:
    with open(
        Path(get_package_share_directory("mote_description")) / "config" / "robot.yaml"
    ) as f:
        return yaml.safe_load(f)


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
            t = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
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


class Replay(Node):
    def __init__(self):
        super().__init__("icp_gate_replay")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.tf_pub = self.create_publisher(TFMessage, "/tf", tf_qos)
        self.odom_pub = self.create_publisher(
            Odometry,
            GATED_TOPIC,
            QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.create_subscription(TFMessage, "/tf", self.on_tf, tf_qos)
        self.gated: list[TransformStamped] = []

    def on_tf(self, msg: TFMessage):
        for tr in msg.transforms:
            if (tr.header.frame_id, tr.child_frame_id) == ICP_EDGE:
                self.gated.append(tr)

    def tick(self, t: float):
        c = Clock()
        c.clock.sec = int(t)
        c.clock.nanosec = int(round((t - int(t)) * 1e9))
        self.clock_pub.publish(c)


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
    rows = [(tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9, tr) for tr in gated]
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
    gate = subprocess.Popen(
        [
            "ros2",
            "run",
            "mote_nav",
            "icp_odom_gate",
            "--ros-args",
            "-r",
            f"odom_in:={GATED_TOPIC}",
            "-p",
            "use_sim_time:=true",
            "-p",
            f"max_wheel_speed:={float(robot['max_wheel_speed'])}",
            "-p",
            f"wheel_separation:={float(robot['wheel_separation'])}",
            "-p",
            f"tolerance:={args.tolerance}",
            "-p",
            "tf_timeout:=0.5",
        ]
    )

    rclpy.init()
    node = Replay()
    try:
        # Wait for the gate's subscription rather than guessing how long
        # `ros2 run` takes to come up: anything published before the match is
        # silently lost, and the run would then be scoring a shorter track.
        t0 = icp[0][0]
        deadline = time.monotonic() + 30.0
        while node.odom_pub.get_subscription_count() == 0:
            if time.monotonic() > deadline:
                raise SystemExit(
                    "the gate never subscribed; is mote_nav built and sourced?"
                )
            node.tick(t0)
            rclpy.spin_once(node, timeout_sec=0.05)
        # And let it receive a clock before the first scan, so its TF buffer is
        # not stamping arrivals against a zero time.
        for _ in range(20):
            node.tick(t0)
            rclpy.spin_once(node, timeout_sec=0.02)

        wi = 0
        for n, (t, tr) in enumerate(icp):
            node.tick(t)
            batch = []
            while wi < len(wheel) and wheel[wi][0] <= t + WHEEL_LOOKAHEAD:
                batch.append(wheel[wi][1])
                wi += 1
            if batch:
                m = TFMessage()
                m.transforms = batch
                node.tf_pub.publish(m)
            rclpy.spin_once(node, timeout_sec=0.0)
            node.odom_pub.publish(as_odometry(tr, "odom_icp"))
            end = time.monotonic() + args.pace
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.001)
            if n % 2000 == 0:
                print(f"  {n}/{len(icp)}  gated={len(node.gated)}")

        for _ in range(100):
            node.tick(icp[-1][0])
            rclpy.spin_once(node, timeout_sec=0.01)
    finally:
        gate.terminate()
        gate.wait(timeout=10)

    print(f"gate published {len(node.gated)} of {len(icp)} poses")
    if len(node.gated) < len(icp):
        raise SystemExit(
            f"the gate returned {len(icp) - len(node.gated)} fewer poses than it was "
            "given -- messages were dropped, so the output is not comparable; "
            "re-run with a larger --pace"
        )

    write_bag(args.out, node.gated, wheel)
    node.destroy_node()
    rclpy.shutdown()
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
