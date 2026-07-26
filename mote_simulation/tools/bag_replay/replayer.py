#!/usr/bin/env python3
"""Replay one recorded mapping bag into a live SLAM/ICP node and capture outputs.

Assumes the stack under test is already running in the same (isolated) ROS graph
— ``replay.py`` owns launching ``async_slam_toolbox_node`` (slam mode) or
``kinematic_icp_online_node`` (icp mode) and tearing it down. This process is the
ROS client for a single parameter set: it streams the bag's ``/scan_filtered`` +
``/tf`` + ``/tf_static`` back onto the graph in sim-time order (driving ``/clock``
itself, so the node runs on bag time regardless of wall speed), records the
estimator's output trajectory, and grabs the finished ``/map``.

The one subtlety is TF ownership. A mapping bag's ``/tf`` already contains the
edges the *original* run produced — ``map->odom`` from slam_toolbox and
``odom->base_footprint`` from kinematic_icp. Replaying those verbatim would fight
the fresh node publishing the same edge, so the edge the node-under-test owns is
stripped from the replayed ``/tf`` (see ``STRIP``); everything else is passed
through. In slam mode the recorded ``odom->base`` is kept and *fed* to slam as
its odometry prior, exactly as on the robot.

Outputs (in ``--out-dir``): ``series.json`` (raw re-scorable trajectory) and, in
slam mode, ``map.npz`` (occupancy grid + resolution/origin for rendering and
map-quality metrics). Run one replay per process for a clean rclpy context.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

import rosbag2_py

# The TF edge the node under test publishes itself — stripped from the replayed
# /tf so the recorded copy can't fight the fresh one. (parent, child) pairs.
STRIP = {
    "slam": {("map", "odom")},
    "icp": {("map", "odom"), ("odom", "base_footprint")},
}

# Which output edge to sample as the estimator's trajectory, per mode.
OUTPUT_EDGE = {
    "slam": ("map", "base_link"),
    "icp": ("odom", "base_footprint"),
}

BAG_TOPICS = {
    "/tf": TFMessage,
    "/tf_static": TFMessage,
    "/scan_filtered": LaserScan,
}


def yaw_of(q):
    import math

    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Replayer(Node):
    def __init__(self, mode, sample_dt):
        super().__init__("bag_replayer")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.mode = mode
        self.strip = STRIP[mode]
        self.out_parent, self.out_child = OUTPUT_EDGE[mode]
        self.sample_dt = sample_dt

        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.tf_pub = self.create_publisher(TFMessage, "/tf", 100)
        static_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.static_pub = self.create_publisher(TFMessage, "/tf_static", static_qos)
        self.scan_pub = self.create_publisher(LaserScan, "/scan_filtered", 10)

        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)
        self.latest_map = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.traj = []  # [sim_t, x, y, yaw]
        self._static_frames = {}  # (parent, child) -> TransformStamped
        self._next_sample = None

    def on_map(self, msg):
        self.latest_map = msg

    def publish_clock(self, t_ns):
        c = Clock()
        c.clock.sec = t_ns // 1_000_000_000
        c.clock.nanosec = t_ns % 1_000_000_000
        self.clock_pub.publish(c)

    def handle_tf_static(self, msg):
        # Accumulate every static edge ever seen and republish the union latched,
        # so a subscriber that joins late still gets the full static tree.
        changed = False
        for tr in msg.transforms:
            key = (tr.header.frame_id, tr.child_frame_id)
            if key not in self._static_frames:
                self._static_frames[key] = tr
                changed = True
        if changed:
            out = TFMessage()
            out.transforms = list(self._static_frames.values())
            self.static_pub.publish(out)

    def handle_tf(self, msg):
        kept = [
            tr
            for tr in msg.transforms
            if (tr.header.frame_id, tr.child_frame_id) not in self.strip
        ]
        if kept:
            out = TFMessage()
            out.transforms = kept
            self.tf_pub.publish(out)

    def sample_output(self, sim_t):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.out_parent, self.out_child, rclpy.time.Time()
            )
        except tf2_ros.TransformException:
            return
        tr = tf.transform.translation
        self.traj.append([sim_t, tr.x, tr.y, yaw_of(tf.transform.rotation)])


def read_bag(bag):
    r = rosbag2_py.SequentialReader()
    r.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    while r.has_next():
        topic, data, t = r.read_next()
        if topic in BAG_TOPICS:
            yield topic, deserialize_message(data, BAG_TOPICS[topic]), t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--mode", choices=("slam", "icp"), default="slam")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--rate", type=float, default=1.0, help="replay speed x realtime")
    ap.add_argument(
        "--sample-dt", type=float, default=0.1, help="traj sample period (s)"
    )
    ap.add_argument(
        "--settle", type=float, default=8.0, help="extra sim s after last scan"
    )
    ap.add_argument(
        "--max-scans", type=int, default=0, help="0 = whole bag (debug cap)"
    )
    args = ap.parse_args()

    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = Replayer(args.mode, args.sample_dt)

    # Give the node-under-test a moment to come up on the graph and discover us.
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.1)

    wall0 = None
    t0_ns = None
    last_ns = None
    last_sample = -1e18  # sim-time of the last trajectory sample (gap-robust)
    n_scans = 0
    stop = False

    for topic, msg, t_ns in read_bag(args.bag):
        if t0_ns is None:
            t0_ns = t_ns
            wall0 = time.monotonic()
        sim_t = (t_ns - t0_ns) / 1e9
        last_ns = t_ns

        # Pace to the requested fraction of realtime, spinning so /map and TF
        # callbacks fire while we wait.
        target = wall0 + sim_t / args.rate
        while True:
            dt = target - time.monotonic()
            if dt <= 0:
                break
            rclpy.spin_once(node, timeout_sec=min(dt, 0.02))

        node.publish_clock(t_ns)
        if topic == "/tf_static":
            node.handle_tf_static(msg)
        elif topic == "/tf":
            node.handle_tf(msg)
        elif topic == "/scan_filtered":
            node.scan_pub.publish(msg)
            n_scans += 1
        rclpy.spin_once(node, timeout_sec=0.0)

        if sim_t - last_sample >= args.sample_dt:
            node.sample_output(sim_t)
            last_sample = sim_t

        if args.max_scans and n_scans >= args.max_scans:
            stop = True
            break

    if t0_ns is None:
        print("FAIL: bag had no replayable topics", file=sys.stderr)
        (out / "error.txt").write_text("bag had no /scan_filtered /tf /tf_static\n")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    # Keep advancing sim time (no new scans) so slam fires a final map update and
    # the last poses flush through TF. Sim time is advanced strictly in step with
    # wall time * rate — spin_once returns instantly while slam publishes, so the
    # loop must be paced by the wall clock, not by iteration count, or sim time
    # would run away.
    settle_start = time.monotonic()
    settle_wall = args.settle / args.rate
    while True:
        elapsed = time.monotonic() - settle_start
        if elapsed >= settle_wall:
            break
        cur_ns = last_ns + int(elapsed * args.rate * 1e9)
        node.publish_clock(cur_ns)
        rclpy.spin_once(node, timeout_sec=0.02)
        sim_t = (cur_ns - t0_ns) / 1e9
        if sim_t - last_sample >= args.sample_dt:
            node.sample_output(sim_t)
            last_sample = sim_t
        time.sleep(0.01)

    result = {
        "mode": args.mode,
        "bag": str(args.bag),
        "truncated": bool(stop),
        "n_scans": n_scans,
        "traj": node.traj,
    }
    if node.latest_map is not None:
        m = node.latest_map
        grid = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        np.savez_compressed(
            out / "map.npz",
            grid=grid,
            resolution=np.float64(m.info.resolution),
            origin=np.array(
                [m.info.origin.position.x, m.info.origin.position.y], dtype=np.float64
            ),
        )
        result["map"] = {
            "width": int(m.info.width),
            "height": int(m.info.height),
            "resolution": float(m.info.resolution),
        }
    (out / "series.json").write_text(json.dumps(result))
    print(
        f"DONE {args.mode}: {n_scans} scans, {len(node.traj)} traj samples, "
        f"map={'yes' if node.latest_map is not None else 'NONE'}",
        flush=True,
    )
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
