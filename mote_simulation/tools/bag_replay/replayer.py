#!/usr/bin/env python3
"""Replay one recorded mapping bag into a live SLAM/ICP node and capture outputs.

Assumes the stack under test is already running in the same (isolated) ROS graph
— ``replay.py`` owns launching ``async_slam_toolbox_node`` (slam mode) or
``kinematic_icp_online_node`` (icp mode) and tearing it down. This process is the
ROS client for a single parameter set: it streams the bag's ``/scan_filtered`` +
``/tf`` + ``/tf_static`` back onto the graph in sim-time order (driving ``/clock``
itself, so the node runs on bag time regardless of wall speed), records the
estimator's output trajectory, and grabs the finished ``/map``.

This is not a reimplementation of ``ros2 bag play``. What replay needs and bag
play has no hook for: stripping individual TF *edges* from inside ``/tf``
messages (bag play excludes whole topics only, and ``/tf`` must partially pass
through), gating replay pace on the estimator actually keeping up rather than
a fixed rate, windowing the stream in time, and capturing the output
trajectory/map/posegraph as it goes.

The one subtlety is TF ownership. A mapping bag's ``/tf`` already contains the
edges the *original* run produced — ``map->odom`` from slam_toolbox and
``odom->base_footprint`` from kinematic_icp. Replaying those verbatim would fight
the fresh node publishing the same edge, so the edge the node-under-test owns is
stripped from the replayed ``/tf`` (see ``STRIP``); everything else is passed
through. In slam mode the recorded ``odom->base`` is kept and *fed* to slam as
its odometry prior, exactly as on the robot.

**Pacing.** By default the stream is paced against the wall clock, so a 21-minute
bag costs 21 minutes — nearly all of it spent waiting, since the SLAM compute
inside is one or two minutes. ``--lockstep`` (slam mode) drops the waiting without
dropping the fidelity: ``acceptance.py`` predicts exactly which scans
slam_toolbox's gates will keep, only those are published, and each predicted
insertion must be acknowledged on ``/pose`` before the next scan goes out. Pace
is therefore set by the node's own consumption — queue-full drops are impossible
by construction — and a prediction the node disagrees with fails the leg loudly
instead of quietly changing the graph. ``replay.py --validate`` is the standing
proof that the two produce the same map.

Outputs (in ``--out-dir``): ``series.json`` (raw re-scorable trajectory) and, in
slam mode, ``map.npz`` (occupancy grid + resolution/origin for rendering and
map-quality metrics) plus the serialized posegraph (``map.posegraph`` +
``map.data``) — so a replayed session is not just scoreable but *continuable*:
the winning parameter set's output can be assembled into a site revision and
extended on the robot in the same frame. Run one replay per process for a
clean rclpy context.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
import tf2_ros
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

import rosbag2_py

sys.path.insert(0, str(Path(__file__).resolve().parent))
import acceptance  # noqa: E402
from tf_lookup import TfTree, se2_of  # noqa: E402

# The TF edge the node under test publishes itself — stripped from the replayed
# /tf so the recorded copy can't fight the fresh one. (parent, child) pairs.
STRIP = {
    "slam": {("map", "odom")},
    "icp": {("map", "odom"), ("odom", "base_footprint")},
}

# The edge --frame re-bases, so the session's map frame is born where asked.
PRIOR_EDGE = ("odom", "base_footprint")

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

# Lockstep publishes transforms as fast as the loop can go, and a stationary
# stretch of bag can put thousands of them between two fed scans. The listener's
# queue is finite and a dropped transform would silently change the odometry
# prior, so the burst is broken up to let the node drain. Costs milliseconds.
TF_BURST = 200
TF_BURST_DWELL = 0.001


class LockstepError(RuntimeError):
    """The running node did not do what the acceptance chain predicted."""


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def rebase_se2(x, y, yaw, frame):
    """Pre-multiply an SE2 onto a planar pose — the ``--frame`` re-basing."""
    fx, fy, fyaw = frame
    c, s = math.cos(fyaw), math.sin(fyaw)
    return c * x - s * y + fx, s * x + c * y + fy, yaw + fyaw


class Replayer(Node):
    def __init__(self, mode, sample_dt, frame=None):
        super().__init__("bag_replayer")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.mode = mode
        self.strip = STRIP[mode]
        self.out_parent, self.out_child = OUTPUT_EDGE[mode]
        self.sample_dt = sample_dt
        # Optional SE2 pre-multiplied onto the replayed odom->base_footprint
        # prior, so the session's map frame is *born* where you want it (e.g.
        # registered to an earlier revision's frame) instead of wherever the
        # bag's odometry happened to be pointing. Rotating artifacts after the
        # fact would shear the map against its posegraph; rotating the prior
        # keeps map, posegraph and trajectory consistent.
        self.frame = frame  # (x, y, yaw_rad) or None

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

        # Every graph insertion publishes the corrected pose at the scan's own
        # stamp, so /pose is both the lockstep acknowledgement and an exact count
        # of pose-graph nodes. Paced replays record it too, which is what makes a
        # reference and a lockstep leg comparable on node count.
        self.acks = set()
        self.expected_acks = None  # set by lockstep; None disables the audit
        self.bad_acks = []
        self.pose_traj = []
        if mode == "slam":
            self.create_subscription(
                PoseWithCovarianceStamped, "/pose", self.on_pose, 50
            )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.traj = []  # [sim_t, x, y, yaw]
        self._static_frames = {}  # (parent, child) -> TransformStamped

    def on_map(self, msg):
        self.latest_map = msg

    def on_pose(self, msg):
        ns = stamp_ns(msg.header.stamp)
        p = msg.pose.pose
        self.pose_traj.append(
            [ns / 1e9, p.position.x, p.position.y, yaw_of(p.orientation)]
        )
        if ns in self.acks:
            self.bad_acks.append(("duplicate", ns))
        elif self.expected_acks is not None and ns not in self.expected_acks:
            self.bad_acks.append(("unpredicted", ns))
        self.acks.add(ns)

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
        if self.frame is not None:
            for tr in kept:
                if (tr.header.frame_id, tr.child_frame_id) != PRIOR_EDGE:
                    continue
                t = tr.transform.translation
                q = tr.transform.rotation
                t.x, t.y, yaw = rebase_se2(t.x, t.y, yaw_of(q), self.frame)
                q.x = q.y = 0.0
                q.z = math.sin(yaw / 2)
                q.w = math.cos(yaw / 2)
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


def msg_transform(tr):
    t, q = tr.transform.translation, tr.transform.rotation
    return ((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))


class Plan:
    """What lockstep will publish, and the prediction it is answerable to."""

    def __init__(self):
        self.steps = []
        self.by_scan_index = {}
        self.stamp_of_scan = {}
        self.decisions = []
        self.chain_edges = set()
        self.expected_acks = set()
        self.last_scan_index = -1
        self.laser_offset = (0.0, 0.0, 0.0)
        self.truncated = False

    def counts(self):
        c = {}
        for d in self.decisions:
            c[d] = c.get(d, 0) + 1
        return c


def build_plan(args, gates, frame):
    """Read the bag once and decide, per scan, what the node would do with it.

    Mirrors the feed loop's own windowing exactly (a scan withheld by
    ``--skip-secs`` never reaches the node, so it never reaches the scan counter
    either), resolves each surviving scan's ``odom -> base_frame`` pose out of the
    bag's transforms the way slam_toolbox's tf2 buffer will, and runs the
    acceptance chain over the result.
    """
    plan = Plan()
    tree = TfTree()
    scans = []  # (bag scan index, header stamp ns)
    scan_frame = None
    t0_ns = None
    index = -1
    for topic, msg, t_ns in read_bag(args.bag):
        if t0_ns is None:
            t0_ns = t_ns
        sim_t = (t_ns - t0_ns) / 1e9
        if args.stop_secs and sim_t >= args.stop_secs:
            plan.truncated = True
            break
        if topic == "/tf_static":
            for tr in msg.transforms:
                tree.add_static(
                    tr.header.frame_id, tr.child_frame_id, msg_transform(tr)
                )
        elif topic == "/tf":
            for tr in msg.transforms:
                key = (tr.header.frame_id, tr.child_frame_id)
                if key in STRIP["slam"]:
                    continue
                t = msg_transform(tr)
                if frame is not None and key == PRIOR_EDGE:
                    (x, y, z), _ = t
                    x, y, yaw = rebase_se2(x, y, se2_of(t)[2], frame)
                    t = ((x, y, z), (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))
                tree.add_dynamic(key[0], key[1], stamp_ns(tr.header.stamp), t)
        else:
            index += 1
            if sim_t < args.skip_secs:
                continue
            if args.max_scans and len(scans) >= args.max_scans:
                plan.truncated = True
                continue
            if scan_frame is None:
                scan_frame = msg.header.frame_id
            scans.append((index, stamp_ns(msg.header.stamp)))
    tree.finalize()

    poses = []
    have_offset = False
    for _, ns in scans:
        base = tree.lookup(args.odom_frame, args.base_frame, ns)
        laser = tree.lookup(args.base_frame, scan_frame, ns) if scan_frame else None
        if base is None or laser is None:
            poses.append(None)
            continue
        if not have_offset:
            # The node reads the mounting offset off TF once, on the first scan
            # it can place, and caches it for the rest of the session.
            plan.laser_offset = se2_of(laser)
            have_offset = True
        poses.append(se2_of(base))

    plan.decisions = acceptance.simulate(
        [(ns, p) for (_, ns), p in zip(scans, poses)], gates, plan.laser_offset
    )
    plan.steps = acceptance.feed_plan(plan.decisions, gates)
    plan.by_scan_index = {scans[s.index][0]: s for s in plan.steps}
    plan.stamp_of_scan = dict(scans)
    plan.expected_acks = {scans[s.index][1] for s in plan.steps if s.expect_ack}
    plan.last_scan_index = max(plan.by_scan_index, default=-1)
    plan.chain_edges = tree.dynamic_chain(args.odom_frame, args.base_frame)
    if scan_frame:
        plan.chain_edges |= tree.dynamic_chain(args.base_frame, scan_frame)
    return plan


def wait_for_stack(node, timeout=30.0):
    """Block until the node under test has matched us, so nothing is fed into the void.

    Paced replay absorbed discovery in its first seconds of real-time pacing;
    lockstep publishes its first scan immediately, and a scan lost to an unmatched
    subscription is a scan the acceptance prediction still expects.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        ready = node.count_subscribers("/scan_filtered") > 0
        if node.mode == "slam":
            ready = ready and node.count_publishers("/pose") > 0
        if ready:
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.05)
            return True
        rclpy.spin_once(node, timeout_sec=0.1)
    return False


class FeedResult:
    """What a feed leg leaves behind for the settle and the report."""

    def __init__(self, t0_ns, last_ns, n_scans, truncated, last_sample, last_scan_ns):
        self.t0_ns = t0_ns
        self.last_ns = last_ns
        self.n_scans = n_scans
        self.truncated = truncated
        self.last_sample = last_sample
        # Stamp of the last scan handed to the node. slam stamps each occupancy
        # grid with the last scan it *received*, so this is what separates the
        # finished map from one published earlier in the run.
        self.last_scan_ns = last_scan_ns


def feed_paced(node, args):
    """The reference: stream every message at (a fraction of) real time."""
    wall0 = None
    t0_ns = None
    last_ns = None
    last_sample = -1e18  # sim-time of the last trajectory sample (gap-robust)
    n_scans = 0
    last_scan_ns = 0
    stop = False

    for topic, msg, t_ns in read_bag(args.bag):
        if t0_ns is None:
            t0_ns = t_ns
            wall0 = time.monotonic()
        sim_t = (t_ns - t0_ns) / 1e9
        last_ns = t_ns

        # Pace to the requested fraction of realtime, spinning so /map and
        # TF callbacks fire while we wait.
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
            if sim_t >= args.skip_secs:
                node.scan_pub.publish(msg)
                n_scans += 1
                last_scan_ns = stamp_ns(msg.header.stamp)
        rclpy.spin_once(node, timeout_sec=0.0)

        if sim_t - last_sample >= args.sample_dt:
            node.sample_output(sim_t)
            last_sample = sim_t

        if args.max_scans and n_scans >= args.max_scans:
            stop = True
            break
        if args.stop_secs and sim_t >= args.stop_secs:
            stop = True
            break
    return FeedResult(t0_ns, last_ns, n_scans, stop, last_sample, last_scan_ns)


def feed_lockstep(node, args, plan):
    """Consumption-paced: publish only scans the node will keep, one at a time.

    Transforms are published *ahead* of the scan that needs them. In bag order a
    scan is recorded before the odometry derived from it, which the live node
    handles by parking the scan in its tf2 message filter — a wait that would
    deadlock a feeder blocked on that same scan's acknowledgement. So a fed scan
    is held back until every dynamic edge on its frame chain has a sample at or
    past its stamp. tf2 interpolates between the same two samples either way, so
    the pose the node reads is unchanged.
    """
    node.expected_acks = plan.expected_acks
    t0_ns = None
    last_ns = None
    index = -1
    pending = []  # [(step, msg, stamp_ns)]
    covered = {e: None for e in plan.chain_edges}
    n_scans = 0
    last_scan_ns = 0
    tf_since_dwell = 0
    done = False

    def coverage():
        if not covered:
            return None
        vals = list(covered.values())
        return None if any(v is None for v in vals) else min(vals)

    def publish(step, msg, ns):
        nonlocal n_scans, last_scan_ns
        node.scan_pub.publish(msg)
        last_scan_ns = ns
        if step.role != acceptance.FILLER:
            n_scans += 1
        if step.expect_ack:
            deadline = time.monotonic() + args.ack_timeout
            while ns not in node.acks:
                if time.monotonic() > deadline:
                    raise LockstepError(
                        f"no /pose acknowledgement for the scan at {ns} "
                        f"(bag scan {step.index}, insertion "
                        f"{len(node.acks) + 1}/{len(plan.expected_acks)}) within "
                        f"{args.ack_timeout:g}s — the predicted acceptance chain "
                        "disagrees with the running node"
                    )
                rclpy.spin_once(node, timeout_sec=0.01)
        else:
            rclpy.spin_once(node, timeout_sec=0.0)
        if node.bad_acks:
            raise LockstepError(
                f"unexpected /pose acknowledgements: {node.bad_acks[:5]}"
            )

    def drain():
        cov = coverage()
        while pending and cov is not None and pending[0][2] <= cov:
            step, msg, ns = pending.pop(0)
            publish(step, msg, ns)

    for topic, msg, t_ns in read_bag(args.bag):
        if t0_ns is None:
            t0_ns = t_ns
        sim_t = (t_ns - t0_ns) / 1e9
        if args.stop_secs and sim_t >= args.stop_secs:
            break
        last_ns = t_ns
        node.publish_clock(t_ns)

        if topic == "/tf_static":
            node.handle_tf_static(msg)
        elif topic == "/tf":
            node.handle_tf(msg)
            for tr in msg.transforms:
                key = (tr.header.frame_id, tr.child_frame_id)
                if key in covered:
                    ns = stamp_ns(tr.header.stamp)
                    if covered[key] is None or ns > covered[key]:
                        covered[key] = ns
            tf_since_dwell += 1
            if tf_since_dwell >= TF_BURST:
                tf_since_dwell = 0
                rclpy.spin_once(node, timeout_sec=0.0)
                time.sleep(TF_BURST_DWELL)
            drain()
        else:
            index += 1
            step = plan.by_scan_index.get(index)
            if step is not None:
                ns = plan.stamp_of_scan[index]
                cov = coverage()
                if cov is not None and ns <= cov:
                    publish(step, msg, ns)
                else:
                    pending.append((step, msg, ns))
        if index >= plan.last_scan_index and not pending:
            done = True
            break

    drain()
    if pending:
        raise LockstepError(
            f"{len(pending)} predicted scans never got covering transforms"
        )
    missing = plan.expected_acks - node.acks
    if missing:
        raise LockstepError(f"{len(missing)} predicted insertions were never acked")
    if not done and plan.steps:
        raise LockstepError("the bag ran out before the last predicted scan")
    return FeedResult(t0_ns, last_ns, n_scans, plan.truncated, -1e18, last_scan_ns)


def wait_for_final_map(node, args, fed):
    """Take the grid built *after* the last scan we fed, not the next one to arrive.

    slam rebuilds the whole occupancy grid on a 5-second **wall** timer and stamps
    each one with the last scan it received, so "the newest map" and "the finished
    map" are different things whenever a replay outruns that timer — which
    lockstep does by an order of magnitude. Worse, the publisher is
    ``transient_local`` and slam's own ``map_saver`` holds a subscription, so a
    partially-built grid is always waiting to be delivered the moment a replayer
    subscribes: the first full-bag lockstep leg captured one, and it was missing
    the last two minutes of the run.
    """
    end_ns = fed.last_ns + int(args.settle * 1e9)
    deadline = time.monotonic() + args.map_timeout
    while time.monotonic() < deadline:
        m = node.latest_map
        if m is not None and stamp_ns(m.header.stamp) >= fed.last_scan_ns:
            print(
                f"map: {m.info.width}x{m.info.height} at "
                f"{stamp_ns(m.header.stamp) / 1e9:.3f}",
                flush=True,
            )
            return True
        node.publish_clock(end_ns)
        rclpy.spin_once(node, timeout_sec=0.05)
    print(
        f"WARNING: no /map covering the last fed scan within "
        f"{args.map_timeout:g}s — the captured map is incomplete",
        file=sys.stderr,
        flush=True,
    )
    return False


def settle_paced(node, args, t0_ns, last_ns, last_sample):
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


def serialize_posegraph(node, out):
    from slam_toolbox.srv import SerializePoseGraph

    cli = node.create_client(SerializePoseGraph, "/slam_toolbox/serialize_map")
    ok = False
    if cli.wait_for_service(timeout_sec=5.0):
        req = SerializePoseGraph.Request()
        req.filename = str(out / "map")
        fut = cli.call_async(req)
        deadline = time.monotonic() + 30
        while not fut.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        ok = fut.done() and fut.result() is not None and fut.result().result == 0
    print(f"posegraph serialize: {'ok' if ok else 'FAILED'}", flush=True)
    return ok


def load_gates(params_file):
    if not params_file:
        return acceptance.Gates()
    return acceptance.Gates.from_params(
        yaml.safe_load(Path(params_file).read_text()) or {}
    )


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
    ap.add_argument(
        "--skip-secs",
        type=float,
        default=0.0,
        help="withhold scans before this bag-relative time (TF still replays, "
        "so the odometry prior is warm when insertion starts) — surgical "
        "trim of a bad opening, e.g. a collision during the seeding spin",
    )
    ap.add_argument(
        "--stop-secs",
        type=float,
        default=0.0,
        help="stop feeding at this bag-relative time (0 = whole bag) — "
        "surgical trim of a drifty ending",
    )
    ap.add_argument(
        "--frame",
        nargs=3,
        type=float,
        metavar=("X", "Y", "YAW_DEG"),
        help="SE2 pre-multiplied onto the odometry prior so the map frame is "
        "born aligned (e.g. registered to an earlier revision)",
    )
    ap.add_argument(
        "--lockstep",
        action="store_true",
        help="compute-bound replay (slam mode): publish only the scans "
        "slam_toolbox's own gates would keep, one at a time, waiting for each "
        "predicted insertion to be acknowledged on /pose. Wall pacing is "
        "dropped; a prediction the node disagrees with fails the leg.",
    )
    ap.add_argument(
        "--slam-params",
        default=None,
        help="the params file the stack was launched with — lockstep reads the "
        "acceptance gates from it, so no gate value is hardcoded here",
    )
    ap.add_argument("--odom-frame", default="odom")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument(
        "--ack-timeout",
        type=float,
        default=120.0,
        help="lockstep: wall seconds to wait for one insertion to be acked",
    )
    ap.add_argument(
        "--map-timeout",
        type=float,
        default=90.0,
        help="wall seconds to wait for an occupancy grid built after the last "
        "scan (slam rebuilds it on its own wall timer, so a fast replay finishes "
        "well before one exists)",
    )
    args = ap.parse_args()

    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)

    if args.lockstep and args.mode != "slam":
        print("FAIL: --lockstep is slam mode only", file=sys.stderr)
        return 2

    frame = None
    if args.frame:
        frame = (args.frame[0], args.frame[1], math.radians(args.frame[2]))

    plan = None
    if args.lockstep:
        gates = load_gates(args.slam_params)
        t_plan = time.monotonic()
        try:
            plan = build_plan(args, gates, frame)
        except ValueError as e:
            print(f"FAIL: {e}", file=sys.stderr)
            (out / "error.txt").write_text(f"{e}\n")
            return 1
        c = plan.counts()
        print(
            f"acceptance plan ({time.monotonic() - t_plan:.0f}s, gates {gates}): "
            f"{len(plan.decisions)} scans -> {c.get(acceptance.ACCEPT, 0)} insertions, "
            f"{c.get(acceptance.MAPPER_REJECT, 0)} fed-not-inserted, "
            f"{c.get(acceptance.NODE_REJECT, 0)} gated out, "
            f"{c.get(acceptance.NO_TF, 0)} without transforms; "
            f"{len(plan.steps)} scans to publish "
            f"({sum(1 for s in plan.steps if s.role == acceptance.FILLER)} of them "
            f"counter padding); laser offset {plan.laser_offset}",
            flush=True,
        )
        if not plan.steps:
            print("FAIL: the acceptance plan feeds no scans", file=sys.stderr)
            return 1

    rclpy.init()
    node = Replayer(args.mode, args.sample_dt, frame=frame)
    if not wait_for_stack(node):
        print("FAIL: the node under test never matched our topics", file=sys.stderr)
        (out / "error.txt").write_text("stack never matched the replayer\n")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    wall0 = time.monotonic()
    try:
        if args.lockstep:
            fed = feed_lockstep(node, args, plan)
        else:
            fed = feed_paced(node, args)
    except LockstepError as e:
        print(f"FAIL: {e}", file=sys.stderr)
        (out / "error.txt").write_text(f"{e}\n")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    if fed.t0_ns is None:
        print("FAIL: bag had no replayable topics", file=sys.stderr)
        (out / "error.txt").write_text("bag had no /scan_filtered /tf /tf_static\n")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    if not args.lockstep:
        settle_paced(node, args, fed.t0_ns, fed.last_ns, fed.last_sample)
    map_final = args.mode != "slam" or wait_for_final_map(node, args, fed)
    feed_wall = time.monotonic() - wall0

    posegraph_ok = serialize_posegraph(node, out) if args.mode == "slam" else False

    traj = node.traj
    traj_source = "tf"
    if args.lockstep:
        # map->odom is broadcast on a wall-clock timer, so sampling TF over a
        # lockstep leg would yield a handful of poses for the whole bag. The pose
        # graph's own node poses are denser and exact — but they are a different
        # sampling, so only compare legs fed the same way.
        traj = [[t - fed.t0_ns / 1e9, x, y, yaw] for t, x, y, yaw in node.pose_traj]
        traj_source = "pose"

    result = {
        "mode": args.mode,
        "bag": str(args.bag),
        "feed": "lockstep" if args.lockstep else "paced",
        "truncated": bool(fed.truncated),
        "n_scans": fed.n_scans,
        "n_inserted": len(node.acks),
        "posegraph": posegraph_ok,
        "map_final": bool(map_final),
        # The stamp of every scan that became a pose-graph node. --validate
        # compares the sets, not just the counts: two legs can insert the same
        # number of scans and not the same scans.
        "inserted_stamps": sorted(node.acks),
        "wall_s": round(feed_wall, 1),
        "traj_source": traj_source,
        "traj": traj,
    }
    if plan is not None:
        result["plan"] = plan.counts()
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
            "origin": [
                float(m.info.origin.position.x),
                float(m.info.origin.position.y),
            ],
        }
    (out / "series.json").write_text(json.dumps(result))
    print(
        f"DONE {args.mode} ({result['feed']}, {feed_wall:.0f}s wall): "
        f"{fed.n_scans} scans fed, {len(node.acks)} inserted, "
        f"{len(traj)} traj samples, "
        f"map={'yes' if map_final else 'INCOMPLETE' if node.latest_map else 'NONE'}",
        flush=True,
    )
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
