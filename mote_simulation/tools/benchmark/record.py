#!/usr/bin/env python3
"""Record and score one scripted nav mission in the running sim.

Assumes the nav sim is already up (``sim_launch.py mode:=nav`` + a ground-truth
pose bridge) — ``bench.py`` owns launching and tearing that down. This process
is the ROS client for a single trial: it drives a sequence of NavigateToPose
goals, records the true and estimated trajectories plus scan clearance, cmd_vel,
and Nav2 recovery activity, then writes ``series.json`` (raw samples, for offline
re-scoring) and ``metrics.json`` (via the ROS-free :mod:`metrics` module).

Run one trial per process so each gets a clean rclpy context and DDS discovery,
mirroring how ``mote_bringup``'s explore tool drives the mapping sim. Everything
is gated on sim time (``/clock``), not wall time, so results are invariant to
real-time factor.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import rclpy
import tf2_ros
import yaml
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan

sys.path.insert(0, str(Path(__file__).resolve().parent))
import metrics  # noqa: E402

# Nav2's behavior_server plugins (nav2_params.yaml). Each exposes an action
# whose /<name>/_action/status topic reveals when a recovery fires.
RECOVERY_ACTIONS = ("spin", "backup", "drive_on_heading", "wait")


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def yaw_to_quat(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class Recorder(Node):
    def __init__(self, gt_topic, base_frame, sample_hz):
        super().__init__("bench_recorder")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.base_frame = base_frame
        self.truth = []  # [t, x, y, yaw]
        self.est = []  # [t, x, y, yaw]  (map->base: full localization estimate)
        self.odom_est = []  # [t, x, y, yaw]  (odom->base: dead-reckoning only)
        self.scan_min = []  # [t, min_range]
        self.cmd = []  # [t, vx, wz]
        self.recovery_ids = {a: set() for a in RECOVERY_ACTIONS}
        self._map_seen = False

        # Ground truth is the true model pose bridged from Gazebo's PosePublisher
        # (/model/mote/pose, a single PoseStamped in the world frame).
        self.create_subscription(PoseStamped, gt_topic, self.on_truth, 50)
        self.create_subscription(LaserScan, "/scan_filtered", self.on_scan, 10)
        self.create_subscription(
            TwistStamped, "/diff_drive_controller/cmd_vel", self.on_cmd, 20
        )
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)
        # Action status is offered transient-local+reliable; subscribe reliable
        # (volatile is compatible) with a deep queue so no recovery goal is missed.
        status_qos = QoSProfile(depth=100, reliability=QoSReliabilityPolicy.RELIABLE)
        for action in RECOVERY_ACTIONS:
            self.create_subscription(
                GoalStatusArray,
                f"/{action}/_action/status",
                lambda msg, a=action: self.on_recovery(msg, a),
                status_qos,
            )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.create_timer(1.0 / sample_hz, self.sample_est)

    def sim_now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def settle(self, seconds):
        """Spin (recording) for ``seconds`` of sim time so AMCL and the costmaps
        can populate before the first goal, without commanding any motion."""
        start = self.sim_now()
        while self.sim_now() - start < seconds:
            rclpy.spin_once(self, timeout_sec=0.1)

    def on_truth(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.position
        self.truth.append([t, p.x, p.y, yaw_of(msg.pose.orientation)])

    def sample_est(self):
        # map->base: the full localization estimate (AMCL + kinematic_icp). This
        # is what nav actually uses; AMCL's map correction can mask an odometry
        # change, so it is sampled alongside — not instead of — odom->base below.
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", self.base_frame, rclpy.time.Time()
            )
            t = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            tr = tf.transform.translation
            self.est.append([t, tr.x, tr.y, yaw_of(tf.transform.rotation)])
        except tf2_ros.TransformException:
            pass
        # odom->base: pure dead-reckoning (wheel odom refined by kinematic_icp)
        # with no map correction. This isolates odometry quality from the map
        # correction that would otherwise hide a change in it.
        try:
            tf = self.tf_buffer.lookup_transform(
                "odom", self.base_frame, rclpy.time.Time()
            )
            t = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            tr = tf.transform.translation
            self.odom_est.append([t, tr.x, tr.y, yaw_of(tf.transform.rotation)])
        except tf2_ros.TransformException:
            pass

    def on_scan(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        rmin = float("inf")
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max and r < rmin:
                rmin = r
        self.scan_min.append([t, rmin])

    def on_cmd(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.cmd.append([t, msg.twist.linear.x, msg.twist.angular.z])

    def on_map(self, msg):
        self._map_seen = True

    def on_recovery(self, msg, action):
        for status in msg.status_list:
            uid = bytes(status.goal_info.goal_id.uuid).hex()
            self.recovery_ids[action].add(uid)

    def wait_ready(self, timeout_s=90.0):
        """Block until Nav2's action server, a map->base TF, and ground truth are
        all available, or ``timeout_s`` wall seconds elapse."""
        if not self.nav.wait_for_server(timeout_sec=timeout_s):
            return False, "navigate_to_pose action server never appeared"
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            have_tf = self.tf_buffer.can_transform(
                "map", self.base_frame, rclpy.time.Time()
            )
            if have_tf and self.truth and self._map_seen:
                return True, "ready"
        missing = []
        if not self.truth:
            missing.append("ground truth (/model/mote/pose)")
        if not self._map_seen:
            missing.append("/map")
        return False, "not ready: " + ", ".join(missing or ["map->base TF"])

    def drive_goal(self, name, x, y, yaw, timeout_s):
        """Send one NavigateToPose goal, spinning until it terminates or times
        out (sim seconds). Returns a goal record dict."""
        z, w = yaw_to_quat(yaw)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = z
        goal.pose.pose.orientation.w = w

        t_send = self.sim_now()
        rec = {"name": name, "x": x, "y": y, "yaw": yaw, "t_send": t_send}
        send = self.nav.send_goal_async(goal)
        while not send.done() and self.sim_now() - t_send < 10:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not send.done() or send.result() is None or not send.result().accepted:
            rec.update(result="rejected", t_done=self.sim_now(), duration=None)
            return rec
        handle = send.result()
        result_fut = handle.get_result_async()
        while self.sim_now() - t_send < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.1)
            if result_fut.done():
                ok = result_fut.result().status == GoalStatus.STATUS_SUCCEEDED
                t_done = self.sim_now()
                rec.update(
                    result="ok" if ok else "aborted",
                    t_done=t_done,
                    duration=t_done - t_send,
                )
                return rec
        handle.cancel_goal_async()
        rclpy.spin_once(self, timeout_sec=0.5)
        rec.update(result="timeout", t_done=self.sim_now(), duration=None)
        return rec

    def series(self, goals):
        return {
            "truth": self.truth,
            "est": self.est,
            "odom_est": self.odom_est,
            "scan_min": self.scan_min,
            "cmd": self.cmd,
            "goals": goals,
            "recoveries": {
                **{a: len(ids) for a, ids in self.recovery_ids.items()},
                "total": sum(len(ids) for ids in self.recovery_ids.values()),
            },
        }


def load_goal_sequence(zones_file, order):
    with open(zones_file) as f:
        data = yaml.safe_load(f)
    zones = data.get("zones", {})
    seq = []
    for name in order:
        if name not in zones:
            raise SystemExit(f"zone '{name}' not in {zones_file}")
        z = zones[name]
        seq.append((name, float(z["x"]), float(z["y"]), float(z.get("yaw", 0.0))))
    return seq


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--zones-file", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--gt-topic", required=True, help="bridged ground-truth pose topic")
    ap.add_argument("--base-frame", default="base_footprint")
    ap.add_argument("--order", default="pickup,dropoff,home")
    ap.add_argument("--goal-timeout", type=float, default=120.0, help="sim s per goal")
    ap.add_argument(
        "--settle",
        type=float,
        default=8.0,
        help="sim s to let localization/costmaps settle before goal 1",
    )
    ap.add_argument("--sample-hz", type=float, default=20.0)
    ap.add_argument("--ready-timeout", type=float, default=120.0)
    args = ap.parse_args()

    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    sequence = load_goal_sequence(args.zones_file, args.order.split(","))

    rclpy.init()
    node = Recorder(args.gt_topic, args.base_frame, args.sample_hz)

    ok, reason = node.wait_ready(args.ready_timeout)
    if not ok:
        print(f"FAIL: {reason}", file=sys.stderr, flush=True)
        (out / "error.txt").write_text(reason + "\n")
        node.destroy_node()
        rclpy.shutdown()
        return 1
    print(f"READY: settling {args.settle:.0f}s then driving goal sequence", flush=True)
    node.settle(args.settle)

    goals = []
    for name, x, y, yaw in sequence:
        print(f">> goal {name} ({x:+.2f}, {y:+.2f})", flush=True)
        rec = node.drive_goal(name, x, y, yaw, args.goal_timeout)
        dur = f" ({rec['duration']:.1f}s)" if rec["duration"] is not None else ""
        print(f"   {name}: {rec['result']}{dur}", flush=True)
        goals.append(rec)

    series = node.series(goals)
    summary = metrics.summarize(series)
    (out / "series.json").write_text(json.dumps(series))
    (out / "metrics.json").write_text(json.dumps(summary, indent=2))
    print(
        f"DONE: {summary['goals']['n_succeeded']}/{summary['goals']['n_goals']} goals, "
        f"ATE rmse {summary['localization'].get('rmse_m', float('nan')):.3f} m",
        flush=True,
    )
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
