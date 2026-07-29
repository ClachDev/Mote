#!/usr/bin/env python3
"""Headless autonomous exploration for building a sim map.

Drives the robot to cover an unmapped world so slam_toolbox sees every wall,
then exits. Two behaviours working together off the live ``/map``:

* follow — reactive left-wall following (the left-hand-rule maze walker) off
  ``/scan_filtered``: keep a wall on the left, turn right when blocked. Traces
  the connected boundary densely, so slam builds crisp walls with good loop
  closure. Great locally; but in a multiply-connected layout (the hospital's
  looping corridor grid) it cycles one loop and stops discovering.
* relocate — when following plateaus (the map stops growing) with frontiers
  (mapped-free next to unknown) still open, hand a frontier to Nav2's
  ``navigate_to_pose``. Nav2 plans a path *around* walls to a new region — which
  reactive steering cannot — then following resumes there. Unreachable
  frontiers are blacklisted so exploration keeps moving.

Ends when no reachable frontier remains (covered) or the budget elapses. Gates
on sim time (/clock), not wall time. The lidar frame is yawed from base_link,
so scan bearings are rotated into the base frame (via TF) before sectoring.

Run inside the mapping sim (sim_launch.py mode:=mapping is already up):
    python explore.py [--budget SECONDS]
"""

import argparse
import math
import sys
import time

import numpy as np
import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TwistStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan

CRUISE = 0.28  # forward speed (m/s); under the controller's 0.3 linear cap
DESIRED_LEFT = 0.8  # target distance to the left wall (m)
OBSTACLE = 0.55  # front clearance below which we stop and turn (m)
KP = 1.5  # wall-follow proportional gain
WZ_CAP = 0.6  # max wall-follow yaw rate (rad/s)
TURN = 0.8  # in-place turn rate when blocked (rad/s)
FOLLOW_BAND = 1.3  # left distance under which we track the wall (else go straight)
PLATEAU = 20.0  # sim s of flat map before handing off to a Nav2 relocation
MAX_FOLLOW = 70.0  # force a relocation after this long following (break loop cycling)
NAV_TIMEOUT = 60.0  # sim s to reach a relocation frontier before giving up
BL_RADIUS = 2.5  # blacklist this radius (m) around an unreachable frontier
REACHED = 1.0  # frontiers within this of the robot are ignored (m)
COARSE = 4.0  # coarse-cell size (m) for the visited grid — steers relocation to
# regions the robot has not been in yet, so it sweeps the whole map instead of
# cycling the loop it happens to be on
BACKOFF = 0.8  # aim the Nav2 goal this far back from the frontier, into known
# free space, so the planner can actually reach it (a frontier cell sits on the
# unknown boundary, inside costmap inflation, and goals there never quite arrive)


def norm(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Explorer(Node):
    def __init__(self):
        super().__init__("sim_explorer")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.scan = None
        self.odom = None
        self.grid = None
        self.yaw_off = None
        # /scan_filtered, not /scan: the filter chain nulls the blind-spot
        # sectors where the lidar sees the robot's own body (~0.12 m). Raw /scan
        # would read those self-hits as a permanent wall 13 cm to each side.
        self.create_subscription(LaserScan, "/scan_filtered", self.on_scan, 10)
        self.create_subscription(
            Odometry, "/diff_drive_controller/odom", self.on_odom, 10
        )
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)
        # Through the drive mux's teleop input, not straight at the controller:
        # explore stands in for a human driver, and while it is driving it
        # should out-rank whatever Nav2 is doing.
        self.cmd_pub = self.create_publisher(
            TwistStamped, "/cmd_vel_teleop_stamped", 10
        )
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def on_scan(self, msg):
        self.scan = msg

    def on_odom(self, msg):
        self.odom = msg

    def on_map(self, msg):
        self.grid = msg

    def sim_now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def resolve_yaw_offset(self):
        frame = self.scan.header.frame_id
        for _ in range(50):
            try:
                tf = self.tf_buffer.lookup_transform(
                    "base_link", frame, rclpy.time.Time()
                )
                self.yaw_off = yaw_of(tf.transform.rotation)
                print(
                    f"lidar frame '{frame}' yaw offset "
                    f"{math.degrees(self.yaw_off):+.1f} deg",
                    flush=True,
                )
                return
            except tf2_ros.TransformException:
                rclpy.spin_once(self, timeout_sec=0.2)
        print(f"WARNING: no TF base_link<-{frame}; assuming 0 offset", flush=True)
        self.yaw_off = 0.0

    def robot_xy(self):
        """(x, y) of base_link in the map frame, or None."""
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            return tf.transform.translation.x, tf.transform.translation.y
        except tf2_ros.TransformException:
            return None

    def sectors(self):
        """(front, left, right) nearest obstacle distance in base-frame sectors."""
        s = self.scan
        front = left = right = float("inf")
        a = s.angle_min + self.yaw_off
        inc = s.angle_increment
        for r in s.ranges:
            if s.range_min < r < s.range_max:
                b = norm(a)
                if abs(b) <= 0.35:
                    front = min(front, r)
                elif 0.7 <= b <= 2.4:
                    left = min(left, r)
                elif -2.4 <= b <= -0.7:
                    right = min(right, r)
            a += inc
        return front, left, right

    def publish(self, vx, wz):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.twist.linear.x = vx
        m.twist.angular.z = wz
        self.cmd_pub.publish(m)

    def spin_seed(self, seconds):
        start = self.sim_now()
        while self.sim_now() - start < seconds:
            self.publish(0.0, 0.6)
            rclpy.spin_once(self, timeout_sec=0.05)

    def wall_follow_step(self, front, left):
        vx_scale = 0.4 if front < 2 * OBSTACLE else 1.0
        if front < OBSTACLE:
            self.publish(0.0, -TURN)  # blocked: turn right, keep wall on the left
        elif left < FOLLOW_BAND:
            wz = max(-WZ_CAP, min(WZ_CAP, KP * (left - DESIRED_LEFT)))
            self.publish(CRUISE * vx_scale, wz)
        else:
            self.publish(CRUISE * vx_scale, 0.0)  # open: go straight to find a wall

    def frontiers(self):
        """(N,2) world-frame frontier points: free cells 4-adjacent to unknown."""
        g = self.grid
        if g is None:
            return np.empty((0, 2))
        w, h = g.info.width, g.info.height
        a = np.array(g.data, dtype=np.int16).reshape(h, w)
        free = a == 0
        unk = a == -1
        nbr = np.zeros_like(unk)
        nbr[1:, :] |= unk[:-1, :]
        nbr[:-1, :] |= unk[1:, :]
        nbr[:, 1:] |= unk[:, :-1]
        nbr[:, :-1] |= unk[:, 1:]
        ys, xs = np.where(free & nbr)
        res = g.info.resolution
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        return np.column_stack((ox + (xs + 0.5) * res, oy + (ys + 0.5) * res))

    def pick_target(self, fronts, here, blacklist, visited):
        """Frontier to relocate toward: the nearest one whose coarse region the
        robot has not visited yet (so exploration sweeps into new territory
        rather than cycling the current loop), falling back to the plain nearest
        if every frontier is in an already-visited region. Skips frontiers at the
        robot and near blacklisted (unreachable) points."""
        if len(fronts) == 0:
            return None
        d = np.hypot(fronts[:, 0] - here[0], fronts[:, 1] - here[1])
        ok = d > REACHED
        for bx, by in blacklist:
            ok &= np.hypot(fronts[:, 0] - bx, fronts[:, 1] - by) >= BL_RADIUS
        idx = np.where(ok)[0]
        if len(idx) == 0:
            return None
        cells = np.floor(fronts[idx] / COARSE).astype(int)
        unvisited = np.array([(int(cx), int(cy)) not in visited for cx, cy in cells])
        pool = idx[unvisited] if unvisited.any() else idx
        return fronts[pool[np.argmin(d[pool])]]

    def navigate_to(self, x, y):
        """Drive to (x, y) via Nav2. Returns 'ok'/'aborted'/'rejected'/'timeout'."""
        if not self.nav.wait_for_server(timeout_sec=5):
            return "rejected"
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0
        send = self.nav.send_goal_async(goal)
        t0 = self.sim_now()
        while not send.done() and self.sim_now() - t0 < 10:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not send.done() or send.result() is None or not send.result().accepted:
            return "rejected"
        handle = send.result()
        result_fut = handle.get_result_async()
        while self.sim_now() - t0 < NAV_TIMEOUT:
            rclpy.spin_once(self, timeout_sec=0.1)
            if result_fut.done():
                ok = result_fut.result().status == GoalStatus.STATUS_SUCCEEDED
                return "ok" if ok else "aborted"
        handle.cancel_goal_async()
        rclpy.spin_once(self, timeout_sec=0.5)
        return "timeout"


def known_cells(grid):
    return int(np.count_nonzero(np.array(grid.data, dtype=np.int16) >= 0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--budget", type=float, default=900.0, help="sim-time seconds")
    ap.add_argument(
        "--min-time",
        type=float,
        default=90.0,
        help="explore at least this long (sim s)",
    )
    args = ap.parse_args()

    rclpy.init()
    node = Explorer()

    deadline = time.monotonic() + 40
    while (node.scan is None or node.odom is None) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.scan is None:
        print("FAIL: no /scan_filtered received", file=sys.stderr)
        return 1
    node.resolve_yaw_offset()

    print("seeding map with an in-place spin...", flush=True)
    node.spin_seed(7.0)

    start = node.sim_now()
    best_known = 0
    best_at = start
    check_at = start
    last_relocate = start
    blacklist = []
    visited = set()

    while node.sim_now() - start < args.budget:
        rclpy.spin_once(node, timeout_sec=0.05)
        t = node.sim_now()
        front, left, _ = node.sectors()
        node.wall_follow_step(front, left)

        if t - check_at <= 5.0:
            continue
        check_at = t
        k = known_cells(node.grid) if node.grid is not None else 0
        if k > best_known + 100:
            best_known = k
            best_at = t
        fronts = node.frontiers()
        here = node.robot_xy()
        if here is not None:
            visited.add((int(here[0] // COARSE), int(here[1] // COARSE)))
        print(
            f"[{t - start:6.1f}s] known {k} frontiers {len(fronts)} "
            f"bl {len(blacklist)} seen {len(visited)} pose "
            f"{('(%+.1f,%+.1f)' % here) if here else '(?)'}",
            flush=True,
        )

        # Relocate when following has stalled (map flat) or has been cycling one
        # loop too long — either way head for a new region via Nav2.
        due = t - best_at > PLATEAU or t - last_relocate > MAX_FOLLOW
        if not due or here is None:
            continue

        target = node.pick_target(fronts, here, blacklist, visited)
        if target is None:
            if t - start > args.min_time:
                print(
                    f"[{t - start:6.1f}s] no reachable frontiers — covered", flush=True
                )
                break
            best_at = t
            continue
        # aim BACKOFF metres back from the frontier, into known free space
        vx, vy = here[0] - target[0], here[1] - target[1]
        n = math.hypot(vx, vy) or 1.0
        gx, gy = target[0] + BACKOFF * vx / n, target[1] + BACKOFF * vy / n
        dist = math.hypot(target[0] - here[0], target[1] - here[1])
        print(
            f"[{t - start:6.1f}s] relocate -> Nav2 ({target[0]:+.1f},{target[1]:+.1f}) "
            f"{dist:.1f} m away",
            flush=True,
        )
        res = node.navigate_to(gx, gy)
        print(f"[{node.sim_now() - start:6.1f}s]   relocate {res}", flush=True)
        if res != "ok":
            blacklist.append(target)
        best_at = node.sim_now()  # fresh windows wherever we ended up
        last_relocate = node.sim_now()
        check_at = node.sim_now()

    node.publish(0.0, 0.0)
    print(f"exploration done after {node.sim_now() - start:.1f}s sim time", flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
