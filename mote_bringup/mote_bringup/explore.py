#!/usr/bin/env python3
"""Autonomous exploration for building a map — real robot or sim.

Drives the robot to cover an unmapped space so slam_toolbox sees every wall,
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

A stuck detector watches for commanded forward motion that produces no
map-frame displacement — the signature of the obstacles a 2D lidar cannot see
(rug edges, cables, low clutter): the wheels are told to drive, scan says the
way is clear, and the pose does not move. On firing it backs off, turns away
from the followed wall, and blacklists the spot for relocation targets.

Ends when no reachable frontier remains (covered) or the budget elapses. All
mission gates run on the node clock: wall time on the robot, /clock under
``--sim-time``. The lidar frame is yawed from base_link, so scan bearings are
rotated into the base frame (via TF) before sectoring.

Run on the robot, inside a mapping mission (`pixi run mapping` is already up —
run both on the Pi so a wifi drop cannot end the session):
    pixi run explore
Run in the sim (sim_launch.py mode:=mapping is already up):
    ros2 run mote_bringup explore --sim-time [--budget SECONDS]

The default geometry thresholds suit corridor-scale spaces; domestic layouts
(~0.75 m doorways, ~1 m hallways) want them tightened — see --cruise,
--obstacle, --desired-left, --follow-band.
"""

import argparse
import math
import sys
import time
from collections import deque

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

KP = 1.5  # wall-follow proportional gain
WZ_CAP = 0.6  # max wall-follow yaw rate (rad/s)
TURN = 0.8  # in-place turn rate when blocked (rad/s)
PLATEAU = 20.0  # s of flat map before handing off to a Nav2 relocation
MAX_FOLLOW = 70.0  # force a relocation after this long following (break loop cycling)
NAV_TIMEOUT = 60.0  # s to reach a relocation frontier before giving up
BL_RADIUS = 2.5  # blacklist this radius (m) around an unreachable frontier
REACHED = 1.0  # frontiers within this of the robot are ignored (m)
COARSE = 4.0  # coarse-cell size (m) for the visited grid — steers relocation to
# regions the robot has not been in yet, so it sweeps the whole map instead of
# cycling the loop it happens to be on
BACKOFF = 0.8  # aim the Nav2 goal this far back from the frontier, into known
# free space, so the planner can actually reach it (a frontier cell sits on the
# unknown boundary, inside costmap inflation, and goals there never quite arrive)
STUCK_WINDOW = 6.0  # s of commanded-forward, no-displacement before escaping
STUCK_TRAVEL = 0.10  # map-frame displacement (m) under which "no displacement"
ESCAPE_REVERSE = 1.5  # s of backing off when stuck
ESCAPE_TURN = 1.5  # s of turning away from the followed wall after backing off
SCAN_STALE = 1.0  # s without a scan before we stop and wait — driving on a
# frozen scan is driving blind, and a frozen pose then reads as "stuck",
# poisoning the frontier blacklist (observed when a wifi drop wedged the
# on-robot DDS graph mid-run: phantom stucks starved the frontier picker and
# the mission exited "covered" half done). 1 s is ~10 missed scans; the same
# wedge also showed scans dribbling in at 1-2 s intervals, each one seconds
# old — hence tight, plus the settling period below.
SETTLE = 3.0  # s of continuous fresh data required after a stale spell or a
# pose jump before driving resumes — resuming instantly fired a Nav2
# relocation built from the frozen-then-corrupted map (run 2's wild phase)
KIDNAP_JUMP = 0.5  # map-frame jump (m) between loop iterations that cannot be
# real motion: a carried robot or a large SLAM correction. Either way the pose
# history is fiction — stop and settle rather than act on it.
PIN_WINDOW = 45.0  # s of pose history examined for the orbit check
PIN_RADIUS = 1.5  # confinement radius (m): this long inside this circle while
# following means the robot is orbiting an island (free-standing furniture) —
# the map-stall check alone misses it, because each lap trickles a few new
# cells through doorway sightlines and keeps resetting the plateau timer
ORPHAN_HOLD = 120.0  # s to hold zero velocity and retry a cancel at exit when
# Nav2 still holds a goal we could not confirm dead — the mux hands the wheels
# to Nav2 one second after our zeros stop, so exiting with a live goal turns
# "mission over" into "Nav2 drives at a stale target unsupervised"


def norm(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class StuckDetector:
    """Commanded forward motion with no actual displacement, over a window.

    Fed ``(t, x, y)`` pose samples while forward motion is commanded; any
    non-forward command resets the window (turning in place at a wall is not
    the stuck mode this detects). Fires when the window is full and the net
    displacement across it stays under ``min_travel`` — pure logic, no ROS, so
    the threshold behaviour is unit-testable.
    """

    def __init__(self, window=STUCK_WINDOW, min_travel=STUCK_TRAVEL):
        self.window = window
        self.min_travel = min_travel
        self.samples = deque()

    def reset(self):
        self.samples.clear()

    def update(self, t, x, y, forward):
        if not forward:
            self.reset()
            return False
        self.samples.append((t, x, y))
        while self.samples and t - self.samples[0][0] > self.window:
            self.samples.popleft()
        t0, x0, y0 = self.samples[0]
        if t - t0 < 0.9 * self.window:
            return False
        return math.hypot(x - x0, y - y0) < self.min_travel


class Explorer(Node):
    def __init__(self, args):
        super().__init__("explorer")
        if args.sim_time:
            self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.cruise = args.cruise
        self.obstacle = args.obstacle
        self.desired_left = args.desired_left
        self.follow_band = args.follow_band
        self.bl_radius = args.blacklist_radius
        self.scan = None
        self.scan_at = None
        self.odom = None
        self.grid = None
        self.yaw_off = None
        self.cmd_vx = 0.0
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
        self.orphan = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def on_scan(self, msg):
        self.scan = msg
        self.scan_at = self.now_s()

    def on_odom(self, msg):
        self.odom = msg

    def on_map(self, msg):
        self.grid = msg

    def now_s(self):
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
        self.cmd_vx = vx

    def spin_seed(self, seconds):
        start = self.now_s()
        while self.now_s() - start < seconds:
            self.publish(0.0, 0.6)
            rclpy.spin_once(self, timeout_sec=0.05)

    def wall_follow_step(self, front, left):
        vx_scale = 0.4 if front < 2 * self.obstacle else 1.0
        if front < self.obstacle:
            self.publish(0.0, -TURN)  # blocked: turn right, keep wall on the left
        elif left < self.follow_band:
            wz = max(-WZ_CAP, min(WZ_CAP, KP * (left - self.desired_left)))
            self.publish(self.cruise * vx_scale, wz)
        else:
            self.publish(self.cruise * vx_scale, 0.0)  # open: find a wall
        return self.cmd_vx > 0.0

    def escape(self):
        """Back off and turn away from the followed wall (right, since the wall
        is kept on the left) — the recovery for lidar-invisible obstacles."""
        for vx, wz, dur in ((-0.15, 0.0, ESCAPE_REVERSE), (0.0, -TURN, ESCAPE_TURN)):
            start = self.now_s()
            while self.now_s() - start < dur:
                self.publish(vx, wz)
                rclpy.spin_once(self, timeout_sec=0.05)
        self.publish(0.0, 0.0)

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

    def pick_target(self, fronts, here, blacklist, visited, far=False):
        """Frontier to relocate toward: the nearest one whose coarse region the
        robot has not visited yet (so exploration sweeps into new territory
        rather than cycling the current loop), falling back to the plain nearest
        if every frontier is in an already-visited region. Skips frontiers at the
        robot and near blacklisted (unreachable) points.

        ``far`` flips the choice to the farthest candidate. Nearest is right
        when following is productive and just needs a hop; it is exactly wrong
        when the map has stalled because the robot is orbiting an island (the
        left-hand rule's failure mode on free-standing furniture) — the nearest
        frontier sits beside the island and re-inserts the robot into the same
        orbit. Farthest hands Nav2 a goal that leaves the room."""
        if len(fronts) == 0:
            return None
        d = np.hypot(fronts[:, 0] - here[0], fronts[:, 1] - here[1])
        ok = d > REACHED
        for bx, by in blacklist:
            ok &= np.hypot(fronts[:, 0] - bx, fronts[:, 1] - by) >= self.bl_radius
        idx = np.where(ok)[0]
        if len(idx) == 0:
            return None
        cells = np.floor(fronts[idx] / COARSE).astype(int)
        unvisited = np.array([(int(cx), int(cy)) not in visited for cx, cy in cells])
        pool = idx[unvisited] if unvisited.any() else idx
        pick = np.argmax(d[pool]) if far else np.argmin(d[pool])
        return fronts[pool[pick]]

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
        t0 = self.now_s()
        while not send.done() and self.now_s() - t0 < 10:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not send.done() or send.result() is None or not send.result().accepted:
            return "rejected"
        handle = send.result()
        result_fut = handle.get_result_async()
        while self.now_s() - t0 < NAV_TIMEOUT:
            rclpy.spin_once(self, timeout_sec=0.1)
            if result_fut.done():
                ok = result_fut.result().status == GoalStatus.STATUS_SUCCEEDED
                return "ok" if ok else "aborted"
        # The cancel must be confirmed dead, not fired and forgotten: over a
        # wedged graph the cancel never arrives, and a goal that outlives this
        # process gets the wheels one mux timeout after our commands stop.
        if not self.cancel_confirmed(handle, result_fut):
            self.orphan = (handle, result_fut)
            print(
                "WARNING: Nav2 goal cancel unconfirmed — holding it as an "
                "orphan; exit will not release the wheels until it is dead",
                flush=True,
            )
        return "timeout"

    def cancel_confirmed(self, handle, result_fut, wait_s=10.0):
        """Cancel a Nav2 goal and wait for a terminal result. True == dead."""
        handle.cancel_goal_async()
        t0 = self.now_s()
        while self.now_s() - t0 < wait_s:
            self.publish(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.1)
            if result_fut.done():
                return True
        return False


def known_cells(grid):
    return int(np.count_nonzero(np.array(grid.data, dtype=np.int16) >= 0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--budget", type=float, default=900.0, help="mission seconds")
    ap.add_argument(
        "--min-time",
        type=float,
        default=90.0,
        help="explore at least this long (s)",
    )
    ap.add_argument(
        "--sim-time",
        action="store_true",
        help="gate on /clock (running inside the sim)",
    )
    ap.add_argument("--cruise", type=float, default=0.28, help="forward speed (m/s)")
    ap.add_argument(
        "--obstacle",
        type=float,
        default=0.55,
        help="front clearance below which we stop and turn (m)",
    )
    ap.add_argument(
        "--desired-left",
        type=float,
        default=0.8,
        help="target distance to the left wall (m)",
    )
    ap.add_argument(
        "--follow-band",
        type=float,
        default=1.3,
        help="left distance under which we track the wall (m)",
    )
    ap.add_argument(
        "--blacklist-radius",
        type=float,
        default=2.5,
        help="exclude relocation frontiers within this of a failed/stuck spot "
        "(m); shrink for domestic scale or a few stuck spots starve the picker",
    )
    args = ap.parse_args()

    rclpy.init()
    node = Explorer(args)

    deadline = time.monotonic() + 40
    while (node.scan is None or node.odom is None) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.scan is None:
        print("FAIL: no /scan_filtered received", file=sys.stderr)
        return 1
    node.resolve_yaw_offset()

    print("seeding map with an in-place spin...", flush=True)
    node.spin_seed(7.0)

    start = node.now_s()
    best_known = 0
    best_at = start
    check_at = start
    last_relocate = start
    blacklist = []
    visited = set()
    stuck = StuckDetector()

    stale_since = None
    settle_until = start
    prev_here = None
    recent = deque()
    while node.now_s() - start < args.budget:
        rclpy.spin_once(node, timeout_sec=0.05)
        t = node.now_s()
        if node.scan_at is None or t - node.scan_at > SCAN_STALE:
            if stale_since is None:
                stale_since = t
                print(
                    f"[{t - start:6.1f}s] scan stale — stopping until data returns",
                    flush=True,
                )
            node.publish(0.0, 0.0)
            stuck.reset()
            prev_here = None
            continue
        if stale_since is not None:
            print(
                f"[{t - start:6.1f}s] scan resumed after {t - stale_since:.0f}s "
                f"— settling for {SETTLE:.0f}s",
                flush=True,
            )
            stale_since = None
            settle_until = t + SETTLE
        if t < settle_until:
            # Hold still and hold the mission clocks: acting the instant data
            # returns means acting on whatever the outage did to the map — the
            # observed failure was an immediate Nav2 relocation into fiction.
            node.publish(0.0, 0.0)
            stuck.reset()
            best_at = last_relocate = check_at = t
            continue

        here = node.robot_xy()
        if here is not None and prev_here is not None:
            jump = math.hypot(here[0] - prev_here[0], here[1] - prev_here[1])
            if jump > KIDNAP_JUMP:
                print(
                    f"[{t - start:6.1f}s] pose jumped {jump:.1f} m in one tick "
                    "(carried, or a large SLAM correction) — settling",
                    flush=True,
                )
                node.publish(0.0, 0.0)
                stuck.reset()
                settle_until = t + SETTLE
                prev_here = here
                continue
        if here is not None:
            prev_here = here

        front, left, _ = node.sectors()
        forward = node.wall_follow_step(front, left)

        if here is not None and stuck.update(t, here[0], here[1], forward):
            print(
                f"[{t - start:6.1f}s] stuck at ({here[0]:+.1f},{here[1]:+.1f}) "
                "(forward commanded, no map-frame motion) — escaping",
                flush=True,
            )
            node.escape()
            blacklist.append(here)
            stuck.reset()
            # best_at deliberately not reset: a run of stuck-escapes means the
            # map is not growing, and that is exactly what the plateau trigger
            # exists to notice — resetting it here would let repeated bumps
            # suppress the Nav2 relocation that breaks the cycle.
            continue

        if t - check_at <= 5.0:
            continue
        check_at = t
        k = known_cells(node.grid) if node.grid is not None else 0
        if k > best_known + 100:
            best_known = k
            best_at = t
        fronts = node.frontiers()
        pinned = False
        if here is not None:
            visited.add((int(here[0] // COARSE), int(here[1] // COARSE)))
            recent.append((t, here[0], here[1]))
            while recent and t - recent[0][0] > PIN_WINDOW:
                recent.popleft()
            if t - recent[0][0] > 0.8 * PIN_WINDOW:
                cx = sum(p[1] for p in recent) / len(recent)
                cy = sum(p[2] for p in recent) / len(recent)
                pinned = all(
                    math.hypot(px - cx, py - cy) < PIN_RADIUS for _, px, py in recent
                )
        print(
            f"[{t - start:6.1f}s] known {k} frontiers {len(fronts)} "
            f"bl {len(blacklist)} seen {len(visited)} pose "
            f"{('(%+.1f,%+.1f)' % here) if here else '(?)'}",
            flush=True,
        )

        # Relocate when following has stalled (map flat), is pinned inside an
        # orbit-sized circle, or has been cycling one loop too long — head for
        # a new region via Nav2. Stalled or pinned means following is going in
        # circles (island orbit), so those relocate far; the cycling-timer case
        # is merely "spread out", nearest.
        stalled = t - best_at > PLATEAU
        due = stalled or pinned or t - last_relocate > MAX_FOLLOW
        if not due or here is None:
            continue

        far = stalled or pinned
        target = node.pick_target(fronts, here, blacklist, visited, far=far)
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
            f"[{t - start:6.1f}s] relocate{'(far)' if far else ''} -> Nav2 "
            f"({target[0]:+.1f},{target[1]:+.1f}) {dist:.1f} m away",
            flush=True,
        )
        res = node.navigate_to(gx, gy)
        print(f"[{node.now_s() - start:6.1f}s]   relocate {res}", flush=True)
        if res != "ok":
            blacklist.append(target)
        stuck.reset()
        prev_here = None
        recent.clear()
        best_at = node.now_s()  # fresh windows wherever we ended up
        last_relocate = node.now_s()
        check_at = node.now_s()

    node.publish(0.0, 0.0)
    print(f"exploration done after {node.now_s() - start:.1f}s", flush=True)
    # Exiting stops our zero-commands, and one mux timeout later Nav2 owns the
    # wheels — so an unconfirmed-dead goal must be resolved before we go.
    if node.orphan is not None:
        deadline = node.now_s() + ORPHAN_HOLD
        warned_at = 0.0
        while node.now_s() < deadline:
            handle, result_fut = node.orphan
            if result_fut.done():
                node.orphan = None
                print("orphaned Nav2 goal confirmed dead", flush=True)
                break
            if node.now_s() - warned_at > 10.0:
                warned_at = node.now_s()
                print(
                    "WARNING: Nav2 still holds a goal — holding zero velocity "
                    "and retrying the cancel",
                    flush=True,
                )
            handle.cancel_goal_async()
            hold = node.now_s() + 1.0
            while node.now_s() < hold:
                node.publish(0.0, 0.0)
                rclpy.spin_once(node, timeout_sec=0.1)
        if node.orphan is not None:
            print(
                "WARNING: exiting with a possibly-live Nav2 goal — the mux "
                "will hand it the wheels; stop the stack (`pixi run kill`) "
                "before leaving the robot unattended",
                flush=True,
            )
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
