#!/usr/bin/env python3
"""Headless autonomous exploration for building a sim map.

Drives the robot to cover an unmapped world so slam_toolbox observes every wall
square-on, then exits. Greedy frontier exploration (Yamauchi) with Nav2 as the
sole driver and a next-best-view observation spin at each goal:

* detect — frontier cells (mapped-free 4-adjacent to unknown) off the live
  ``/map``, clustered into connected components by BFS. Each cluster is one
  unexplored opening; its centroid and cell count drive selection.
* select — the nearest cluster above a minimum size (greedy-nearest is
  provably complete and minimises travel). Tiny speckle clusters and a
  blacklist of unreachable points are filtered out.
* observe — Nav2 ``navigate_to_pose`` drives to a goal backed off the frontier
  into verified free space and *oriented to face the unknown*, so the robot
  arrives looking into the new region. On arrival it spins in place a full turn
  so slam sees the surrounding walls square-on — this is what fills the
  grazing-angle "fans" a corridor-only pass leaves beside every doorway.

Ends when no reachable frontier cluster remains (covered) or the budget
elapses. Gates on sim time (/clock), not wall time.

Run inside the mapping sim (sim_launch.py mode:=mapping is already up):
    python explore.py [--budget SECONDS]
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
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

SEED_SPIN = 7.0  # sim s of in-place rotation to bootstrap the first frontiers
SPIN_RATE = 1.0  # observation-spin yaw rate (rad/s), under the 1.87 angular cap
BACKOFF = 0.8  # aim the Nav2 goal at least this far back from the frontier, into
# known free space, so the planner can actually reach it (a frontier cell sits on
# the unknown boundary, inside costmap inflation, and goals there never arrive)
MAX_BACKOFF = 1.3  # ...but no further than this from the frontier, so the goal
# stays *near* the opening and the robot drives up to it rather than backing all
# the way to its own position (which would make progress stall)
MIN_CLUSTER = 6  # ignore frontier clusters smaller than this many cells (noise)
CLEANUP_CLUSTER = 3  # ...but in the cleanup sweep chase smaller ones too
OBSERVE_R = 1.2  # ring radius (m) for viewpoint sampling when the straight
# back-off collapses onto the robot — used to circle a free-standing obstacle
COLLAPSE_MIN = 0.6  # a back-off goal closer than this to the robot is "collapsed"
# (no travel, so an occluding obstacle would never be circled); trigger the ring
CLEANUP_BUDGET = 240.0  # sim s cap on the post-coverage cleanup sweep
MIN_PROGRESS = 0.35  # if a nav+spin cycle moves the robot less than this, the
# frontier is occluded/unreachable from here (the goal collapsed onto the robot);
# blacklist it so greedy-nearest advances to the next real opening
BL_RADIUS = 1.5  # blacklist this radius (m) around an unreachable frontier
NAV_TIMEOUT = 90.0  # hard cap (sim s) on a single frontier goal
STUCK_WINDOW = 25.0  # cancel a goal early if the robot hasn't moved STUCK_MOVE in
STUCK_MOVE = 0.3  # this many sim s — catches oscillation at a narrow doorway fast
# without abandoning legitimately-long inter-region drives (which keep progressing)
CLEAR_CELLS = 3  # goal cell must have this radius (cells) of free space around it
DONE_RETRIES = 3  # consecutive empty frontier scans before declaring covered
MAX_RETRIES = 3  # times to clear the blacklist and re-attempt abandoned frontiers


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Explorer(Node):
    def __init__(self):
        super().__init__("sim_explorer")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.grid = None
        self.cleanup = False  # cleanup sweep engages viewpoint-ring circling
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)
        self.cmd_pub = self.create_publisher(
            TwistStamped, "/diff_drive_controller/cmd_vel", 10
        )
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def on_map(self, msg):
        self.grid = msg

    def sim_now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def robot_xy(self):
        """(x, y) of base_link in the map frame, or None."""
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            return tf.transform.translation.x, tf.transform.translation.y
        except tf2_ros.TransformException:
            return None

    def publish(self, vx, wz):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.twist.linear.x = vx
        m.twist.angular.z = wz
        self.cmd_pub.publish(m)

    def spin_in_place(self, seconds):
        """Rotate in place for `seconds` sim-time, spinning ROS to keep /map,
        TF and the clock live. Used to seed the map and to observe on arrival."""
        start = self.sim_now()
        while self.sim_now() - start < seconds:
            self.publish(0.0, SPIN_RATE)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.publish(0.0, 0.0)

    def grid_array(self):
        g = self.grid
        w, h = g.info.width, g.info.height
        return np.array(g.data, dtype=np.int16).reshape(h, w)

    def clusters(self, min_size=MIN_CLUSTER):
        """Frontier clusters as (centroid_x, centroid_y, size). A frontier cell
        is mapped-free 4-adjacent to unknown; clusters are 8-connected
        components of those cells, found by BFS."""
        g = self.grid
        if g is None:
            return []
        a = self.grid_array()
        free = a == 0
        unk = a == -1
        nbr = np.zeros_like(unk)
        nbr[1:, :] |= unk[:-1, :]
        nbr[:-1, :] |= unk[1:, :]
        nbr[:, 1:] |= unk[:, :-1]
        nbr[:, :-1] |= unk[:, 1:]
        front = free & nbr
        ys, xs = np.where(front)
        cellset = set(zip(ys.tolist(), xs.tolist()))
        res = g.info.resolution
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        out = []
        seen = set()
        for cell in cellset:
            if cell in seen:
                continue
            comp = []
            q = deque([cell])
            seen.add(cell)
            while q:
                cy, cx = q.popleft()
                comp.append((cy, cx))
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        n = (cy + dy, cx + dx)
                        if n in cellset and n not in seen:
                            seen.add(n)
                            q.append(n)
            if len(comp) < min_size:
                continue
            arr = np.array(comp)  # (n, 2) as (row, col)
            my, mx = arr[:, 0].mean(), arr[:, 1].mean()
            out.append((ox + (mx + 0.5) * res, oy + (my + 0.5) * res, len(comp)))
        return out

    def _clear(self, a, info, gx, gy):
        """True if world point (gx, gy) is a free cell with CLEAR_CELLS of free
        space around it (so Nav2 accepts it, not lost in costmap inflation)."""
        res = info.resolution
        col = int((gx - info.origin.position.x) / res)
        row = int((gy - info.origin.position.y) / res)
        h, w = a.shape
        if not (CLEAR_CELLS <= row < h - CLEAR_CELLS):
            return False
        if not (CLEAR_CELLS <= col < w - CLEAR_CELLS):
            return False
        patch = a[
            row - CLEAR_CELLS : row + CLEAR_CELLS + 1,
            col - CLEAR_CELLS : col + CLEAR_CELLS + 1,
        ]
        return bool(np.all(patch == 0))

    def free_goal(self, cx, cy, here):
        """A reachable goal for the frontier at (cx, cy), oriented to face it.

        Primary: step back from the frontier toward the robot (BACKOFF..MAX_BACKOFF
        metres) to the first clear cell — near the opening but out of inflation.

        Fallback (cleanup sweep only): if that clear cell lands on the robot
        (COLLAPSE_MIN), the frontier is a shadow the robot can't see from where it
        stands (e.g. behind a free-standing obstacle). Sample viewpoints on a ring
        around the frontier and take the reachable one *farthest* from the robot,
        so it circles round to observe. This runs only after coverage is complete
        (self.cleanup) — during the main sweep such collapses are left for greedy
        exploration to reach from elsewhere, which keeps the robot from being
        diverted onto long, often-unreachable detours mid-exploration. Returns
        (gx, gy, yaw) or None if nothing clear/reachable is found."""
        a = self.grid_array()
        info = self.grid.info
        res = info.resolution
        vx, vy = here[0] - cx, here[1] - cy
        norm = math.hypot(vx, vy) or 1.0
        ux, uy = vx / norm, vy / norm
        lo = int(BACKOFF / res)
        hi = max(lo, int(min(norm, MAX_BACKOFF) / res))
        for s in range(lo, hi + 1):
            gx, gy = cx + ux * s * res, cy + uy * s * res
            if self._clear(a, info, gx, gy):
                if (
                    not self.cleanup
                    or math.hypot(gx - here[0], gy - here[1]) >= COLLAPSE_MIN
                ):
                    return gx, gy, math.atan2(cy - gy, cx - gx)
                break  # cleanup: collapsed onto the robot — circle it instead
        if not self.cleanup:
            return None
        best, best_d = None, COLLAPSE_MIN
        for k in range(16):
            ang = 2 * math.pi * k / 16
            gx, gy = cx + OBSERVE_R * math.cos(ang), cy + OBSERVE_R * math.sin(ang)
            if not self._clear(a, info, gx, gy):
                continue
            d = math.hypot(gx - here[0], gy - here[1])
            if d > best_d:
                best, best_d = (gx, gy, math.atan2(cy - gy, cx - gx)), d
        return best

    def navigate_to(self, x, y, yaw):
        """Drive to (x, y, yaw) via Nav2. Returns 'ok'/'aborted'/'rejected'/'timeout'."""
        if not self.nav.wait_for_server(timeout_sec=5):
            return "rejected"
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        send = self.nav.send_goal_async(goal)
        t0 = self.sim_now()
        while not send.done() and self.sim_now() - t0 < 10:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not send.done() or send.result() is None or not send.result().accepted:
            return "rejected"
        handle = send.result()
        result_fut = handle.get_result_async()
        here = self.robot_xy()
        moved_at = t0
        while self.sim_now() - t0 < NAV_TIMEOUT:
            rclpy.spin_once(self, timeout_sec=0.1)
            if result_fut.done():
                ok = result_fut.result().status == GoalStatus.STATUS_SUCCEEDED
                return "ok" if ok else "aborted"
            now = self.robot_xy()
            if now is not None and here is not None:
                if math.hypot(now[0] - here[0], now[1] - here[1]) > STUCK_MOVE:
                    here, moved_at = now, self.sim_now()
                elif self.sim_now() - moved_at > STUCK_WINDOW:
                    handle.cancel_goal_async()
                    rclpy.spin_once(self, timeout_sec=0.5)
                    return "stuck"
        handle.cancel_goal_async()
        rclpy.spin_once(self, timeout_sec=0.5)
        return "timeout"


def pick(clusters, here, blacklist):
    """Nearest cluster centroid (greedy-nearest) not near a blacklisted point."""
    best = None
    best_d = float("inf")
    for cx, cy, size in clusters:
        if any(math.hypot(cx - bx, cy - by) < BL_RADIUS for bx, by in blacklist):
            continue
        d = math.hypot(cx - here[0], cy - here[1])
        if d < best_d:
            best_d, best = d, (cx, cy, size)
    return best, best_d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--budget", type=float, default=900.0, help="sim-time seconds")
    ap.add_argument(
        "--min-time",
        type=float,
        default=60.0,
        help="explore at least this long (sim s)",
    )
    args = ap.parse_args()

    rclpy.init()
    node = Explorer()

    deadline = time.monotonic() + 40
    while node.grid is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.grid is None:
        print("FAIL: no /map received", file=sys.stderr)
        return 1

    # Require a working map->base_link TF before driving. If a stale node from a
    # previous run poisoned the TF tree with future-stamped odom transforms,
    # robot_xy() never resolves — fail loudly here rather than spin uselessly for
    # the whole budget producing an empty map.
    deadline = time.monotonic() + 30
    while node.robot_xy() is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.robot_xy() is None:
        print("FAIL: no map->base_link TF (poisoned TF tree?)", file=sys.stderr)
        return 1

    print("seeding map with an in-place spin...", flush=True)
    node.spin_in_place(SEED_SPIN)

    start = node.sim_now()
    blacklist = []
    empty_scans = 0
    retries = 0
    cleanup_deadline = 0.0
    lost_tf = 0

    while node.sim_now() - start < args.budget:
        rclpy.spin_once(node, timeout_sec=0.05)
        t = node.sim_now()
        if node.cleanup and t > cleanup_deadline:
            print(f"[{t - start:6.1f}s] cleanup budget spent — done", flush=True)
            break
        here = node.robot_xy()
        if here is None:
            lost_tf += 1
            if lost_tf > 400:  # ~persistent TF loss, not a transient miss
                print("FAIL: lost map->base_link TF mid-run", file=sys.stderr)
                return 1
            continue
        lost_tf = 0
        clusters = node.clusters(CLEANUP_CLUSTER if node.cleanup else MIN_CLUSTER)
        target, dist = pick(clusters, here, blacklist)
        print(
            f"[{t - start:6.1f}s] {'cleanup ' if node.cleanup else ''}"
            f"clusters {len(clusters)} bl {len(blacklist)} "
            f"pose ({here[0]:+.1f},{here[1]:+.1f}) "
            f"{('-> nearest %.1f m' % dist) if target else '-> none'}",
            flush=True,
        )

        if target is None:
            # Frontiers remain but all are blacklisted: give the abandoned
            # regions a second chance from wherever the robot is now (a goal
            # that timed out from across the map may plan fine from close by).
            if clusters and blacklist and retries < MAX_RETRIES:
                retries += 1
                print(
                    f"[{t - start:6.1f}s] all {len(clusters)} frontiers blacklisted "
                    f"— clearing blacklist (retry {retries}/{MAX_RETRIES})",
                    flush=True,
                )
                blacklist = []
                continue
            empty_scans += 1
            if empty_scans >= DONE_RETRIES and t - start > args.min_time:
                if not node.cleanup:
                    # Main coverage is complete. Engage the cleanup sweep: chase
                    # smaller residual frontiers and circle occluded ones (e.g.
                    # lidar shadows behind free-standing obstacles) that greedy
                    # exploration left. Bounded so it can't run away.
                    print(
                        f"[{t - start:6.1f}s] coverage complete — cleanup sweep",
                        flush=True,
                    )
                    node.cleanup = True
                    cleanup_deadline = t + CLEANUP_BUDGET
                    blacklist = []
                    empty_scans = 0
                    continue
                print(
                    f"[{t - start:6.1f}s] no reachable frontiers — covered", flush=True
                )
                break
            node.spin_in_place(2.0)  # nudge the map; frontiers may reappear
            continue
        empty_scans = 0

        cx, cy, size = target
        goal = node.free_goal(cx, cy, here)
        if goal is None:
            blacklist.append((cx, cy))
            continue
        gx, gy, yaw = goal
        print(
            f"[{t - start:6.1f}s] -> frontier ({cx:+.1f},{cy:+.1f}) size {size}, "
            f"goal ({gx:+.1f},{gy:+.1f})",
            flush=True,
        )
        res = node.navigate_to(gx, gy, yaw)
        print(f"[{node.sim_now() - start:6.1f}s]   nav {res}", flush=True)
        if res == "ok":
            node.spin_in_place(2 * math.pi / SPIN_RATE)  # observe square-on
        else:
            blacklist.append((cx, cy))
            continue
        # If we didn't actually travel, this frontier is occluded/unreachable
        # from here — blacklist it so we don't pick it again next scan.
        after = node.robot_xy()
        if (
            after is not None
            and math.hypot(after[0] - here[0], after[1] - here[1]) < MIN_PROGRESS
        ):
            blacklist.append((cx, cy))

    node.publish(0.0, 0.0)
    print(f"exploration done after {node.sim_now() - start:.1f}s sim time", flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
