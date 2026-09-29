#!/usr/bin/env python3
"""A promoted map loads into a running sim Nav2, keeps the pose, and drives.

Run by run_map_reload.sh against ``sim_launch.py mode:=nav`` on a copy of the
sim MOTE_HOME whose mote_world floor holds two revisions. It plays the fleet
agent's part — flip the floor's ``map`` symlink, publish ``map/installed`` —
and then asserts, with no restart:

1. ``map_reloader`` reports the new revision on ``map/serving``;
2. ``/map`` republishes with the new revision's content;
3. the ``map``->``base_link`` pose after the load is within tolerance of the
   one before it (AMCL re-initialises on a new map; the reloader carries it);
4. a ``goto`` dispatched afterwards succeeds.

Exits 0 on PASS, 1 on any failure (printed as "FAIL: ...").
"""

import argparse
import hashlib
import json
import math
import sys
import time

import rclpy
import tf2_ros
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from mote_bringup import map_reload, sites
from mote_bringup.spec import mission

LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class Verifier(Node):
    def __init__(self):
        super().__init__("map_reload_verifier")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.maps = []
        self.serving = None
        self.capabilities = None
        self.statuses = []
        self.create_subscription(OccupancyGrid, "/map", self._on_map, LATCHED)
        self.create_subscription(
            String,
            map_reload.SERVING_TOPIC,
            lambda m: setattr(self, "serving", map_reload.parse(m.data)),
            LATCHED,
        )
        self.create_subscription(
            String,
            "task/capabilities",
            lambda m: setattr(self, "capabilities", json.loads(m.data)),
            LATCHED,
        )
        self.create_subscription(
            String,
            "task/status",
            lambda m: self.statuses.append(json.loads(m.data)),
            10,
        )
        self.command_pub = self.create_publisher(String, "task/command", 1)
        self.installed_pub = self.create_publisher(
            String, map_reload.INSTALLED_TOPIC, LATCHED
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _on_map(self, msg):
        digest = hashlib.sha256(bytes(bytearray(msg.data))).hexdigest()[:12]
        self.maps.append((digest, msg))

    def spin_until(self, condition, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if condition():
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return condition()

    def active(self, node_name: str) -> bool:
        """Whether a Nav2 lifecycle node has reached ACTIVE."""
        client = self.create_client(GetState, f"/{node_name}/get_state")
        try:
            if not client.wait_for_service(timeout_sec=1.0):
                return False
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            result = future.result()
            return (
                bool(result) and result.current_state.id == State.PRIMARY_STATE_ACTIVE
            )
        finally:
            self.destroy_client(client)

    def pose(self):
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
        return t.x, t.y, yaw

    def goto(self, target, timeout):
        platform = self.capabilities["platform_id"]
        command = mission.command(platform, "goto", {"target": target})
        self.command_pub.publish(String(data=json.dumps(command)))
        mine = lambda: [s for s in self.statuses if s["id"] == command["id"]]  # noqa: E731
        ok = self.spin_until(lambda: any(s["terminal"] for s in mine()), timeout)
        states = [s["state"] for s in mine()]
        return ok and states[-1] == mission.SUCCEEDED, states, mine()


def fail(message):
    print(f"FAIL: {message}")
    sys.exit(1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--site", default="mote_world")
    parser.add_argument("--floor", default="ground")
    parser.add_argument("--new", required=True, help="the revision to promote")
    parser.add_argument("--before", default="kitchen", help="zone to drive to first")
    parser.add_argument("--after", default="pickup", help="zone to drive to after")
    parser.add_argument("--tolerance", type=float, default=0.25)
    parser.add_argument("--yaw-tolerance", type=float, default=0.2)
    args = parser.parse_args()

    rclpy.init()
    node = Verifier()
    floor = sites.floor_dir(args.site, args.floor)
    old = sites.current_revision(floor)

    if not node.spin_until(
        lambda: node.serving and node.serving["revision"] == old, 120
    ):
        fail(f"map_reloader never reported serving {old}: {node.serving}")
    print(f"STEP1 OK: map_reloader serving {old}")
    if not node.spin_until(lambda: node.maps and node.capabilities, 60):
        fail("no /map or no task capabilities")
    if not node.spin_until(
        lambda: node.command_pub.get_subscription_count() > 0 and node.pose(), 60
    ):
        fail("task_server not subscribed, or no map->base_link")
    for server in ("amcl", "bt_navigator"):
        if not node.spin_until(lambda: node.active(server), 120):
            fail(f"{server} never became active")

    ok, states, _ = node.goto(args.before, 180)
    if not ok:
        fail(f"goto {args.before} before the promotion: {states}")
    time.sleep(1.0)
    node.spin_until(lambda: False, 2.0)
    before = node.pose()
    old_digest = node.maps[-1][0]
    print(
        f"STEP2 OK: goto {args.before} succeeded; pose "
        f"({before[0]:.3f}, {before[1]:.3f}, {math.degrees(before[2]):.1f} deg); "
        f"/map {old_digest}"
    )

    # What the agent's pull does once the bytes are staged.
    sites._publish_revision(floor, args.new)
    node.installed_pub.publish(
        String(data=map_reload.installed(args.site, args.floor, args.new))
    )
    started = time.monotonic()
    if not node.spin_until(
        lambda: node.serving and node.serving["revision"] == args.new, 60
    ):
        fail(f"map_reloader did not load {args.new}: {node.serving}")
    if not node.spin_until(lambda: node.maps[-1][0] != old_digest, 30):
        fail("/map did not change after the load")
    loaded = time.monotonic() - started
    print(
        f"STEP3 OK: serving {args.new} and /map now {node.maps[-1][0]} "
        f"({loaded:.2f}s after map/installed, no restart)"
    )

    node.spin_until(lambda: False, 5.0)
    after = node.pose()
    if after is None:
        fail("no map->base_link after the load")
    d = math.hypot(after[0] - before[0], after[1] - before[1])
    dyaw = abs(
        math.atan2(math.sin(after[2] - before[2]), math.cos(after[2] - before[2]))
    )
    print(
        f"       pose after ({after[0]:.3f}, {after[1]:.3f}, "
        f"{math.degrees(after[2]):.1f} deg): off by {d:.3f} m, "
        f"{math.degrees(dyaw):.1f} deg"
    )
    if d > args.tolerance or dyaw > args.yaw_tolerance:
        fail(f"pose estimate did not survive the load ({d:.3f} m, {dyaw:.3f} rad)")
    print("STEP4 OK: pose estimate carried across the load")

    ok, states, _ = node.goto(args.after, 180)
    if not ok:
        fail(f"goto {args.after} after the promotion: {states}")
    print(f"STEP5 OK: goto {args.after} succeeded on {args.new}")
    print("MAP RELOAD TEST PASS")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
