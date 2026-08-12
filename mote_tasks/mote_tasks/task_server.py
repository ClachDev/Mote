"""Hosts the mission behaviour trees and accepts task commands.

Commands arrive on ``task/command`` (std_msgs/String):

    fetch <target> <drop_zone>   fetch a zone/object to a drop zone
    goto <zone>                  drive to a named zone ("go to the kitchen")

For ``fetch``, ``target`` is a zone name when it matches one, otherwise it is
an open-vocabulary object label handed to the detector (underscores become
spaces, so ``fetch red_box dropoff`` looks for "red box"). For ``goto``, the
argument names a zone. Zone names come from the zones YAML (parameter
``zones_file``, falling back to the committed config/zones.default.yaml).
Outcomes are published on ``task/status`` as accepted/rejected/succeeded/failed
strings.

The active tree is ticked at ``tick_period`` while a task is running and at the
slower ``idle_tick_period`` between them, since a tree waiting for work has
nothing to advance. Command acceptance is unaffected either way: it happens in
the subscription callback, not on a tick.
"""

import os

import py_trees
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String

from mote_tasks import zones
from mote_tasks.trees import fetch, goto
from mote_tasks.trees.common import TASK_KEY


class TaskServer(Node):
    def __init__(self, **node_kwargs):
        super().__init__("task_server", **node_kwargs)
        zones_file = self.declare_parameter("zones_file", "").value
        tick_period = self.declare_parameter("tick_period", 0.1).value
        idle_tick_period = self.declare_parameter("idle_tick_period", 1.0).value
        pick_duration = self.declare_parameter("pick_duration", 3.0).value
        place_duration = self.declare_parameter("place_duration", 3.0).value

        if not zones_file:
            zones_file = os.path.join(
                get_package_share_directory("mote_tasks"),
                "config",
                "zones.default.yaml",
            )
        self.zones = zones.load_zones(zones_file)
        self.get_logger().info(f"Zones {sorted(self.zones)} from {zones_file}")

        self.fetch_tree = fetch.create_fetch_tree(pick_duration, place_duration)
        self.goto_tree = goto.create_goto_tree()
        for tree in (self.fetch_tree, self.goto_tree):
            tree.setup(node=self)
        # The active tree is the one tick() drives; the others sit idle. It
        # starts on fetch, which just idles in WaitForTask until a command sets
        # the active tree and the task key together.
        self.active = self.fetch_tree

        self.blackboard = py_trees.blackboard.Client(name="task_server")
        for key in (
            TASK_KEY,
            fetch.OBJECT_POSE_KEY,
            fetch.OBJECT_LABEL_KEY,
            fetch.DROP_POSE_KEY,
            goto.GOTO_POSE_KEY,
        ):
            self.blackboard.register_key(key, access=py_trees.common.Access.WRITE)
        self.blackboard.set(TASK_KEY, None)

        self.status_pub = self.create_publisher(String, "task/status", 1)
        self.create_subscription(String, "task/command", self.on_command, 1)
        self.last_tip = None
        # Two rates, because a tree between missions has nothing to advance: it
        # idles in WaitForTask, whose whole update() is one blackboard read, and
        # ticking that at the mission rate is ten wake-ups a second to learn
        # nothing has changed. An idle rate faster than the mission rate would
        # be a contradiction, so it is floored at it.
        self.tick_period = tick_period
        self.idle_tick_period = max(idle_tick_period, tick_period)
        self.ticking_fast = False
        self.tick_timer = self.create_timer(self.idle_tick_period, self.tick)

    def _set_tick_rate(self, active: bool):
        """Tick at the mission rate while a task runs, slowly between them.

        The timer is *reset* as well as re-periodded, because setting a period
        does not move the expiry already pending — measured: a 5 s timer 0.3 s
        into its period still reports 4.7 s to go after its period is set to
        0.05 s, and 0.05 s after ``reset()``. Without the reset, a command
        accepted just after an idle tick would wait out the rest of the idle
        period before the tree ticked at all. Accepting a command is already
        independent of the tick (``on_command`` publishes the outcome itself);
        what this protects is the first tick of the accepted tree, which is
        what sends the Nav2 goal and starts the robot driving.
        """
        if active == self.ticking_fast:
            return
        self.ticking_fast = active
        period = self.tick_period if active else self.idle_tick_period
        self.tick_timer.timer_period_ns = int(period * 1e9)
        self.tick_timer.reset()

    def publish_status(self, text: str):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    def _accept_fetch(self, words):
        object_pose, object_label, drop_pose = fetch.parse_command(self.zones, words)
        if object_pose is not None:
            self.blackboard.set(fetch.OBJECT_POSE_KEY, object_pose)
        else:
            self.blackboard.unset(fetch.OBJECT_POSE_KEY)
        self.blackboard.set(fetch.OBJECT_LABEL_KEY, object_label)
        self.blackboard.set(fetch.DROP_POSE_KEY, drop_pose)
        return self.fetch_tree

    def _accept_goto(self, words):
        pose = goto.parse_command(self.zones, words)
        self.blackboard.set(goto.GOTO_POSE_KEY, pose)
        return self.goto_tree

    def on_command(self, msg: String):
        if self.blackboard.get(TASK_KEY):
            self.publish_status(f"rejected: busy with '{self.blackboard.task}'")
            return
        words = msg.data.split()
        command = words[0] if words else ""
        dispatch = {fetch.COMMAND: self._accept_fetch, goto.COMMAND: self._accept_goto}
        if command not in dispatch:
            self.publish_status(
                f"rejected: '{msg.data}' (unknown command, "
                f"have: {', '.join(sorted(dispatch))})"
            )
            return
        try:
            self.active = dispatch[command](words)
        except ValueError as e:
            self.publish_status(f"rejected: '{msg.data}' ({e})")
            return
        self.blackboard.set(TASK_KEY, msg.data)
        self._set_tick_rate(active=True)
        self.publish_status(f"accepted: {msg.data}")

    def tick(self):
        self.active.tick()
        root = self.active.root
        tip = root.tip()
        label = f"{tip.name} [{tip.status.name}]" if tip else "-"
        if label != self.last_tip:
            self.get_logger().info(f"tree: {label}")
            self.last_tip = label
        if root.status == py_trees.common.Status.SUCCESS:
            self.publish_status(f"succeeded: {self.blackboard.task}")
            self.blackboard.set(TASK_KEY, None)
            self._set_tick_rate(active=False)
        elif root.status == py_trees.common.Status.FAILURE:
            self.publish_status(f"failed: {self.blackboard.task} (at {label})")
            self.blackboard.set(TASK_KEY, None)
            self._set_tick_rate(active=False)


def main():
    rclpy.init()
    node = TaskServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
