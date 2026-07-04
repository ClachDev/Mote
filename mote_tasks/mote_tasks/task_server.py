"""Hosts the fetch behaviour tree and accepts task commands.

Commands arrive on ``task/command`` (std_msgs/String):

    fetch <object_zone> <drop_zone>

Zone names come from the zones YAML (parameter ``zones_file``, falling back
to the committed config/zones.default.yaml). Outcomes are published on
``task/status`` as accepted/rejected/succeeded/failed strings.
"""

import os

import py_trees
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String

from mote_tasks import zones
from mote_tasks.trees import fetch


class TaskServer(Node):
    def __init__(self, **node_kwargs):
        super().__init__("task_server", **node_kwargs)
        zones_file = self.declare_parameter("zones_file", "").value
        tick_period = self.declare_parameter("tick_period", 0.1).value
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

        self.tree = fetch.create_fetch_tree(pick_duration, place_duration)
        self.tree.setup(node=self)
        self.blackboard = py_trees.blackboard.Client(name="task_server")
        for key in (fetch.TASK_KEY, fetch.OBJECT_POSE_KEY, fetch.DROP_POSE_KEY):
            self.blackboard.register_key(key, access=py_trees.common.Access.WRITE)
        self.blackboard.set(fetch.TASK_KEY, None)

        self.status_pub = self.create_publisher(String, "task/status", 1)
        self.create_subscription(String, "task/command", self.on_command, 1)
        self.last_tip = None
        self.create_timer(tick_period, self.tick)

    def publish_status(self, text: str):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    def on_command(self, msg: String):
        words = msg.data.split()
        if self.blackboard.get(fetch.TASK_KEY):
            self.publish_status(f"rejected: busy with '{self.blackboard.task}'")
            return
        if len(words) != 3 or words[0] != "fetch":
            self.publish_status(
                f"rejected: '{msg.data}' (expected: fetch <object_zone> <drop_zone>)"
            )
            return
        unknown = [w for w in words[1:] if w not in self.zones]
        if unknown:
            self.publish_status(
                f"rejected: unknown zone(s) {unknown}, have {sorted(self.zones)}"
            )
            return
        self.blackboard.set(fetch.OBJECT_POSE_KEY, self.zones[words[1]])
        self.blackboard.set(fetch.DROP_POSE_KEY, self.zones[words[2]])
        self.blackboard.set(fetch.TASK_KEY, msg.data)
        self.publish_status(f"accepted: {msg.data}")

    def tick(self):
        self.tree.tick()
        root = self.tree.root
        tip = root.tip()
        label = f"{tip.name} [{tip.status.name}]" if tip else "-"
        if label != self.last_tip:
            self.get_logger().info(f"tree: {label}")
            self.last_tip = label
        if root.status == py_trees.common.Status.SUCCESS:
            self.publish_status(f"succeeded: {self.blackboard.task}")
            self.blackboard.set(fetch.TASK_KEY, None)
        elif root.status == py_trees.common.Status.FAILURE:
            self.publish_status(f"failed: {self.blackboard.task} (at {label})")
            self.blackboard.set(fetch.TASK_KEY, None)


def main():
    rclpy.init()
    node = TaskServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
