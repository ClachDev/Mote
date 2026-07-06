"""Teach a zone by driving to it: capture the robot's current map-frame pose
into the active site's zones.yaml (legacy ~/.mote/zones.yaml if no site).

    ros2 run mote_tasks save_zone <name> [base_frame]

Poses taught this way are reachable by construction. Re-teaching an existing
name replaces it.
"""

import sys

import rclpy
import tf2_ros
from mote_bringup import sites
from rclpy.node import Node
from rclpy.time import Time

from mote_tasks.zones import append_zone, yaw_from_quaternion

LOOKUP_TIMEOUT = 10.0


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    if not args:
        sys.exit("usage: save_zone <name> [base_frame]")
    name = args[0]
    base_frame = args[1] if len(args) > 1 else "base_link"

    rclpy.init()
    node = Node("save_zone")
    buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(buffer, node)

    deadline = node.get_clock().now().nanoseconds + int(LOOKUP_TIMEOUT * 1e9)
    while not buffer.can_transform("map", base_frame, Time()):
        if node.get_clock().now().nanoseconds > deadline:
            node.destroy_node()
            sys.exit(f"no map->{base_frame} transform (is localisation running?)")
        rclpy.spin_once(node, timeout_sec=0.1)

    tf = buffer.lookup_transform("map", base_frame, Time())
    t, q = tf.transform.translation, tf.transform.rotation
    yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

    path = sites.zones_for_write()
    replaced = append_zone(path, name, t.x, t.y, yaw)
    verb = "replaced" if replaced else "added"
    print(f"{verb} zone '{name}': x={t.x:.3f} y={t.y:.3f} yaw={yaw:.3f} in {path}")
    node.destroy_node()


if __name__ == "__main__":
    main()
