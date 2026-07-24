"""Teach a zone by driving to it: capture the robot's current map-frame pose
into the active site's zones.yaml (legacy ~/.mote/zones.yaml if no site).

    ros2 run mote_tasks save_zone <name> [--radius R] [base_frame]

Poses taught this way are reachable by construction. ``--radius`` (metres)
gives the zone a circular area footprint, so it answers "am I in it" and reads
as a room rather than a bare waypoint; omit it for a plain navigation target.
Re-teaching an existing name replaces it.
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
    radius = None
    positional = []
    argv = sys.argv[1:]
    i = 0
    while i < len(argv):
        arg = argv[i]
        if arg == "--radius":
            i += 1
            if i >= len(argv):
                sys.exit("--radius needs a value")
            radius = float(argv[i])
        elif arg.startswith("--radius="):
            radius = float(arg.split("=", 1)[1])
        elif not arg.startswith("-"):
            positional.append(arg)
        i += 1
    if not positional:
        sys.exit("usage: save_zone <name> [--radius R] [base_frame]")
    name = positional[0]
    base_frame = positional[1] if len(positional) > 1 else "base_link"

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
    replaced = append_zone(path, name, t.x, t.y, yaw, radius)
    verb = "replaced" if replaced else "added"
    extra = f" radius={radius:.3f}" if radius is not None else ""
    print(
        f"{verb} zone '{name}': x={t.x:.3f} y={t.y:.3f} yaw={yaw:.3f}{extra} in {path}"
    )
    node.destroy_node()


if __name__ == "__main__":
    main()
