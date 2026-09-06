"""Bind a zone by driving to it: capture the robot's current map-frame pose
into the active site's floor (legacy ~/.mote/zones.yaml if no site).

    ros2 run mote_tasks save_zone <name> [--radius R] [--note TEXT]
                                        [--no-navigable] [base_frame]

One of three ways geometry reaches a floor — ``segment-map`` reads outlines off
a saved map, and the fleet dashboard's zone editor places them on a candidate
revision — and the only one that has to happen on the robot. What it buys for
that is a pose the robot demonstrably reached, at the heading it arrived on:
this is the command for a place a mission must approach from a particular side,
where the other two put a coordinate on the map and leave the rest to the
planner. The binding is anchored ``taught``, which is what tells a reader later
that re-mapping the floor invalidates it.

``--radius`` (metres) gives the zone a circular area footprint, so it answers
"am I in it" and reads as a room rather than a bare waypoint; omit it for a
plain navigation target. Re-capturing an existing name replaces its pose but
keeps its footprint, which may be a polygon outline this command cannot capture
(see mote_tasks.zones).

The name is the whole of what a place is called — quote it if it has spaces —
and ``--note`` is free text for what the name cannot say: "stationery lives
here, not in the office". Names and notes are the half of a zone that travels
on its own: the fleet serves them to a dispatcher at ``/v1/zones`` and never
the pose, because the pose is only true against the map frame beside it.
``--no-navigable`` marks a place a robot must not be sent to. Re-capturing
keeps whatever the zone already carries — a better coordinate is not a rename.
"""

import sys

import rclpy
import tf2_ros
from mote_bringup import bundle, sites
from rclpy.node import Node
from rclpy.time import Time

from mote_tasks.zones import append_zone, yaw_from_quaternion

LOOKUP_TIMEOUT = 10.0


def main():
    radius = None
    note = None
    navigable = None
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
        elif arg == "--note":
            i += 1
            if i >= len(argv):
                sys.exit("--note needs a value")
            note = argv[i]
        elif arg.startswith("--note="):
            note = arg.split("=", 1)[1]
        elif arg == "--no-navigable":
            navigable = False
        elif arg == "--navigable":
            navigable = True
        elif not arg.startswith("-"):
            positional.append(arg)
        i += 1
    if not positional:
        sys.exit(
            "usage: save_zone <name> [--radius R] [--note TEXT] "
            "[--no-navigable] [base_frame]"
        )
    name = positional[0]
    base_frame = positional[1] if len(positional) > 1 else "base_link"
    # Checked before ROS starts: the operator is standing at the robot having
    # just driven it somewhere, and finding out about a typo after the ten
    # second transform wait means driving there again.
    if not bundle.ZONE_NAME_RE.match(name):
        sys.exit(f"'{name}' is not a name anyone can type")

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
    site, floor = sites.active() or ("", "")
    replaced = append_zone(
        path,
        name,
        t.x,
        t.y,
        yaw,
        radius,
        note,
        navigable,
        site=site,
        floor=floor,
    )
    verb = "replaced" if replaced else "added"
    extra = f" radius={radius:.3f}" if radius is not None else ""
    extra += f" note={note!r}" if note is not None else ""
    extra += " navigable=false" if navigable is False else ""
    print(
        f"{verb} zone '{name}': x={t.x:.3f} y={t.y:.3f} yaw={yaw:.3f}{extra} in {path}"
    )
    node.destroy_node()


if __name__ == "__main__":
    main()
