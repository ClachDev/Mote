#!/usr/bin/env python3
"""Time how long a camera-only obstacle mark outlives the obstacle.

The local costmap's `camera_layer` is fed a cloud of *above-floor points only*
(`mote_perception/depth_obstacle_node` strips the floor to keep the stream off
the Wi-Fi), so nothing ever raytraces a departed obstacle away. It is a
`spatio_temporal_voxel_layer` instead, which expires a voxel `voxel_decay`
seconds after it was last marked — and several of its settings switch that off
without any error, one of them by re-marking the stale cloud forever.

This runs the layer for real and measures it: a standalone Nav2 costmap built
from the shipped `nav2_params.yaml` (this layer only, no lidar and no
inflation), fed a synthetic obstacle for a few seconds, then left alone. The
robot never moves, so a mark that goes can only have decayed.

    pixi run camera-decay-check

Exit 0 if the mark appeared and then cleared within a decay period of
`voxel_decay`, 1 otherwise. `mote_bringup/test/test_costmap_layers.py` is the
static half of this — cheap enough to run on every build, where this is not.
"""

import argparse
import os
import pathlib
import signal
import struct
import subprocess
import sys
import tempfile
import time

import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField

NAV2_PARAMS = (
    pathlib.Path(__file__).resolve().parents[1] / "config" / "nav2_params.yaml"
)

CLOUD_TOPIC = "/camera_obstacles"
# Well inside the layer's 0.25-1.2 m band, and above its 0.02 m floor deadband.
OBSTACLE = (0.6, 0.0, 0.10)
FEED_SECONDS = 6.0


def costmap_params(node_name):
    """A standalone-costmap params file carrying only the real camera layer."""
    local = yaml.safe_load(NAV2_PARAMS.read_text())["local_costmap"]["local_costmap"]
    shipped = local["ros__parameters"]
    return {
        node_name: {
            "ros__parameters": {
                key: shipped[key]
                for key in (
                    "update_frequency",
                    "publish_frequency",
                    "global_frame",
                    "robot_base_frame",
                    "rolling_window",
                    "width",
                    "height",
                    "resolution",
                    "robot_radius",
                    "always_send_full_costmap",
                )
            }
            | {"plugins": ["camera_layer"], "camera_layer": shipped["camera_layer"]}
        }
    }


def xyz_cloud(stamp, frame, points):
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name=axis, offset=4 * i, datatype=PointField.FLOAT32, count=1)
        for i, axis in enumerate(("x", "y", "z"))
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.is_dense = True
    msg.data = b"".join(struct.pack("<fff", *p) for p in points)
    return msg


class Probe(Node):
    """Publishes the obstacle and counts the lethal cells it produces."""

    def __init__(self, cloud_frame, costmap_topic):
        super().__init__("camera_layer_decay_probe")
        self.cloud_frame = cloud_frame
        self.pub = self.create_publisher(PointCloud2, CLOUD_TOPIC, 5)
        self.grid = None
        self.create_subscription(
            OccupancyGrid,
            costmap_topic,
            lambda msg: setattr(self, "grid", msg),
            QoSProfile(
                depth=1,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                reliability=QoSReliabilityPolicy.RELIABLE,
            ),
        )

    def publish_obstacle(self):
        x, y, z = OBSTACLE
        patch = [
            (x + dx * 0.02, y + dy * 0.02, z)
            for dx in range(-2, 3)
            for dy in range(-2, 3)
        ]
        self.pub.publish(
            xyz_cloud(self.get_clock().now().to_msg(), self.cloud_frame, patch)
        )

    def marked_cells(self):
        """Lethal cells within 0.1 m of the obstacle, or None before a grid."""
        grid = self.grid
        if grid is None:
            return None
        info, x, y = grid.info, OBSTACLE[0], OBSTACLE[1]
        offsets = [k * info.resolution for k in (-2, -1, 0, 1, 2)]
        marked = 0
        for wy in (y + d for d in offsets):
            for wx in (x + d for d in offsets):
                cx = int((wx - info.origin.position.x) / info.resolution)
                cy = int((wy - info.origin.position.y) / info.resolution)
                if 0 <= cx < info.width and 0 <= cy < info.height:
                    marked += grid.data[cy * info.width + cx] >= 99
        return marked


def spawn(command, log):
    return subprocess.Popen(
        command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True
    )


def static_tf(log, parent, child, extra=()):
    return spawn(
        ["ros2", "run", "tf2_ros", "static_transform_publisher", *extra]
        + ["--frame-id", parent, "--child-frame-id", child],
        log,
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--log",
        default=os.path.join(tempfile.gettempdir(), "camera_layer_decay.log"),
        help="where the costmap and TF process output goes",
    )
    args = ap.parse_args()

    node_name = "costmap"
    params = costmap_params(node_name)
    layer = params[node_name]["ros__parameters"]["camera_layer"]
    decay = layer.get("voxel_decay")
    if decay is None:
        print(
            "camera_layer has no voxel_decay: it is not a decaying layer, so "
            "there is nothing here to measure"
        )
        return 1
    # The costmap resolves the cloud through TF, so the harness has to stand in
    # for the robot: the frames the layer names, held still at the origin.
    global_frame = params[node_name]["ros__parameters"]["global_frame"]
    base_frame = params[node_name]["ros__parameters"]["robot_base_frame"]
    sensor_frame = layer["camera"].get("sensor_frame") or base_frame
    cloud_frame = "base_footprint"

    handle = open(args.log, "w")
    procs = [
        static_tf(handle, "map", global_frame),
        static_tf(handle, global_frame, base_frame),
        static_tf(handle, base_frame, cloud_frame),
        # roughly the real mount: 0.10 m up, forward, optical-frame axes
        static_tf(
            handle,
            base_frame,
            sensor_frame,
            ["--x", "0.05", "--z", "0.10", "--roll", "-1.5708", "--yaw", "-1.5708"],
        ),
    ]
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
        yaml.safe_dump(params, f)
        params_file = f.name
    procs.append(
        spawn(
            [
                "ros2",
                "run",
                "nav2_costmap_2d",
                "nav2_costmap_2d",
                "--ros-args",
                "--params-file",
                params_file,
            ],
            handle,
        )
    )

    try:
        time.sleep(4)
        for transition in ("configure", "activate"):
            subprocess.run(
                ["ros2", "lifecycle", "set", f"/{node_name}", transition],
                check=True,
                stdout=handle,
                stderr=subprocess.STDOUT,
            )
            time.sleep(1.5)

        rclpy.init()
        probe = Probe(cloud_frame, f"/{node_name}")

        print(f"{layer['plugin']}, voxel_decay = {decay} s")
        print(f"feeding an obstacle at {OBSTACLE} m for {FEED_SECONDS:.0f} s")
        deadline = time.monotonic() + FEED_SECONDS
        peak = 0
        while time.monotonic() < deadline:
            probe.publish_obstacle()
            rclpy.spin_once(probe, timeout_sec=0.4)
            peak = max(peak, probe.marked_cells() or 0)
        print(f"  while present: {peak} lethal cells")
        if not peak:
            print(
                "FAIL: the layer never marked -- nothing to time. Check the "
                f"log at {args.log}"
            )
            return 1

        # Two decay periods, so a mark that is going to expire has expired and
        # one that is being re-marked has clearly shown it.
        watch = 2 * decay + 5.0
        print(
            f"obstacle removed (no more clouds), robot stationary; watching "
            f"{watch:.0f} s"
        )
        removed = time.monotonic()
        reported = 0.0
        cleared = None
        while (elapsed := time.monotonic() - removed) < watch:
            rclpy.spin_once(probe, timeout_sec=0.4)
            marked = probe.marked_cells()
            if elapsed - reported >= 1.0:
                print(f"  t+{elapsed:5.1f}s  lethal cells: {marked}")
                reported = elapsed
            if not marked:
                cleared = elapsed
                break

        if cleared is None:
            print(
                f"FAIL: still marked {watch:.0f} s after the obstacle left. "
                "The mark is permanent -- the usual cause is the source's "
                "buffer being re-read, i.e. clear_after_reading is not True."
            )
            return 1
        print(
            f"OK: cleared {cleared:.1f} s after the obstacle left "
            f"(voxel_decay {decay} s, no robot motion)"
        )
        return 0
    finally:
        try:
            rclpy.shutdown()
        except RuntimeError:
            pass
        for proc in procs:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass
        handle.close()
        os.unlink(params_file)


if __name__ == "__main__":
    sys.exit(main())
