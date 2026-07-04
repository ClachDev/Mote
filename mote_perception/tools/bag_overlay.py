"""Offline geometry check: draw a metric floor grid into real bag frames.

Before any obstacle CV, this validates the pixel<->ground projection on real
footage. It samples frames from a recorded bag, projects a floor grid (range arcs
and bearing lines) into each via GroundProjector, and overlays the lidar return
points (projected onto the floor) as an independent geometry cross-check: where a
wall meets the floor, the projected lidar points should land on the base of that
wall in the image. Outputs annotated PNGs to inspect by eye.

Run in the dev or sim pixi env (needs cv2 + rosbag2_py):
    pixi run -e dev python mote_perception/tools/bag_overlay.py <bag_dir> [out_dir]
"""

import os
import sys

import cv2
import numpy as np
import rosbag2_py
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from mote_perception.ground_projection import (
    GroundProjector,
    chain_static_transforms,
)

RANGE_ARCS = [0.3, 0.5, 1.0, 1.5, 2.0, 3.0]  # metres
BEARINGS = np.deg2rad([-30, -20, -10, 0, 10, 20, 30])
N_FRAMES = 10


def open_reader(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    return reader, types


def draw_grid(img, proj):
    """Draw floor range arcs and bearing lines onto the image."""
    out = img.copy()
    # Range arcs: sample many bearings at each range, project, connect.
    fan = np.linspace(np.deg2rad(-60), np.deg2rad(60), 120)
    for r in RANGE_ARCS:
        xy = np.column_stack([r * np.cos(fan), r * np.sin(fan)])
        px = proj.ground_to_pixels(xy)
        pts = px.astype(np.int32)
        for a, b in zip(pts[:-1], pts[1:]):
            cv2.line(out, tuple(a), tuple(b), (0, 200, 255), 1, cv2.LINE_AA)
        mid = pts[len(pts) // 2]
        cv2.putText(
            out,
            f"{r:g}m",
            tuple(mid),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (0, 200, 255),
            1,
            cv2.LINE_AA,
        )
    # Bearing lines from near to far.
    rr = np.linspace(0.2, 3.0, 40)
    for b in BEARINGS:
        xy = np.column_stack([rr * np.cos(b), rr * np.sin(b)])
        px = proj.ground_to_pixels(xy).astype(np.int32)
        for p0, p1 in zip(px[:-1], px[1:]):
            cv2.line(out, tuple(p0), tuple(p1), (80, 220, 80), 1, cv2.LINE_AA)
    return out


def draw_lidar(img, proj, scan):
    """Project lidar returns onto the floor and overlay them (geometry cross-check)."""
    out = img
    ranges = np.asarray(scan.ranges)
    angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
    good = np.isfinite(ranges) & (ranges > scan.range_min) & (ranges < scan.range_max)
    # Lidar frame: x forward, y left. The lidar is rotated/offset from base, but
    # both share heading here closely enough for a base-frame floor overlay sanity
    # check; project the (x, y) of each return as a floor point.
    x = ranges[good] * np.cos(angles[good])
    y = ranges[good] * np.sin(angles[good])
    px = proj.ground_to_pixels(np.column_stack([x, y]))
    for u, v in px.astype(np.int32):
        if 0 <= u < proj.width and 0 <= v < proj.height:
            cv2.circle(out, (u, v), 2, (255, 80, 255), -1)
    return out


def main():
    bag = (
        sys.argv[1]
        if len(sys.argv) > 1
        else os.path.expanduser("~/.mote/bags/20260627_132846")
    )
    out_dir = sys.argv[2] if len(sys.argv) > 2 else os.path.join(bag, "_overlay")
    os.makedirs(out_dir, exist_ok=True)
    bridge = CvBridge()

    reader, types = open_reader(bag)
    tf_static = None
    cam_info = None
    proj = None
    latest_scan = None
    img_count = 0
    saved = 0
    # estimate total image frames to set a stride; metadata said ~13k camera_info
    stride = 700

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == "/tf_static" and tf_static is None:
            tf_static = deserialize_message(data, get_message(types[topic]))
        elif topic == "/camera_info" and cam_info is None:
            cam_info = deserialize_message(data, get_message(types[topic]))
        elif topic == "/scan_filtered":
            latest_scan = deserialize_message(data, get_message(types[topic]))
        elif topic == "/image_raw/compressed":
            if proj is None and tf_static is not None and cam_info is not None:
                T = chain_static_transforms(
                    tf_static.transforms, "camera_optical_link", "base_footprint"
                )
                proj = GroundProjector.from_camera_info(cam_info, T)
                print(f"camera height = {proj.camera_height:.3f} m")
            img_count += 1
            if proj is None or img_count % stride != 0:
                continue
            msg = deserialize_message(data, get_message(types[topic]))
            frame = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            annotated = draw_grid(frame, proj)
            if latest_scan is not None:
                annotated = draw_lidar(annotated, proj, latest_scan)
            path = os.path.join(out_dir, f"frame_{saved:02d}.png")
            cv2.imwrite(path, annotated)
            print(f"saved {path}")
            saved += 1
            if saved >= N_FRAMES:
                break

    print(
        f"done: {saved} frames, camera height {proj.camera_height:.3f} m -> {out_dir}"
    )


if __name__ == "__main__":
    main()
