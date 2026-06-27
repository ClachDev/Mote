"""Offline evaluation of floor segmentation against a recorded bag.

For each sampled frame produces a side-by-side panel:
  left  - camera image with the floor mask (green) and obstacle boundary (red)
  right - top-down (BEV) view comparing camera obstacle points (cyan) against the
          lidar returns (magenta), both in the base frame

The lidar is the geometry ground truth where it and the camera both see a surface
(walls, furniture meeting the floor). Disagreement on the lidar-blind case (low
objects) is the point of the camera and must be judged by eye on the image.

    pixi run -e dev python mote_perception/tools/floor_eval.py <bag_dir> [out_dir]
"""

import os
import sys

import cv2
import numpy as np
import rosbag2_py
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from mote_perception.free_space import FloorSegmenter
from mote_perception.ground_projection import (
    GroundProjector,
    chain_static_transforms,
)

N_FRAMES = 12
STRIDE = 560
BEV_M = 3.0  # forward/side extent shown, metres
BEV_PX = 480


def bev_canvas():
    img = np.full((BEV_PX, BEV_PX, 3), 30, np.uint8)
    cx = BEV_PX // 2
    scale = BEV_PX / (2 * BEV_M)  # px per metre, y spans [-BEV_M, BEV_M]
    for r in (0.5, 1.0, 2.0, 3.0):
        cv2.circle(img, (cx, BEV_PX - 1), int(r * scale), (60, 60, 60), 1)
    cv2.line(img, (cx, 0), (cx, BEV_PX - 1), (60, 60, 60), 1)
    return img, cx, scale


def bev_plot(img, cx, scale, xy, color):
    for x, y in xy:
        if x <= 0:
            continue
        u = int(cx - y * scale)  # +y (left) goes left on screen
        v = int(BEV_PX - 1 - x * scale)  # +x (forward) goes up
        if 0 <= u < BEV_PX and 0 <= v < BEV_PX:
            cv2.circle(img, (u, v), 2, color, -1)


def scan_to_base(scan, T_base_scan):
    ranges = np.asarray(scan.ranges)
    angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
    good = np.isfinite(ranges) & (ranges > scan.range_min) & (ranges < scan.range_max)
    pts = np.column_stack(
        [
            ranges[good] * np.cos(angles[good]),
            ranges[good] * np.sin(angles[good]),
            np.zeros(good.sum()),
            np.ones(good.sum()),
        ]
    )
    base = (T_base_scan @ pts.T).T
    return base[:, :2]


def main():
    bag = (
        sys.argv[1] if len(sys.argv) > 1 else "/home/michael/.mote/bags/20260627_132846"
    )
    out_dir = (
        sys.argv[2]
        if len(sys.argv) > 2
        else "/home/michael/.claude/jobs/b37cd0ff/tmp/eval"
    )
    os.makedirs(out_dir, exist_ok=True)
    bridge = CvBridge()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    tf_static = cam_info = proj = seg = None
    T_base_scan = None
    latest_scan = None
    img_count = saved = 0

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == "/tf_static" and tf_static is None:
            tf_static = deserialize_message(data, get_message(types[topic]))
        elif topic == "/camera_info" and cam_info is None:
            cam_info = deserialize_message(data, get_message(types[topic]))
        elif topic == "/scan_filtered":
            latest_scan = deserialize_message(data, get_message(types[topic]))
            if tf_static is not None and T_base_scan is None:
                T_base_scan = chain_static_transforms(
                    tf_static.transforms, latest_scan.header.frame_id, "base_footprint"
                )
        elif topic == "/image_raw/compressed":
            if proj is None and tf_static is not None and cam_info is not None:
                T = chain_static_transforms(
                    tf_static.transforms, "camera_optical_link", "base_footprint"
                )
                proj = GroundProjector.from_camera_info(cam_info, T)
                seg = FloorSegmenter(
                    horizon_row=int(proj.K[1, 2]),
                    seed_rows=(0.90, 0.99),
                    seed_cols=(0.05, 0.95),
                    thresh=0.015,
                )
            img_count += 1
            if proj is None or img_count % STRIDE != 0:
                continue
            msg = deserialize_message(data, get_message(types[topic]))
            frame = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

            cam_xy, mask, boundary, is_obs = seg.detect(frame, proj)

            overlay = frame.copy()
            overlay[mask > 0] = (
                0.5 * overlay[mask > 0] + np.array([0, 120, 0])
            ).astype(np.uint8)
            for c in range(0, frame.shape[1], 2):
                if is_obs[c]:
                    cv2.circle(overlay, (c, int(boundary[c])), 1, (0, 0, 255), -1)

            bev, bcx, bscale = bev_canvas()
            if latest_scan is not None and T_base_scan is not None:
                bev_plot(
                    bev,
                    bcx,
                    bscale,
                    scan_to_base(latest_scan, T_base_scan),
                    (255, 80, 255),
                )
            bev_plot(bev, bcx, bscale, cam_xy, (255, 255, 0))

            panel = np.hstack(
                [overlay, cv2.resize(bev, (frame.shape[0], frame.shape[0]))]
            )
            path = os.path.join(out_dir, f"eval_{saved:02d}.png")
            cv2.imwrite(path, panel)
            print(
                f"saved {path}  cam_pts={len(cam_xy)} lidar_pts={'-' if latest_scan is None else 'ok'}"
            )
            saved += 1
            if saved >= N_FRAMES:
                break

    print(f"done -> {out_dir}")


if __name__ == "__main__":
    main()
