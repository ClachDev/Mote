"""Render a bag to video with the floor segmentation overlaid, for inspection.

Each output frame is a panel:
  left  - camera image, floor mask in green, obstacle boundary in red
  right - top-down (BEV): camera obstacle points (cyan) vs lidar returns (magenta)
A header shows the frame's camera-only-obstacle bearing count (high = likely a
false-positive / phantom-floor frame to inspect).

    pixi run -e dev python mote_perception/tools/bag_video.py <bag> [out.mp4] [stride] [max_sec]
"""

import math
import sys

import cv2
import numpy as np
import rosbag2_py
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from mote_perception.free_space import FloorSegmenter
from mote_perception.ground_projection import GroundProjector, chain_static_transforms

BEV_M = 3.0
NEAR, FAR, CONE = 0.3, 1.2, 40.0
BEARING_TOL = math.radians(1.5)
BLIND_MARGIN = 0.35


def scan_base(scan, T):
    ranges = np.asarray(scan.ranges)
    ang = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
    good = np.isfinite(ranges) & (ranges > scan.range_min) & (ranges < scan.range_max)
    pts = np.column_stack(
        [
            ranges[good] * np.cos(ang[good]),
            ranges[good] * np.sin(ang[good]),
            np.zeros(good.sum()),
            np.ones(good.sum()),
        ]
    )
    base = (T @ pts.T).T[:, :2]
    return base


def bev(cam_xy, lidar_xy, size):
    img = np.full((size, size, 3), 30, np.uint8)
    cx = size // 2
    sc = size / (2 * BEV_M)
    for r in (0.5, 1.0, 2.0, 3.0):
        cv2.circle(img, (cx, size - 1), int(r * sc), (60, 60, 60), 1)
    cv2.line(img, (cx, 0), (cx, size - 1), (60, 60, 60), 1)

    def plot(xy, color):
        for x, y in xy:
            if x <= 0:
                continue
            u, v = int(cx - y * sc), int(size - 1 - x * sc)
            if 0 <= u < size and 0 <= v < size:
                cv2.circle(img, (u, v), 2, color, -1)

    if lidar_xy is not None:
        plot(lidar_xy, (255, 80, 255))
    plot(cam_xy, (255, 255, 0))
    return img


def main():
    bag = (
        sys.argv[1] if len(sys.argv) > 1 else "/home/michael/.mote/bags/20260627_132846"
    )
    out = (
        sys.argv[2]
        if len(sys.argv) > 2
        else "/home/michael/.claude/jobs/b37cd0ff/tmp/segmentation.mp4"
    )
    stride = int(sys.argv[3]) if len(sys.argv) > 3 else 2
    max_sec = float(sys.argv[4]) if len(sys.argv) > 4 else 1e9
    bridge = CvBridge()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    tf_static = cam_info = proj = seg = T_base_scan = None
    latest_scan = None
    img_count = written = 0
    writer = None
    t0_ns = None

    while reader.has_next():
        topic, data, t = reader.read_next()
        if t0_ns is None:
            t0_ns = t
        if (t - t0_ns) / 1e9 > max_sec:
            break
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
            if proj is None or img_count % stride != 0:
                continue
            msg = deserialize_message(data, get_message(types[topic]))
            frame = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            cam_xy, mask, boundary, is_obs = seg.detect(frame, proj)

            ov = frame.copy()
            ov[mask > 0] = (0.5 * ov[mask > 0] + np.array([0, 120, 0])).astype(np.uint8)
            for c in range(0, frame.shape[1], 2):
                if is_obs[c]:
                    cv2.circle(ov, (c, int(boundary[c])), 1, (0, 0, 255), -1)

            lidar_xy = (
                scan_base(latest_scan, T_base_scan)
                if (latest_scan is not None and T_base_scan is not None)
                else None
            )

            # count camera-only obstacle bearings vs lidar (rough FP/blind indicator)
            n_only = 0
            if lidar_xy is not None and len(cam_xy):
                lb = np.arctan2(lidar_xy[:, 1], lidar_xy[:, 0])
                lr = np.hypot(lidar_xy[:, 0], lidar_xy[:, 1])
                cb = np.arctan2(cam_xy[:, 1], cam_xy[:, 0])
                cr = np.hypot(cam_xy[:, 0], cam_xy[:, 1])
                band = (cr >= NEAR) & (cr <= FAR) & (np.abs(cb) <= math.radians(CONE))
                for bb, rr in zip(cb[band], cr[band]):
                    m = np.abs(lb - bb) <= BEARING_TOL
                    if m.any() and lr[m].min() > rr + BLIND_MARGIN:
                        n_only += 1

            h = frame.shape[0]
            panel = np.hstack([ov, cv2.resize(bev(cam_xy, lidar_xy, h), (h, h))])
            cv2.putText(
                panel,
                f"t={(t - t0_ns) / 1e9:5.1f}s  camera-only obstacles: {n_only}",
                (10, 22),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255) if n_only > 20 else (0, 255, 0),
                2,
            )
            if writer is None:
                writer = cv2.VideoWriter(
                    out,
                    cv2.VideoWriter_fourcc(*"mp4v"),
                    15,
                    (panel.shape[1], panel.shape[0]),
                )
            writer.write(panel)
            written += 1

    if writer is not None:
        writer.release()
    print(f"wrote {written} frames -> {out}")


if __name__ == "__main__":
    main()
