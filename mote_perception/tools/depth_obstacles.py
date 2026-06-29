"""Turn Depth Anything metric depth into obstacles and compare to lidar.

Reads the bag (same stride as the frame extraction), loads the precomputed metric
depth (.npy) per frame, back-projects to 3D in the base frame, marks points above
the floor as obstacles, and renders:
  [ camera | colorized depth | BEV: depth-obstacles (cyan) vs lidar (magenta) ]
Also reports depth-vs-lidar range error at matching bearings (metric accuracy).

    pixi run -e dev python mote_perception/tools/depth_obstacles.py
"""

import math
import os

import cv2
import numpy as np
import rosbag2_py
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from mote_perception.ground_projection import GroundProjector, chain_static_transforms
from mote_perception.depth_rescale import DepthFloorRescaler

BAG = "/home/michael/.mote/bags/20260627_132846"
DEPTH = "/home/michael/.claude/jobs/b37cd0ff/tmp/depth"
OUT = "/home/michael/.claude/jobs/b37cd0ff/tmp/depth_obs"
STRIDE = 280
Z_OBSTACLE = 0.04  # m above floor to count as obstacle
Z_CEIL = 1.6
RMIN, RMAX = 0.25, 3.0
CONE = math.radians(45)
BEV_M, BEV_PX = 3.0, 360
BEARING_TOL = math.radians(1.5)


def scan_base(scan, T):
    rng = np.asarray(scan.ranges)
    ang = scan.angle_min + np.arange(len(rng)) * scan.angle_increment
    g = np.isfinite(rng) & (rng > scan.range_min) & (rng < scan.range_max)
    pts = np.column_stack(
        [
            rng[g] * np.cos(ang[g]),
            rng[g] * np.sin(ang[g]),
            np.zeros(g.sum()),
            np.ones(g.sum()),
        ]
    )
    return (T @ pts.T).T[:, :2]


def bev(cam_xy, lidar_xy):
    img = np.full((BEV_PX, BEV_PX, 3), 30, np.uint8)
    cx = BEV_PX // 2
    sc = BEV_PX / (2 * BEV_M)
    for r in (0.5, 1.0, 2.0, 3.0):
        cv2.circle(img, (cx, BEV_PX - 1), int(r * sc), (60, 60, 60), 1)

    def plot(xy, color, rad):
        for x, y in xy:
            if x <= 0:
                continue
            u, v = int(cx - y * sc), int(BEV_PX - 1 - x * sc)
            if 0 <= u < BEV_PX and 0 <= v < BEV_PX:
                cv2.circle(img, (u, v), rad, color, -1)

    if lidar_xy is not None:
        plot(lidar_xy, (255, 80, 255), 2)
    plot(cam_xy, (255, 255, 0), 1)
    return img


def main():
    os.makedirs(OUT, exist_ok=True)
    bridge = CvBridge()
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=BAG, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    tf_static = cam_info = proj = T_base_scan = rescaler = None
    uu = vv = None
    scans = []
    img_count = saved = 0
    err = []  # depth-vs-lidar range errors

    while reader.has_next() and saved < 20:
        topic, data, _ = reader.read_next()
        if topic == "/tf_static" and tf_static is None:
            tf_static = deserialize_message(data, get_message(types[topic]))
        elif topic == "/camera_info" and cam_info is None:
            cam_info = deserialize_message(data, get_message(types[topic]))
        elif topic == "/scan_filtered":
            scan = deserialize_message(data, get_message(types[topic]))
            if tf_static is not None:
                if T_base_scan is None:
                    T_base_scan = chain_static_transforms(
                        tf_static.transforms, scan.header.frame_id, "base_footprint"
                    )
                st = scan.header.stamp.sec * 1e9 + scan.header.stamp.nanosec
                scans.append((st, scan_base(scan, T_base_scan)))
                scans[:] = scans[-50:]
        elif topic == "/image_raw/compressed":
            if proj is None and tf_static is not None and cam_info is not None:
                T = chain_static_transforms(
                    tf_static.transforms, "camera_optical_link", "base_footprint"
                )
                proj = GroundProjector.from_camera_info(cam_info, T)
                rescaler = DepthFloorRescaler(proj)
                uu, vv = np.meshgrid(np.arange(proj.width), np.arange(proj.height))
                uu, vv = uu.ravel(), vv.ravel()
            img_count += 1
            if proj is None or img_count % STRIDE != 0:
                continue
            npy = f"{DEPTH}/frame_{saved:02d}.npy"
            if not os.path.exists(npy):
                saved += 1
                continue
            msg = deserialize_message(data, get_message(types[topic]))
            ist = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
            frame = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            depth_raw = np.load(npy)
            depth_corr, (a, b, frac) = rescaler.rescale(depth_raw)
            depth = depth_corr.ravel()

            fx, fy = proj.K[0, 0], proj.K[1, 1]
            cx0, cy0 = proj.K[0, 2], proj.K[1, 2]
            X = (uu - cx0) / fx * depth
            Y = (vv - cy0) / fy * depth
            Z = depth
            pts_opt = np.column_stack([X, Y, Z])
            pts_base = pts_opt @ proj.R.T + proj.C  # optical -> base
            bx, by, bz = pts_base[:, 0], pts_base[:, 1], pts_base[:, 2]
            rng = np.hypot(bx, by)
            bearing = np.arctan2(by, bx)
            obs = (
                (bz > Z_OBSTACLE)
                & (bz < Z_CEIL)
                & (rng > RMIN)
                & (rng < RMAX)
                & (np.abs(bearing) < CONE)
            )
            cam_xy = np.column_stack([bx[obs], by[obs]])

            lidar_xy = min(scans, key=lambda s: abs(s[0] - ist))[1] if scans else None

            # metric accuracy: per lidar bearing in cone, nearest depth-obstacle range
            if lidar_xy is not None and obs.any():
                lb = np.arctan2(lidar_xy[:, 1], lidar_xy[:, 0])
                lr = np.hypot(lidar_xy[:, 0], lidar_xy[:, 1])
                cb = bearing[obs]
                cr = rng[obs]
                for bb, rr in zip(lb, lr):
                    if abs(bb) > CONE or rr < RMIN or rr > RMAX:
                        continue
                    near = np.abs(cb - bb) <= BEARING_TOL
                    if near.any():
                        err.append(abs(cr[near].min() - rr))

            dimg = cv2.imread(
                f"{DEPTH}/frame_{saved:02d}_gray.png", cv2.IMREAD_GRAYSCALE
            )
            dcol = (
                cv2.applyColorMap(dimg, cv2.COLORMAP_INFERNO)
                if dimg is not None
                else np.zeros_like(frame)
            )
            # tint pixels the node marks as obstacle (above floor, in range) red
            tint = frame.copy()
            obs_img = obs.reshape(proj.height, proj.width)
            tint[obs_img] = (0.35 * frame[obs_img] + np.array([0, 0, 165])).astype(
                np.uint8
            )

            b = bev(cam_xy, lidar_xy)
            h = frame.shape[0]
            panel = np.hstack([frame, tint, dcol, cv2.resize(b, (h, h))])
            cv2.imwrite(f"{OUT}/depth_{saved:02d}.png", panel)
            print(f"depth_{saved:02d}: obstacle pts={obs.sum()}")
            saved += 1

    e = np.array(err)
    print(
        f"\ndepth-vs-lidar range error (matched bearings, {RMIN}-{RMAX} m, +-{math.degrees(CONE):.0f} deg):"
    )
    if len(e):
        print(
            f"  n={len(e)}  mean {e.mean():.3f}  median {np.median(e):.3f}  "
            f"RMSE {np.sqrt((e**2).mean()):.3f}  p90 {np.percentile(e, 90):.3f} m"
        )
    print(f"-> {OUT}")


if __name__ == "__main__":
    main()
