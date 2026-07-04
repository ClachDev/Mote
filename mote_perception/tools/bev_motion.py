"""Prototype: motion-based BEV obstacle detection on a bag.

For sampled frame pairs separated by enough robot motion, warps both frames to a
metric bird's-eye view (via the fixed ground homography), registers the earlier
BEV into the later one using odometry, and differences them. Flat floor is
consistent after motion compensation (low difference); anything standing off the
floor parallaxes (high difference) -> obstacle. Colour/lighting are never used.

Per pair it saves a panel:
  [ current image | BEV now | prev BEV warped into now | obstacle heatmap ]

    pixi run -e dev python mote_perception/tools/bev_motion.py <bag> [out_dir]
"""

import math
import os
import sys
from collections import deque

import cv2
import numpy as np
import rosbag2_py
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from mote_perception.ground_projection import GroundProjector, chain_static_transforms

# BEV metric extents (base frame): x forward [X_NEAR, X_FAR], y left [-Y_HALF, Y_HALF]
RES = 0.01  # m / px
X_NEAR, X_FAR, Y_HALF = 0.2, 3.0, 1.5
BASELINE_MIN = 0.04  # m of translation required between paired frames
PAIR_DT = 0.45  # s desired separation between paired frames
DIFF_THRESH = 38  # grayscale difference -> obstacle
N_PANELS = 12
STRIDE = 3


def bev_size():
    w = int(round(2 * Y_HALF / RES))
    h = int(round((X_FAR - X_NEAR) / RES))
    return w, h


def metric_to_bev(x, y):
    u = (Y_HALF - y) / RES
    v = (X_FAR - x) / RES
    return u, v


def bev_homography(proj):
    """3x3 mapping BEV pixel -> image pixel (for warpPerspective WARP_INVERSE_MAP)."""
    w, h = bev_size()
    ground = np.array(
        [[X_FAR, Y_HALF], [X_FAR, -Y_HALF], [X_NEAR, Y_HALF], [X_NEAR, -Y_HALF]]
    )
    bev_pts = np.array([[0, 0], [w - 1, 0], [0, h - 1], [w - 1, h - 1]], np.float32)
    img_pts = proj.ground_to_pixels(ground).astype(np.float32)
    return cv2.getPerspectiveTransform(bev_pts, img_pts), w, h


def warp_bev(img, H, w, h):
    bev = cv2.warpPerspective(
        img,
        H,
        (w, h),
        flags=cv2.WARP_INVERSE_MAP,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0,
    )
    valid = cv2.warpPerspective(
        np.full(img.shape[:2], 255, np.uint8),
        H,
        (w, h),
        flags=cv2.WARP_INVERSE_MAP,
        borderValue=0,
    )
    return bev, valid


def motion_affine(prev_pose, cur_pose, w, h):
    """2x3 affine mapping prev BEV pixels into the current frame's BEV."""
    px, py, pyaw = prev_pose
    cx, cy, cyaw = cur_pose
    dyaw = pyaw - cyaw
    dwx, dwy = px - cx, py - cy
    dx = math.cos(cyaw) * dwx + math.sin(cyaw) * dwy
    dy = -math.sin(cyaw) * dwx + math.cos(cyaw) * dwy
    R = np.array([[math.cos(dyaw), -math.sin(dyaw)], [math.sin(dyaw), math.cos(dyaw)]])
    src = np.array(
        [[w * 0.3, h * 0.3], [w * 0.7, h * 0.3], [w * 0.5, h * 0.8]], np.float32
    )
    dst = np.empty_like(src)
    for i, (u, v) in enumerate(src):
        x = X_FAR - v * RES
        y = Y_HALF - u * RES
        p = R @ np.array([x, y]) + np.array(
            [dx, dy]
        )  # prev point in current base frame
        dst[i] = metric_to_bev(p[0], p[1])
    return cv2.getAffineTransform(src, dst)


def odom_pose(tf_msg):
    for tr in tf_msg.transforms:
        if tr.header.frame_id == "odom" and tr.child_frame_id == "base_footprint":
            t, q = tr.transform.translation, tr.transform.rotation
            yaw = math.atan2(
                2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
            )
            st = tr.header.stamp.sec * 1e9 + tr.header.stamp.nanosec
            return (t.x, t.y, yaw), st
    return None, None


def main():
    bag = (
        sys.argv[1]
        if len(sys.argv) > 1
        else os.path.expanduser("~/.mote/bags/20260627_132846")
    )
    out_dir = sys.argv[2] if len(sys.argv) > 2 else os.path.join(bag, "_bev")
    os.makedirs(out_dir, exist_ok=True)
    bridge = CvBridge()
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    tf_static = cam_info = proj = None
    H = w = h = None
    odoms = deque(maxlen=400)  # (stamp_ns, (x,y,yaw))
    history = deque(maxlen=40)  # (stamp_ns, pose, bev, valid)
    img_count = saved = 0

    def pose_at(stamp):
        if not odoms:
            return None
        return min(odoms, key=lambda o: abs(o[0] - stamp))[1]

    while reader.has_next() and saved < N_PANELS:
        topic, data, _ = reader.read_next()
        if topic == "/tf_static" and tf_static is None:
            tf_static = deserialize_message(data, get_message(types[topic]))
        elif topic == "/camera_info" and cam_info is None:
            cam_info = deserialize_message(data, get_message(types[topic]))
        elif topic == "/tf":
            pose, st = odom_pose(deserialize_message(data, get_message(types[topic])))
            if pose is not None:
                odoms.append((st, pose))
        elif topic == "/image_raw/compressed":
            if proj is None and tf_static is not None and cam_info is not None:
                T = chain_static_transforms(
                    tf_static.transforms, "camera_optical_link", "base_footprint"
                )
                proj = GroundProjector.from_camera_info(cam_info, T)
                H, w, h = bev_homography(proj)
            img_count += 1
            if proj is None or img_count % STRIDE != 0 or not odoms:
                continue
            msg = deserialize_message(data, get_message(types[topic]))
            st = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
            pose = pose_at(st)
            frame = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            bev, valid = warp_bev(frame, H, w, h)
            history.append((st, pose, bev, valid))

            # find an earlier frame ~PAIR_DT back with enough baseline
            prev = None
            for pst, ppose, pbev, pvalid in history:
                if (st - pst) / 1e9 >= PAIR_DT:
                    if (
                        math.hypot(pose[0] - ppose[0], pose[1] - ppose[1])
                        >= BASELINE_MIN
                    ):
                        prev = (pst, ppose, pbev, pvalid)
            if prev is None:
                continue
            _, ppose, pbev, pvalid = prev

            A = motion_affine(ppose, pose, w, h)
            pbev_w = cv2.warpAffine(pbev, A, (w, h))
            pvalid_w = cv2.warpAffine(pvalid, A, (w, h))
            both = (valid > 128) & (pvalid_w > 128)

            g_now = cv2.GaussianBlur(
                cv2.cvtColor(bev, cv2.COLOR_BGR2GRAY), (5, 5), 0
            ).astype(np.float32)
            g_prev = cv2.GaussianBlur(
                cv2.cvtColor(pbev_w, cv2.COLOR_BGR2GRAY), (5, 5), 0
            ).astype(np.float32)
            # match global brightness so exposure changes don't dominate
            if both.any():
                g_prev += g_now[both].mean() - g_prev[both].mean()
            diff = np.abs(g_now - g_prev)
            diff[~both] = 0
            obstacle = (diff > DIFF_THRESH).astype(np.uint8) * 255
            obstacle = cv2.morphologyEx(
                obstacle,
                cv2.MORPH_OPEN,
                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)),
            )

            heat = cv2.applyColorMap(
                np.clip(diff * 2, 0, 255).astype(np.uint8), cv2.COLORMAP_JET
            )
            heat[~both] = 0
            overlay = bev.copy()
            overlay[obstacle > 0] = (0, 0, 255)

            # --- standard cleanups: reliable near-band + nearest hit per bearing ---
            # (kills the radial smear of tall objects and the far-field garbage)
            v_band = int((X_FAR - 1.2) / RES)  # rows nearer than 1.2 m
            band = np.zeros_like(obstacle)
            band[v_band:, :] = obstacle[v_band:, :]
            cleaned_bgr = bev.copy()
            for deg in np.linspace(-40, 40, 100):
                th = math.radians(deg)
                hit = None
                for r in np.linspace(0.25, 1.2, 100):
                    u, v = metric_to_bev(r * math.cos(th), r * math.sin(th))
                    ui, vi = int(round(u)), int(round(v))
                    if 0 <= ui < w and 0 <= vi < h and band[vi, ui] > 0:
                        hit = (ui, vi)
                        break
                if hit:
                    cv2.circle(cleaned_bgr, hit, 3, (0, 0, 255), -1)

            trans = math.hypot(pose[0] - ppose[0], pose[1] - ppose[1])
            dyaw = abs(
                math.atan2(math.sin(pose[2] - ppose[2]), math.cos(pose[2] - ppose[2]))
            )

            ih = h
            img_r = cv2.resize(frame, (int(frame.shape[1] * ih / frame.shape[0]), ih))

            def label(im, txt):
                cv2.putText(
                    im, txt, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1
                )
                return im

            panel = np.hstack(
                [
                    label(img_r, "camera"),
                    label(heat, "diff (obstacles hot)"),
                    label(overlay, "raw obstacles"),
                    label(cleaned_bgr, "cleaned: near-band + nearest/bearing"),
                ]
            )
            p = os.path.join(out_dir, f"bev_{saved:02d}.png")
            cv2.imwrite(p, panel)
            print(
                f"saved {p}  trans={trans:.2f}m  yaw={math.degrees(dyaw):.1f}deg  "
                f"{'ROTATION-DOMINATED' if dyaw > 0.15 and trans < 0.1 else ''}"
            )
            saved += 1

    print(f"done -> {out_dir}")


if __name__ == "__main__":
    main()
