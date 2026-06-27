"""Quantitative L1 spike evaluation against a recorded bag.

Produces the numbers that decide push-vs-pivot, per the roadmap's L1 deliverable:
  * range error of the camera obstacle boundary vs lidar, where both see a surface
    (geometry accuracy) -- scan synced to image by timestamp, reported overall and
    for the low-motion subset (motion blur / sync latency removed);
  * runtime per frame of the detector (workstation CPU; Pi will be slower);
  * lidar-blind "existence" cases: camera marks an obstacle with no corresponding
    lidar return at that bearing -- the actual point of a camera obstacle layer.

Scope: tuned and measured on ONE bag / floor / lighting. The verdict is "on this
floor"; diverse bags are needed before generalising.

    pixi run -e dev python mote_perception/tools/quant_eval.py <bag_dir> [out_dir]
"""

import math
import os
import sys
import time

import cv2
import numpy as np
import rosbag2_py
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from mote_perception.free_space import FloorSegmenter
from mote_perception.ground_projection import GroundProjector, chain_static_transforms

STRIDE = 30  # process every Nth image
SYNC_TOL_NS = 200_000_000  # 200 ms image<->scan (scans are irregular ~7-10 Hz)
TIGHT_SYNC_MS = 80.0  # subset with clean sync
SLOW_V = 0.12  # m/s: "slower" subset for latency-robust error
NEAR, FAR = 0.3, 1.2  # camera reliable range band, metres
CONE_DEG = 40.0  # forward cone compared against lidar
BEARING_TOL = math.radians(1.5)
LOWMO_V, LOWMO_W = 0.05, 0.1  # m/s, rad/s thresholds for "low motion"
BLIND_MARGIN = 0.35  # lidar farther than cam range by this -> blind candidate


def reader_for(bag):
    r = rosbag2_py.SequentialReader()
    r.open(
        rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    return r, {t.name: t.type for t in r.get_all_topics_and_types()}


def scan_polar_base(scan, T):
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
    return np.arctan2(base[:, 1], base[:, 0]), np.hypot(base[:, 0], base[:, 1])


def odom_pose(tf_msg):
    for tr in tf_msg.transforms:
        if tr.header.frame_id == "odom" and tr.child_frame_id == "base_footprint":
            t = tr.transform.translation
            q = tr.transform.rotation
            yaw = math.atan2(
                2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
            )
            stamp = tr.header.stamp.sec * 1_000_000_000 + tr.header.stamp.nanosec
            return np.array([t.x, t.y, yaw]), stamp
    return None, None


def main():
    bag = (
        sys.argv[1] if len(sys.argv) > 1 else "/home/michael/.mote/bags/20260627_132846"
    )
    out_dir = (
        sys.argv[2]
        if len(sys.argv) > 2
        else "/home/michael/.claude/jobs/b37cd0ff/tmp/quant"
    )
    os.makedirs(out_dir, exist_ok=True)
    bridge = CvBridge()
    reader, types = reader_for(bag)

    tf_static = cam_info = proj = seg = T_base_scan = None
    scans = []  # (stamp_ns, bearings, ranges)
    last_odom = None
    img_count = proc = 0
    errors = []  # (abs_err, sync_ms, speed)
    runtimes = []
    blind = []  # existence cases: dicts

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == "/tf_static" and tf_static is None:
            tf_static = deserialize_message(data, get_message(types[topic]))
        elif topic == "/camera_info" and cam_info is None:
            cam_info = deserialize_message(data, get_message(types[topic]))
        elif topic == "/tf":
            pose, stamp = odom_pose(
                deserialize_message(data, get_message(types[topic]))
            )
            if pose is not None:
                last_odom = (pose, stamp)
        elif topic == "/scan_filtered":
            scan = deserialize_message(data, get_message(types[topic]))
            if tf_static is not None:
                if T_base_scan is None:
                    T_base_scan = chain_static_transforms(
                        tf_static.transforms, scan.header.frame_id, "base_footprint"
                    )
                st = scan.header.stamp.sec * 1_000_000_000 + scan.header.stamp.nanosec
                b, r = scan_polar_base(scan, T_base_scan)
                scans.append((st, b, r))
                scans[:] = scans[-50:]
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
            if proj is None or img_count % STRIDE != 0 or not scans:
                continue
            msg = deserialize_message(data, get_message(types[topic]))
            ist = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            sst, sb, sr = min(scans, key=lambda s: abs(s[0] - ist))
            if abs(sst - ist) > SYNC_TOL_NS:
                continue

            # robot linear speed near this frame (finite diff over the last odom step)
            speed = float("nan")
            sync_ms = abs(sst - ist) / 1e6
            if last_odom is not None:
                (px, py, _), pst = last_odom
                if getattr(main, "_prev", None) is not None:
                    (qx, qy, _), qst = main._prev
                    dt = (pst - qst) / 1e9
                    if dt > 1e-3:
                        speed = math.hypot(px - qx, py - qy) / dt
                main._prev = last_odom

            frame = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            t0 = time.perf_counter()
            cam_xy, mask, boundary, is_obs = seg.detect(frame, proj)
            runtimes.append((time.perf_counter() - t0) * 1000.0)
            proc += 1

            cam_b = np.arctan2(cam_xy[:, 1], cam_xy[:, 0])
            cam_r = np.hypot(cam_xy[:, 0], cam_xy[:, 1])
            in_band = (
                (cam_r >= NEAR)
                & (cam_r <= FAR)
                & (np.abs(cam_b) <= math.radians(CONE_DEG))
            )
            n_blind_frame = 0
            for bb, rr in zip(cam_b[in_band], cam_r[in_band]):
                near_lidar = np.abs(sb - bb) <= BEARING_TOL
                if not near_lidar.any():
                    continue
                lr = sr[near_lidar].min()
                if abs(lr - rr) <= 0.5:  # both see the same surface
                    errors.append((abs(rr - lr), sync_ms, speed))
                elif lr > rr + BLIND_MARGIN:  # lidar free/farther: camera-only obstacle
                    n_blind_frame += 1
            if n_blind_frame >= 3:
                blind.append(
                    (
                        n_blind_frame,
                        ist,
                        frame.copy(),
                        boundary.copy(),
                        is_obs.copy(),
                        cam_xy.copy(),
                        sb.copy(),
                        sr.copy(),
                    )
                )

    # ---- report ----
    rt = np.array(runtimes)
    err = np.array(errors)  # columns: abs_err, sync_ms, speed

    def stat(label, e):
        if len(e) == 0:
            print(f"  {label}: (no matches)")
            return
        print(
            f"  {label} (n={len(e)}): mean {e.mean():.3f}  median {np.median(e):.3f}  "
            f"RMSE {np.sqrt((e**2).mean()):.3f}  p90 {np.percentile(e, 90):.3f} m"
        )

    print(f"\n=== L1 spike quantitative report (bag: {os.path.basename(bag)}) ===")
    print(f"frames processed: {proc}")
    print(
        f"\nruntime/frame (workstation CPU): mean {rt.mean():.1f} ms  "
        f"median {np.median(rt):.1f} ms  p95 {np.percentile(rt, 95):.1f} ms"
        f"  -> {1000 / rt.mean():.0f} FPS"
    )
    print(
        f"\nrange error vs lidar (same-surface, {NEAR}-{FAR} m, +-{CONE_DEG} deg), "
        f"stratified to separate motion/sync latency from geometry error:"
    )
    stat("ALL matches", err[:, 0])
    stat(f"tight sync (<={TIGHT_SYNC_MS:g} ms)", err[err[:, 1] <= TIGHT_SYNC_MS, 0])
    slow = err[(err[:, 2] <= SLOW_V) & np.isfinite(err[:, 2])]
    stat(f"slower (<={SLOW_V} m/s)", slow[:, 0])
    tight_slow = err[
        (err[:, 1] <= TIGHT_SYNC_MS) & (err[:, 2] <= SLOW_V) & np.isfinite(err[:, 2])
    ]
    stat("tight sync AND slow", tight_slow[:, 0])
    if np.isfinite(err[:, 2]).any():
        sp = err[np.isfinite(err[:, 2]), 2]
        print(
            f"  (frame speeds: median {np.median(sp):.2f}  p90 {np.percentile(sp, 90):.2f} m/s)"
        )
    print(
        f"\nlidar-blind existence frames (>=3 camera-only obstacle bearings): {len(blind)}"
    )

    blind.sort(key=lambda x: -x[0])
    for i, (n, ist, frame, boundary, is_obs, cam_xy, sb, sr) in enumerate(blind[:6]):
        ov = frame.copy()
        for c in range(0, frame.shape[1], 2):
            if is_obs[c]:
                cv2.circle(ov, (c, int(boundary[c])), 1, (0, 0, 255), -1)
        cv2.putText(
            ov,
            f"{n} camera-only obstacle bearings",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2,
        )
        p = os.path.join(out_dir, f"blind_{i:02d}_{n}.png")
        cv2.imwrite(p, ov)
        print(f"  saved {p}")


if __name__ == "__main__":
    main()
