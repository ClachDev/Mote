"""Offline geometry check: draw a metric floor grid into real bag frames.

Validates the pixel<->ground projection on real footage. It samples frames from a
recorded bag, projects a floor grid (range arcs and bearing lines) into each via
GroundProjector, and overlays the lidar return points (transformed into base via
the bag's /tf_static, then projected onto the floor) as an independent geometry
cross-check: where a wall meets the floor, the projected lidar points should land
on the base of that wall in the image. Outputs annotated PNGs to inspect by eye.

Run in the dev or sim pixi env (needs cv2 + rosbag2_py):
    pixi run -e dev python mote_perception/tools/bag_overlay.py <bag_dir> [--out DIR]
"""

import argparse
import os
import tempfile

import cv2
import numpy as np

import bag_utils
from mote_perception.ground_projection import GroundProjector
from mote_perception.lidar_rescale import scan_to_points

RANGE_ARCS = [0.3, 0.5, 1.0, 1.5, 2.0, 3.0]  # metres
BEARINGS = np.deg2rad([-30, -20, -10, 0, 10, 20, 30])


def draw_grid(img, proj):
    """Draw floor range arcs and bearing lines onto the image."""
    out = img.copy()
    # Range arcs: sample many bearings at each range, project, connect.
    fan = np.linspace(np.deg2rad(-60), np.deg2rad(60), 120)
    for r in RANGE_ARCS:
        xy = np.column_stack([r * np.cos(fan), r * np.sin(fan)])
        pts = proj.ground_to_pixels(xy).astype(np.int32)
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


def draw_lidar(img, proj, scan, T_base_scan):
    """Overlay lidar returns as floor points (geometry cross-check).

    The returns are transformed into base_footprint first — the scan frame is
    yawed 90 degrees from base on this robot, so skipping the transform is not
    an approximation, it is wrong.
    """
    pts = scan_to_points(
        scan.ranges,
        scan.angle_min,
        scan.angle_increment,
        scan.range_min,
        scan.range_max,
    )
    xy = (pts @ T_base_scan[:3, :3].T + T_base_scan[:3, 3])[:, :2]
    px = proj.ground_to_pixels(xy)
    for u, v in px.astype(np.int32):
        if 0 <= u < proj.width and 0 <= v < proj.height:
            cv2.circle(img, (u, v), 2, (255, 80, 255), -1)
    return img


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("bag", help="bag directory (mcap)")
    ap.add_argument("--frames", type=int, default=10)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    out = args.out or tempfile.mkdtemp(prefix="bag_overlay_")
    os.makedirs(out, exist_ok=True)

    imgs, scans, tf_static, caminfo = bag_utils.load_perception_bag(args.bag)
    T_bo, T_bs = bag_utils.base_transforms(tf_static, scans)
    proj = GroundProjector.from_camera_info(caminfo, T_bo)
    print(f"camera height = {proj.camera_height:.3f} m")

    for k, i in enumerate(np.linspace(0, len(imgs) - 1, args.frames).astype(int)):
        ts, jpeg = imgs[i]
        frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            continue
        _, scan = bag_utils.nearest_scan(scans, ts)
        annotated = draw_lidar(draw_grid(frame, proj), proj, scan, T_bs)
        path = os.path.join(out, f"frame_{k:02d}.png")
        cv2.imwrite(path, annotated)
        print(f"saved {path}")

    print(f"done: camera height {proj.camera_height:.3f} m -> {out}")


if __name__ == "__main__":
    main()
