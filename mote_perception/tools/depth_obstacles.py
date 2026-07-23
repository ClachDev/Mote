"""Decision-level check of the obstacle pipeline against a bag, vs lidar.

Re-runs the stages the live node runs (server depth -> lidar rescale ->
back-project -> level -> z/range gates) on sampled frames and renders, per frame:
  [ camera | obstacle tint | corrected depth | BEV: camera (cyan) vs lidar (magenta) ]
Both BEV point sets are transformed into base_footprint via the bag's /tf_static,
so the overlay is frame-correct by construction (the scan frame is yawed 90 deg
from base on this robot — raw scan coordinates are not base coordinates). Also
reports depth-vs-lidar range error at matching bearings. This is the committed
generator for the camera-vs-lidar BEV figure.

Needs a depth server listening (see depth_bag_replay.py):
    pixi run python mote_perception/tools/depth_obstacles.py <bag> [--frames N] [--out DIR]
"""

import argparse
import math
import os
import sys
import tempfile

import cv2
import numpy as np

import bag_utils
from mote_perception.depth_wire import DEFAULT_PORT, DepthClient
from mote_perception.ground_projection import (
    GroundProjector,
    fit_ground_plane,
    level_rotation,
)
from mote_perception.lidar_rescale import LidarDepthRescaler, scan_to_points

# Decision gates: match the node's defaults (depth_obstacle_node.py).
Z_OBSTACLE, Z_MAX = 0.02, 0.5
RMIN, RMAX = 0.25, 1.2
CONE = math.radians(45)
BEV_M, BEV_PX = 3.0, 360
BEARING_TOL = math.radians(1.5)


def bev(cam_xy, lidar_xy):
    """Top-down render, robot at bottom centre, x forward up, y left to the left."""
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

    plot(lidar_xy, (255, 80, 255), 2)
    plot(cam_xy, (255, 255, 0), 1)
    cv2.putText(
        img,
        "cyan=camera  magenta=lidar",
        (8, BEV_PX - 8),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.4,
        (200, 200, 200),
        1,
        cv2.LINE_AA,
    )
    return img


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("bag", help="bag directory (mcap)")
    ap.add_argument("--frames", type=int, default=8)
    ap.add_argument("--out", default=None)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    args = ap.parse_args()
    out = args.out or tempfile.mkdtemp(prefix="depth_obs_")
    os.makedirs(out, exist_ok=True)

    imgs, scans, tf_static, caminfo = bag_utils.load_perception_bag(args.bag)
    T_bo, T_bs = bag_utils.base_transforms(tf_static, scans)
    proj = GroundProjector.from_camera_info(caminfo, T_bo)
    resc = LidarDepthRescaler(caminfo.k, caminfo.d, np.linalg.inv(T_bo) @ T_bs)

    client = DepthClient(args.host, args.port)
    if client.connect() is None:
        sys.exit(
            "start the server with `pixi run depth-server` (or `pixi run inference`)"
        )

    err = []  # depth-vs-lidar range errors at matched bearings
    print(f"{len(imgs)} imgs; writing to {out}")
    for k, i in enumerate(np.linspace(0, len(imgs) - 1, args.frames).astype(int)):
        ts, jpeg = imgs[i]
        depth = client.infer(jpeg)
        if depth is None:
            print(f"[f{k:>2}] no depth for this frame")
            continue
        _, scan = bag_utils.nearest_scan(scans, ts)
        fit = resc.rescale(depth, scan)
        if fit is None:
            print(f"[f{k:>2}] lidar can't constrain the fit; skipping")
            continue
        depth_corr, (a, b, frac) = fit

        # Back-project and level, exactly as the node does.
        pts = proj.back_project(depth_corr)
        plane = fit_ground_plane(pts)
        if plane is not None:
            pa, pb, pc, _ = plane
            pts = (pts - proj.C) @ level_rotation(pa, pb).T + proj.C
            pts[:, 2] -= pc
        bx, by, bz = pts[:, 0], pts[:, 1], pts[:, 2]
        rng = np.hypot(bx, by)
        bearing = np.arctan2(by, bx)
        obs = (
            (bz > Z_OBSTACLE)
            & (bz < Z_MAX)
            & (rng > RMIN)
            & (rng < RMAX)
            & (np.abs(bearing) < CONE)
        )
        cam_xy = np.column_stack([bx[obs], by[obs]])

        pts_l = scan_to_points(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
        )
        lidar_xy = (pts_l @ T_bs[:3, :3].T + T_bs[:3, 3])[:, :2]

        # Metric accuracy: per lidar bearing in the cone, nearest obstacle range.
        if obs.any():
            lb = np.arctan2(lidar_xy[:, 1], lidar_xy[:, 0])
            lr = np.hypot(lidar_xy[:, 0], lidar_xy[:, 1])
            cb, cr = bearing[obs], rng[obs]
            for bb, rr in zip(lb, lr):
                if abs(bb) > CONE or rr < RMIN or rr > RMAX:
                    continue
                near = np.abs(cb - bb) <= BEARING_TOL
                if near.any():
                    err.append(abs(cr[near].min() - rr))

        frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            continue
        tint = frame.copy()
        obs_img = obs.reshape(proj.height, proj.width)
        tint[obs_img] = (0.35 * frame[obs_img] + np.array([0, 0, 165])).astype(np.uint8)

        h = frame.shape[0]
        panel = np.hstack(
            [
                frame,
                tint,
                bag_utils.colorize(depth_corr, 5.0),
                cv2.resize(bev(cam_xy, lidar_xy), (h, h)),
            ]
        )
        cv2.imwrite(f"{out}/depth_{k:02d}.png", panel)
        print(f"[f{k:>2}] obstacle pts={obs.sum()}  a={a:.3f} b={b:.3f} inl {frac:.0%}")

    e = np.array(err)
    print(
        f"\ndepth-vs-lidar range error (matched bearings, {RMIN}-{RMAX} m, "
        f"+-{math.degrees(CONE):.0f} deg):"
    )
    if len(e):
        print(
            f"  n={len(e)}  mean {e.mean():.3f}  median {np.median(e):.3f}  "
            f"RMSE {np.sqrt((e**2).mean()):.3f}  p90 {np.percentile(e, 90):.3f} m"
        )
    print(f"-> {out}")


if __name__ == "__main__":
    main()
