"""Offline replay of the depth pipeline against a recorded bag.

Re-runs the exact off-board depth path on recorded footage: forwards each sampled
`/image_raw/compressed` frame to a running depth server, rescales the returned
depth against the time-nearest `/scan_filtered` via the body-fixed lidar->camera
transform (from the bag's `/tf_static`), and reports per-frame what the live node
would compute -- raw model depth stats, the lidar (pred,true) pair count and depth
spread, the fitted affine (a, b) and inlier %, and the rescaled depth range. It
saves colorized raw and corrected depth maps (TURBO) for eyeballing.

This is the rig used to find the RANSAC bistability in the lidar fit: a frame whose
fit collapses prints `DEGENERATE` (slope a < 0.5, i.e. the near-flat/inverted line),
and its corrected map will look inverted. Use it to inspect any future "depth goes
to noise" bag without needing the robot.

Needs a depth server already listening (separate terminal, in the depth env):
    pixi run depth                 # runs server + live node; or just the server:
    env -u PYTHONPATH pixi run depth-server

Then, in the dev/default env (needs cv2 + rosbag2_py):
    pixi run python mote_perception/tools/depth_bag_replay.py <bag_dir> [--frames N] [--out DIR]
"""

import argparse
import sys
import tempfile

import cv2
import numpy as np

import bag_utils
from mote_perception.depth_wire import DEFAULT_PORT, DepthClient
from mote_perception.lidar_rescale import (
    LidarDepthRescaler,
    fit_affine_disparity_theilsen,
)

A_MIN = 0.5  # below this the lidar fit has collapsed to a degenerate/inverted line


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("bag", help="bag directory (mcap)")
    ap.add_argument("--frames", type=int, default=6, help="frames to sample, evenly")
    ap.add_argument(
        "--out", default=None, help="dir for colorized PNGs (default: temp)"
    )
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    args = ap.parse_args()
    out = args.out or tempfile.mkdtemp(prefix="depth_replay_")

    imgs, scans, tf_static, caminfo = bag_utils.load_perception_bag(args.bag)
    print(f"{len(imgs)} imgs, {len(scans)} scans; writing PNGs to {out}")

    # T_optical_scan = inv(T_base_optical) @ T_base_scan (camera and lidar are
    # siblings under base_footprint), built from the bag's static transforms.
    T_bo, T_bs = bag_utils.base_transforms(tf_static, scans)
    resc = LidarDepthRescaler(caminfo.k, caminfo.d, np.linalg.inv(T_bo) @ T_bs)

    client = DepthClient(args.host, args.port)
    if client.connect() is None:
        sys.exit("start the server with `pixi run depth-server` (or `pixi run depth`)")

    for k, i in enumerate(np.linspace(0, len(imgs) - 1, args.frames).astype(int)):
        ts, jpeg = imgs[i]
        depth = client.infer(jpeg)
        if depth is None:
            print(f"[f{k:>2}] no depth for this frame")
            continue
        _, scan = bag_utils.nearest_scan(scans, ts)
        pred, true = resc.pairs(scan, depth)
        rd = depth[np.isfinite(depth)]
        line = (
            f"[f{k:>2}] raw {rd.min():.2f}-{rd.max():.2f}m finite "
            f"{100 * np.isfinite(depth).mean():.0f}%  pairs {len(pred)}"
        )
        corr = None
        if len(pred) >= 8:
            a, b, frac = fit_affine_disparity_theilsen(pred, true)
            tag = "  DEGENERATE" if a < A_MIN else ""
            line += (
                f" spread {np.ptp(true):.2f}m  a={a:.3f} b={b:.3f} inl {frac:.0%}{tag}"
            )
            out_fit = resc.rescale(depth, scan)
            if out_fit is not None:
                corr = out_fit[0]
                cf = corr[np.isfinite(corr)]
                line += (
                    f"  -> corr {cf.min():.2f}-{cf.max():.2f}m "
                    f"med {np.median(cf):.2f} >20m {100 * (cf > 20).mean():.1f}%"
                )
        print(line)
        cv2.imwrite(f"{out}/f{k:02d}_raw.png", bag_utils.colorize(depth, 8.0))
        if corr is not None:
            cv2.imwrite(f"{out}/f{k:02d}_corr.png", bag_utils.colorize(corr, 5.0))


if __name__ == "__main__":
    main()
