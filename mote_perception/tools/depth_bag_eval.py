"""Evaluate a depth model's accuracy and speed against a recorded bag.

Model-agnostic: it talks to whatever depth server is listening on --port (V2 now,
a V3 server later), so the same harness compares models by pointing it at each.
For sampled frames it measures, against the time-nearest lidar scan as ground truth:

  accuracy  -- after the best affine-in-disparity alignment of the model depth to the
               lidar returns (fit on half the pairs, scored on the other half), the
               AbsRel / RMSE / delta<1.25 at the lidar pixels. This is the model's
               intrinsic shape/scale fidelity in the lidar band, independent of our
               per-frame rescale and comparable across models.
  speed     -- server round-trip latency per frame.

And it saves, per frame, images to inspect by eye:
  *_depth.png -- colorized model depth with the lidar returns overprinted as dots
                 (colored by their true range) so model-vs-lidar agreement is visible.
  *_side.png  -- side elevation (forward range x vs height z) of the back-projected
                 cloud: a real vertical edge (chair leg) should stand vertical; if it
                 leans into the distance, the slant is in the depth, not the geometry.
  *_bev.png   -- top-down (x forward, y left) of the cloud with the lidar scan overlaid.

The lidar band only samples one height, so the numbers grade scale/shape there; the
side/BEV views are how you judge the vertical slant. Run with a server up (see
depth_bag_replay.py), in the dev/default env:
    pixi run python mote_perception/tools/depth_bag_eval.py <bag> --label V2 [--frames N]
"""

import argparse
import os
import tempfile
import time

import cv2
import numpy as np

from mote_perception.depth_rescale import (
    apply_affine_disparity,
    fit_affine_disparity_theilsen,
)
from mote_perception.ground_projection import (
    GroundProjector,
    chain_static_transforms,
    fit_ground_plane,
    level_rotation,
)
from mote_perception.lidar_rescale import LidarDepthRescaler, scan_to_points

# sibling tool (same dir is on sys.path when run as a script): bag loader + server client
import depth_bag_replay as rep


def _err(pred, true):
    return (
        float(np.mean(np.abs(pred - true) / true)),
        float(np.mean(np.maximum(pred / true, true / pred) < 1.25)),
    )


def _accuracy(pred, true):
    """(raw AbsRel, raw d1, aligned AbsRel, aligned d1) vs lidar.

    raw = model depth as-is (absolute accuracy; only meaningful for a metric model,
    answers 'does it need rescaling'). aligned = after the best affine in disparity,
    fit on half the pairs and scored on the other (relative shape, model-agnostic).
    """
    if len(pred) < 12:
        return None
    raw_ar, raw_d1 = _err(pred, true)
    a, b, _ = fit_affine_disparity_theilsen(
        pred[::2], true[::2]
    )  # interleave train/test
    al_ar, al_d1 = _err(apply_affine_disparity(pred[1::2], a, b), true[1::2])
    return raw_ar, raw_d1, al_ar, al_d1


def _cloud(depth, proj, rays_opt, stride=4):
    d = depth.ravel()[::stride]
    pts = (rays_opt[::stride] * d[:, None]) @ proj.R.T + proj.C
    ok = np.isfinite(pts).all(1)
    return pts[ok]


def _scatter(xs, ys, xlim, ylim, size=(360, 480), flipy=True):
    """Rasterize points into a canvas; returns BGR image. xlim/ylim in metres."""
    h, w = size
    img = np.full((h, w, 3), 30, np.uint8)
    u = ((xs - xlim[0]) / (xlim[1] - xlim[0]) * (w - 1)).astype(int)
    v = ((ys - ylim[0]) / (ylim[1] - ylim[0]) * (h - 1)).astype(int)
    if flipy:
        v = h - 1 - v
    m = (u >= 0) & (u < w) & (v >= 0) & (v < h)
    img[v[m], u[m]] = (200, 200, 200)
    return img


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("bag")
    ap.add_argument("--label", default="model", help="tag for output + summary")
    ap.add_argument("--frames", type=int, default=8)
    ap.add_argument("--out", default=None)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5601)
    args = ap.parse_args()
    out = args.out or tempfile.mkdtemp(prefix=f"depth_eval_{args.label}_")
    os.makedirs(out, exist_ok=True)

    imgs, scans, tf_static, caminfo = rep.load(args.bag)
    T_bo = chain_static_transforms(
        tf_static.transforms, "camera_optical_link", "base_footprint"
    )
    T_bs = chain_static_transforms(
        tf_static.transforms, "lidar_scan_link", "base_footprint"
    )
    resc = LidarDepthRescaler(caminfo.k, caminfo.d, np.linalg.inv(T_bo) @ T_bs)
    proj = GroundProjector.from_camera_info(caminfo, T_bo)
    H, W = proj.height, proj.width
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    uv = np.column_stack([u.ravel(), v.ravel()]).astype(np.float64).reshape(-1, 1, 2)
    norm = cv2.undistortPoints(uv, proj.K, proj.D).reshape(-1, 2)
    rays_opt = np.column_stack([norm[:, 0], norm[:, 1], np.ones(len(norm))])

    lat, acc = [], []
    print(f"{args.label}: {len(imgs)} imgs; writing to {out}")
    for k, i in enumerate(np.linspace(0, len(imgs) - 1, args.frames).astype(int)):
        ts, jpeg = imgs[i]
        t0 = time.perf_counter()
        depth = rep.infer(jpeg, args.host, args.port)
        lat.append((time.perf_counter() - t0) * 1000)
        _, scan = min(scans, key=lambda s: abs(s[0] - ts))
        pred, true = resc.pairs(scan, depth)
        m = _accuracy(pred, true)
        if m:
            acc.append(m)
        # rescale via our pipeline for the cloud views
        a, b, _ = fit_affine_disparity_theilsen(pred, true)
        corr = apply_affine_disparity(depth, a, b)

        # depth map + lidar overlay (project lidar points, color by true range)
        dimg = rep.colorize(depth, 8.0)
        pts_l = scan_to_points(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
        )
        po = pts_l @ resc.T[:3, :3].T + resc.T[:3, 3]
        front = po[:, 2] > 0.05
        if front.any():
            px = cv2.projectPoints(
                po[front].reshape(-1, 1, 3), np.zeros(3), np.zeros(3), proj.K, proj.D
            )[0].reshape(-1, 2)
            rng = po[front, 2]
            cols = rep.colorize(rng.reshape(1, -1), 8.0).reshape(-1, 3)
            for (xp, yp), c in zip(px, cols):
                if 0 <= xp < W and 0 <= yp < H:
                    cv2.circle(dimg, (int(xp), int(yp)), 3, [int(x) for x in c], -1)
        cv2.imwrite(f"{out}/{args.label}_f{k:02d}_depth.png", dimg)

        cloud = _cloud(corr, proj, rays_opt)
        cv2.imwrite(
            f"{out}/{args.label}_f{k:02d}_side.png",
            _scatter(cloud[:, 0], cloud[:, 2], (0, 4), (-0.1, 1.6)),
        )
        # plane-levelled side view: a real vertical should now stand vertical
        fitp = fit_ground_plane(cloud)
        if fitp is not None:
            lvl = (cloud - proj.C) @ level_rotation(fitp[0], fitp[1]).T + proj.C
            cv2.imwrite(
                f"{out}/{args.label}_f{k:02d}_side_lvl.png",
                _scatter(lvl[:, 0], lvl[:, 2], (0, 4), (-0.1, 1.6)),
            )
        cv2.imwrite(
            f"{out}/{args.label}_f{k:02d}_bev.png",
            _scatter(cloud[:, 1], cloud[:, 0], (-2, 2), (0, 4)),
        )

    lat = np.array(lat)
    print(f"\n=== {args.label} ===")
    print(
        f"latency: mean {lat.mean():.0f} ms  median {np.median(lat):.0f}  "
        f"min {lat.min():.0f}  max {lat.max():.0f}"
    )
    if acc:
        a = np.array(acc)
        print(
            f"accuracy vs lidar:  raw   AbsRel {a[:, 0].mean():.3f}  "
            f"delta1 {100 * a[:, 1].mean():.1f}%   (does it need rescaling?)"
        )
        print(
            f"                    align AbsRel {a[:, 2].mean():.3f}  "
            f"delta1 {100 * a[:, 3].mean():.1f}%   (relative shape, held-out)"
        )


if __name__ == "__main__":
    main()
