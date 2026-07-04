"""Metric rescaling of monocular depth against lidar range returns.

The floor-plane rescale (depth_rescale.py) anchors scale on a narrow near-floor
band and extrapolates out to far walls, and it depends on the camera->floor angle
-- which varies with the floor slope and how the robot rests (measured ~1.5 deg
across rest positions). The lidar instead gives metric range on the surfaces the
depth must get right (walls), referenced through the lidar->camera transform. Both
sensors are bolted to the chassis, so that transform is invariant to chassis tilt:
the scale it yields is independent of how the body or floor is pitched.

A 2D scan samples a single height -- a thin curved band in the image -- but the
correction is a global affine in disparity (the same model the floor fit uses), so
fitting on that band and applying everywhere is valid as long as the band spans a
range of depths. When it doesn't (too few returns in view, or all at one range),
`rescale` returns None and the caller holds the last good fit.

The affine is fit by robust regression (Theil-Sen), not by maximizing inlier count.
The pairs scatter to both sides of the true line, so a count objective is multimodal
-- a steeper or shallower line catches nearly the same count -- and a count-RANSAC
flips between those solutions frame to frame, so the depth flickers even on a static
scene. The median-of-slopes is a unique central estimate with no such ambiguity, and
is naturally positive (no inverted line) with intercept ~0 (no blow-up); `a_min` is
only a defensive reject for a pathological (inverted or near-flat) scan.

`a` is the slope mapping model disparity onto true disparity, so its magnitude
absorbs the model's arbitrary disparity units -- the relative (SSI) model outputs
disparity on a scale where a valid fit lands near 0.25-0.5, an order of magnitude
below the metric model it replaced. `a_min` therefore guards only the sign/near-flat
degeneracy (a <= ~0), not an absolute scale.
"""

import cv2
import numpy as np

from mote_perception.depth_rescale import (
    apply_affine_disparity,
    fit_affine_disparity_theilsen,
)


def scan_to_points(ranges, angle_min, angle_increment, range_min, range_max):
    """LaserScan ranges -> (N, 3) points in the lidar frame (z=0)."""
    r = np.asarray(ranges, dtype=np.float64)
    ang = angle_min + np.arange(len(r)) * angle_increment
    ok = np.isfinite(r) & (r > range_min) & (r < range_max)
    r, ang = r[ok], ang[ok]
    return np.column_stack([r * np.cos(ang), r * np.sin(ang), np.zeros(len(r))])


def lidar_depth_pairs(pts_lidar, T_opt_lidar, depth, K, D):
    """Pair model depth with true range where lidar points land in the image.

    Transforms lidar points into the optical frame, projects them through the
    intrinsics, and for those falling in front of the camera and inside the image
    returns (pred, true) optical-Z depths -- model prediction vs. metric truth.
    """
    R = np.asarray(T_opt_lidar)[:3, :3]
    t = np.asarray(T_opt_lidar)[:3, 3]
    pts_opt = pts_lidar @ R.T + t
    z = pts_opt[:, 2]
    front = z > 0.05
    pts_opt, z = pts_opt[front], z[front]
    if len(pts_opt) == 0:
        return np.empty(0), np.empty(0)

    # Reject off-axis returns before distortion can fold them back into bounds.
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    h, w = depth.shape
    xn = pts_opt[:, 0] / z
    yn = pts_opt[:, 1] / z
    u_und = fx * xn + cx
    v_und = fy * yn + cy
    in_fov = (u_und >= 0) & (u_und < w) & (v_und >= 0) & (v_und < h)
    pts_opt, z = pts_opt[in_fov], z[in_fov]
    if len(pts_opt) == 0:
        return np.empty(0), np.empty(0)

    px, _ = cv2.projectPoints(pts_opt.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, D)
    px = px.reshape(-1, 2)
    u = np.round(px[:, 0]).astype(int)
    v = np.round(px[:, 1]).astype(int)
    inb = (u >= 0) & (u < w) & (v >= 0) & (v < h)
    u, v, z = u[inb], v[inb], z[inb]
    pred = depth[v, u]
    valid = np.isfinite(pred) & (pred > 1e-3)
    return pred[valid], z[valid]


class LidarDepthRescaler:
    """Per-frame metric rescaling of a depth map via lidar range returns.

    Holds the intrinsics and the static lidar->optical transform. `rescale` builds
    the (pred, true) pairs from one scan, fits the affine-in-disparity correction
    (Theil-Sen), and applies it. Returns None when the
    scan gives too few pairs or too little depth spread to constrain the fit.
    """

    def __init__(
        self,
        K,
        D,
        T_opt_lidar,
        range_min=0.1,
        range_max=8.0,
        min_pairs=8,
        min_spread=0.3,
        a_min=0.05,
    ):
        self.K = np.asarray(K, np.float64).reshape(3, 3)
        self.D = np.asarray(D, np.float64).reshape(-1)
        self.T = np.asarray(T_opt_lidar, np.float64)
        self.range_min = range_min
        self.range_max = range_max
        self.min_pairs = min_pairs
        self.min_spread = min_spread
        self.a_min = a_min
        self.last_npairs = 0

    def pairs(self, scan, depth):
        pts = scan_to_points(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            max(scan.range_min, self.range_min),
            min(scan.range_max, self.range_max),
        )
        return lidar_depth_pairs(pts, self.T, depth, self.K, self.D)

    def rescale(self, depth, scan):
        pred, true = self.pairs(scan, depth)
        self.last_npairs = len(pred)
        if len(pred) < self.min_pairs or np.ptp(true) < self.min_spread:
            return None
        a, b, frac = fit_affine_disparity_theilsen(pred, true)
        if a < self.a_min:  # pathological scan; let the caller hold last-good
            return None
        return apply_affine_disparity(depth, a, b), (a, b, frac)
