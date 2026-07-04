"""Metric rescaling of monocular depth against lidar range returns.

Learned monocular depth (Depth Anything V2) is only metric up to a per-frame
affine ambiguity in disparity -- 1/true = a * (1/pred) + b, the natural space for
these networks -- and the ambiguity is not stable frame to frame, so it must be
re-solved every frame. The lidar gives metric range on the surfaces the depth
must get right (walls), referenced through the lidar->camera transform. Both
sensors are bolted to the chassis, so that transform is invariant to chassis
tilt: the scale it yields is independent of how the body or floor is pitched.
(An earlier floor-plane anchor was rejected for exactly that reason: it fit a
narrow near-floor band, extrapolated out to the walls, and shifted with the
camera->floor angle -- measured ~1.5 deg across rest positions.)

A 2D scan samples a single height -- a thin curved band in the image -- but the
correction is a global affine in disparity, so fitting on that band and applying
everywhere is valid as long as the band spans a range of depths. When it doesn't
(too few returns in view, or all at one range), `rescale` returns None and the
caller holds the last good fit.

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


def fit_affine_disparity_theilsen(pred, true, inlier_thresh=0.08):
    """Robust regression of 1/true = a*(1/pred) + b via Theil-Sen.

    The median of all pairwise slopes, then the median intercept -- a unique,
    stable central estimate. Deterministic (full pairwise, no sampling) and
    robust to ~29% outliers. Returns (a, b, inlier_fraction); inlier_thresh is
    in disparity units (1/m).
    """
    p = 1.0 / np.maximum(pred, 1e-3)
    q = 1.0 / np.maximum(true, 1e-3)
    n = len(p)
    if n < 3:
        return 1.0, 0.0, 0.0
    i, j = np.triu_indices(n, 1)
    dp = p[j] - p[i]
    nz = dp != 0
    a = float(np.median((q[j] - q[i])[nz] / dp[nz]))
    b = float(np.median(q - a * p))
    frac = float(np.mean(np.abs(q - (a * p + b)) < inlier_thresh))
    return a, b, frac


def apply_affine_disparity(depth, a, b):
    """Apply the disparity-space correction to a depth map (metres)."""
    disp = a / np.maximum(depth, 1e-3) + b
    return 1.0 / np.maximum(disp, 1e-3)


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
