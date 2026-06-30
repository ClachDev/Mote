"""Metric rescaling of monocular depth using the known floor plane.

Learned monocular depth (e.g. Depth Anything V2) is only metric up to a per-frame
affine ambiguity in disparity, and on Mote that ambiguity is not stable frame to
frame, so it must be re-solved every frame. The robot's fixed camera geometry gives
a dense, free ground-truth surface every frame: the pixels that fall on the floor
(z=0 in the base frame) have a known true depth from the camera. Fitting the
predicted depth to those gives the correction.

The fit is RANSAC so that obstacles sitting in the floor-seed region (precisely the
case we must detect) cannot corrupt the correction. Pure least-squares would be
dragged off exactly when it matters most.

Model (affine in disparity, the natural space for these networks):
    1 / true_depth = a * (1 / pred_depth) + b
"""

import numpy as np


def floor_seed_truth(proj, seed_rows=(0.80, 0.98), seed_cols=(0.30, 0.70), step=2):
    """Pixel coords of the floor-seed region and their TRUE optical-Z depth.

    Uses the fixed camera->base geometry: for each seed pixel, intersect its ray
    with the floor plane (z=0 in base) and return the optical-frame Z of that point.
    Returns (uv: (N,2) int, true_depth: (N,) float). Geometry-only — no image.
    """
    fx, fy = proj.K[0, 0], proj.K[1, 1]
    cx, cy = proj.K[0, 2], proj.K[1, 2]
    H, W = proj.height, proj.width
    vs = np.arange(int(seed_rows[0] * H), int(seed_rows[1] * H), step)
    us = np.arange(int(seed_cols[0] * W), int(seed_cols[1] * W), step)
    uu, vv = np.meshgrid(us, vs)
    uu, vv = uu.ravel(), vv.ravel()
    n = np.column_stack([(uu - cx) / fx, (vv - cy) / fy, np.ones(len(uu))])
    denom = n @ proj.R[2, :]
    t = -proj.C[2] / denom  # optical-Z of the floor point under each pixel
    ok = (denom < 0) & (t > 0.15) & (t < 5.0)
    return np.column_stack([uu[ok], vv[ok]]), t[ok]


def fit_affine_disparity(pred, true, iters=200, inlier_thresh=0.08, seed=0):
    """RANSAC fit of 1/true = a*(1/pred) + b. Returns (a, b, inlier_fraction).

    inlier_thresh is in disparity units (1/m). Robust to a minority of the seed
    pixels actually being on an obstacle (outliers in the up direction). Maximizing
    inlier count tolerates a large outlier fraction in one direction, which is what
    the floor seed needs; for the lidar pairs (scatter in both directions) it is
    multimodal -- use fit_affine_disparity_theilsen there instead.
    """
    p = 1.0 / np.maximum(pred, 1e-3)
    q = 1.0 / np.maximum(true, 1e-3)
    n = len(p)
    if n < 10:
        A = np.column_stack([p, np.ones(n)])
        sol, *_ = np.linalg.lstsq(A, q, rcond=None)
        return float(sol[0]), float(sol[1]), 1.0

    rng = np.random.default_rng(seed)
    best_inliers = None
    best_count = -1
    for _ in range(iters):
        i, j = rng.integers(0, n, size=2)
        if p[i] == p[j]:
            continue
        a = (q[i] - q[j]) / (p[i] - p[j])
        b = q[i] - a * p[i]
        resid = np.abs(q - (a * p + b))
        inl = resid < inlier_thresh
        c = int(inl.sum())
        if c > best_count:
            best_count, best_inliers = c, inl

    A = np.column_stack([p[best_inliers], np.ones(best_count)])
    sol, *_ = np.linalg.lstsq(A, q[best_inliers], rcond=None)
    return float(sol[0]), float(sol[1]), best_count / n


def fit_affine_disparity_theilsen(pred, true, inlier_thresh=0.08):
    """Robust regression of 1/true = a*(1/pred) + b via Theil-Sen.

    The median of all pairwise slopes, then the median intercept -- a unique, stable
    central estimate. Maximizing inlier *count* (fit_affine_disparity) is multimodal
    on the lidar pairs: their two-sided scatter lets a steeper or a shallower line
    catch nearly the same count, so a count-RANSAC flips between them frame to frame
    even on a static scene (the depth flickers). The median-of-slopes has no such
    ambiguity. Deterministic (full pairwise, no sampling) and robust to ~29% outliers.
    Returns (a, b, inlier_fraction).
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


class DepthFloorRescaler:
    """Per-frame metric rescaling of a depth map via the floor plane.

    Caches the geometry-only floor seed (constant for a fixed mount). `rescale`
    fits the correction on the current frame's floor pixels (RANSAC) and applies
    it. An optional EMA on (a, b) damps frame-to-frame jitter; the raw fit is
    returned too so a caller can reject low-inlier frames.
    """

    def __init__(self, proj, ema=0.0, **seed_kw):
        self.proj = proj
        self.uv, self.true = floor_seed_truth(proj, **seed_kw)
        self.ema = ema
        self._ab = None

    def rescale(self, depth):
        pred = depth[self.uv[:, 1], self.uv[:, 0]]
        a, b, frac = fit_affine_disparity(pred, self.true)
        if self.ema and self._ab is not None:
            a = self.ema * self._ab[0] + (1 - self.ema) * a
            b = self.ema * self._ab[1] + (1 - self.ema) * b
        self._ab = (a, b)
        return apply_affine_disparity(depth, a, b), (a, b, frac)
