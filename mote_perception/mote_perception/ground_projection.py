"""Camera <-> base-frame geometry for the monocular obstacle detector.

The camera is rigidly mounted, so back-projection into the base frame and the
mapping to the floor plane (z=0 in base_footprint) are fixed functions of the
camera intrinsics and the static camera->base transform. This module computes
them once and is imported by both the offline bag harnesses and the live ROS
node, so the two cannot drift apart.

Conventions:
- Optical frame (REP-103): z forward, x right, y down.
- Base frame (base_footprint): x forward, y left, z up, floor at z=0.
- Returned ground points are in the base frame.
"""

import cv2
import numpy as np


def quat_to_matrix(x, y, z, w):
    """Rotation matrix (3x3) from a quaternion."""
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def transform_to_matrix(translation, rotation):
    """4x4 homogeneous matrix from a geometry_msgs Transform's parts."""
    m = np.eye(4)
    m[:3, :3] = quat_to_matrix(rotation.x, rotation.y, rotation.z, rotation.w)
    m[:3, 3] = [translation.x, translation.y, translation.z]
    return m


def chain_static_transforms(transforms, source, target):
    """Compose a chain of static TF transforms into a 4x4 matrix `target<-source`.

    `transforms` is the list from a tf2_msgs/TFMessage (e.g. /tf_static). Walks the
    parent links from `source` up to `target`, so the result maps a point expressed
    in `source` into `target`. Raises if no chain connects them.
    """
    parent_of = {t.child_frame_id: t.header.frame_id for t in transforms}
    mat_of = {
        t.child_frame_id: transform_to_matrix(
            t.transform.translation, t.transform.rotation
        )
        for t in transforms
    }
    m = np.eye(4)
    frame = source
    visited = []
    while frame != target:
        visited.append(frame)
        if frame not in parent_of:
            raise ValueError(
                f"no static TF chain from {source!r} to {target!r}; reached {frame!r} "
                f"(visited {visited})"
            )
        m = mat_of[frame] @ m
        frame = parent_of[frame]
    return m


def fit_ground_plane(
    xyz,
    band=0.15,
    max_range=2.0,
    thresh=0.04,
    min_inliers=60,
    min_xspread=0.5,
    max_pts=4000,
    iters=100,
    seed=0,
):
    """RANSAC fit of the floor plane ``z = a*x + b*y + c`` to near-floor points.

    Candidates are points near z=0 (|z| < band) and within max_range, i.e. close
    enough that a residual camera tilt has not yet lifted the floor out of the band.
    RANSAC rejects the obstacle bases that fall in that band. Returns
    (a, b, c, inlier_fraction), or None when the floor is too sparse or too narrow
    in x (forward) to constrain the pitch — the caller then holds its last good fit.
    """
    xyz = np.asarray(xyz, dtype=np.float64)
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    cand = np.isfinite(z) & (np.hypot(x, y) < max_range) & (np.abs(z) < band)
    if cand.sum() < min_inliers:
        return None
    x, y, z = x[cand], y[cand], z[cand]
    if np.ptp(x) < min_xspread:
        return None
    rng = np.random.default_rng(seed)
    if len(x) > max_pts:
        sel = rng.choice(len(x), max_pts, replace=False)
        x, y, z = x[sel], y[sel], z[sel]
    A = np.column_stack([x, y, np.ones(len(x))])
    best_inl, best_count = None, -1
    for _ in range(iters):
        idx = rng.integers(0, len(x), size=3)
        try:
            coef = np.linalg.solve(A[idx], z[idx])
        except np.linalg.LinAlgError:
            continue
        inl = np.abs(z - A @ coef) < thresh
        c = int(inl.sum())
        if c > best_count:
            best_count, best_inl = c, inl
    if best_count < min_inliers:
        return None
    coef, *_ = np.linalg.lstsq(A[best_inl], z[best_inl], rcond=None)
    return float(coef[0]), float(coef[1]), float(coef[2]), best_count / len(x)


def level_rotation(a, b):
    """Rotation (3x3) mapping the floor-plane normal ``(-a, -b, 1)`` onto +z.

    Applied as ``(pts - C) @ R.T + C`` it removes the residual camera tilt the plane
    reveals — the minimal rotation about ``normal x z``, so verticals stand up and the
    floor flattens together (the exact inverse of the tilt, not a z-only shear, which
    would leave the lean in x). Pitch and roll only; a floor cannot reveal yaw.
    """
    n = np.array([-a, -b, 1.0])
    n /= np.linalg.norm(n)
    v = np.cross(n, [0.0, 0.0, 1.0])
    s = float(np.linalg.norm(v))
    if s < 1e-9:
        return np.eye(3)
    angle = np.arctan2(s, n[2])
    R, _ = cv2.Rodrigues(v / s * angle)
    return R


class GroundProjector:
    """Fixed pixel<->floor mapping for a rigidly mounted camera.

    Build from the camera intrinsics (K, D, size) and the optical->base transform.
    The floor is the z=0 plane of the base frame.
    """

    def __init__(self, K, D, width, height, T_base_optical):
        self.K = np.asarray(K, dtype=np.float64).reshape(3, 3)
        self.D = np.asarray(D, dtype=np.float64).reshape(-1)
        self.width = int(width)
        self.height = int(height)
        self.T_base_optical = np.asarray(T_base_optical, dtype=np.float64)
        self.R = self.T_base_optical[:3, :3]
        self.C = self.T_base_optical[:3, 3]  # camera origin in base frame
        self._rays = None

    @classmethod
    def from_camera_info(cls, cam_info, T_base_optical):
        return cls(
            cam_info.k, cam_info.d, cam_info.width, cam_info.height, T_base_optical
        )

    def pixel_rays(self):
        """Undistorted unit-z optical-frame rays for every pixel, row-major (H*W, 3).

        Cached: the grid depends only on the intrinsics. Multiplying by a depth
        map's raveled values back-projects the whole frame; see `back_project`.
        """
        if self._rays is None:
            u, v = np.meshgrid(np.arange(self.width), np.arange(self.height))
            uv = np.column_stack([u.ravel(), v.ravel()]).astype(np.float64)
            norm = cv2.undistortPoints(uv.reshape(-1, 1, 2), self.K, self.D)
            norm = norm.reshape(-1, 2)
            self._rays = np.column_stack([norm[:, 0], norm[:, 1], np.ones(len(norm))])
        return self._rays

    def back_project(self, depth, stride=1):
        """Depth map -> (N, 3) base-frame points, in row-major pixel order.

        With stride > 1 every stride-th pixel is used (N = ceil(H*W/stride));
        stride 1 keeps the output index-aligned with the raveled pixel grid.
        """
        d = np.asarray(depth).ravel()[::stride]
        return (self.pixel_rays()[::stride] * d[:, None]) @ self.R.T + self.C

    def ground_to_pixels(self, xy):
        """Project floor points (base frame, z=0) back into the image.

        `xy` is (N, 2) base-frame (x, y). Returns (N, 2) pixel coords (may fall
        outside the image bounds; caller filters).
        """
        xy = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
        pts_base = np.column_stack([xy, np.zeros(len(xy))])
        rvec, _ = cv2.Rodrigues(self.R.T)  # base->optical rotation
        tvec = -self.R.T @ self.C
        img, _ = cv2.projectPoints(pts_base, rvec, tvec, self.K, self.D)
        return img.reshape(-1, 2)

    @property
    def camera_height(self):
        return float(self.C[2])
