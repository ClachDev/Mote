"""Unit tests for the lidar->depth affine rescale (lidar_rescale.py).

The rescale is fit fresh every frame from a thin band of lidar returns, so the
maths that has to stay right is: the Theil-Sen affine-in-disparity fit and its
robustness, the disparity-space application (its exact inverse on a clean fit),
scan->points filtering, the lidar<->image pairing geometry, and the guards that
make LidarDepthRescaler return None instead of a bad fit.
"""

import numpy as np
import pytest

from mote_perception.lidar_rescale import (
    LidarDepthRescaler,
    apply_affine_disparity,
    fit_affine_disparity_theilsen,
    lidar_depth_pairs,
    scan_to_points,
)

# Optical (z fwd, x right, y down) from lidar (x fwd, y left, z up):
#   opt_x = -lidar_y, opt_y = -lidar_z, opt_z = lidar_x
R_OPT_LIDAR = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])


def _T_opt_lidar():
    T = np.eye(4)
    T[:3, :3] = R_OPT_LIDAR
    return T


def _K(fx=200.0, fy=200.0, cx=160.0, cy=120.0):
    return np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])


class _Scan:
    def __init__(
        self, ranges, angle_min, angle_increment, range_min=0.1, range_max=8.0
    ):
        self.ranges = ranges
        self.angle_min = angle_min
        self.angle_increment = angle_increment
        self.range_min = range_min
        self.range_max = range_max


def test_fit_recovers_known_affine():
    rng = np.random.default_rng(0)
    a_true, b_true = 2.0, 0.1
    pred = rng.uniform(0.5, 5.0, size=200)
    # 1/true = a*(1/pred) + b  ->  true = 1 / (a/pred + b)
    true = 1.0 / (a_true / pred + b_true)
    a, b, frac = fit_affine_disparity_theilsen(pred, true)
    assert a == pytest.approx(a_true, rel=1e-6)
    assert b == pytest.approx(b_true, abs=1e-6)
    assert frac == pytest.approx(1.0)


def test_fit_is_robust_to_outliers():
    rng = np.random.default_rng(1)
    a_true, b_true = 1.5, 0.05
    pred = rng.uniform(0.5, 5.0, size=300)
    true = 1.0 / (a_true / pred + b_true)
    # Corrupt ~25% of the truths (below Theil-Sen's ~29% breakdown point).
    n_bad = 75
    true[:n_bad] = rng.uniform(0.5, 8.0, size=n_bad)
    a, b, frac = fit_affine_disparity_theilsen(pred, true)
    assert a == pytest.approx(a_true, rel=0.02)
    assert b == pytest.approx(b_true, abs=0.02)
    assert frac < 1.0  # the corrupted pairs are counted as outliers


def test_fit_too_few_points_returns_identity():
    result = fit_affine_disparity_theilsen(np.array([1.0, 2.0]), np.array([1.0, 2.0]))
    assert result == (1.0, 0.0, 0.0)


def test_apply_inverts_a_clean_fit():
    # Applying the fitted affine to pred must reproduce true exactly.
    a, b = 2.0, 0.1
    pred = np.array([0.5, 1.0, 2.0, 4.0])
    true = 1.0 / (a / pred + b)
    np.testing.assert_allclose(apply_affine_disparity(pred, a, b), true, rtol=1e-9)


def test_apply_identity_is_noop():
    depth = np.array([0.5, 1.0, 3.0, 7.0])
    np.testing.assert_allclose(
        apply_affine_disparity(depth, 1.0, 0.0), depth, rtol=1e-9
    )


def test_scan_to_points_geometry_and_filtering():
    ranges = [1.0, 2.0, np.inf, 0.05, 100.0, 3.0]
    pts = scan_to_points(
        ranges, angle_min=0.0, angle_increment=np.pi / 2, range_min=0.1, range_max=8.0
    )
    # inf, the 0.05 (< range_min) and 100.0 (> range_max) are dropped -> 3 kept.
    assert pts.shape == (3, 3)
    assert np.all(pts[:, 2] == 0.0)
    # angle 0 -> +x; the first kept point is range 1.0 straight ahead.
    np.testing.assert_allclose(pts[0], [1.0, 0.0, 0.0], atol=1e-12)
    # third kept point (index 5) is at angle 5*pi/2 == pi/2 -> +y, range 3.
    np.testing.assert_allclose(pts[2], [0.0, 3.0, 0.0], atol=1e-9)


def test_lidar_depth_pairs_on_axis_point():
    K, D = _K(), np.zeros(5)
    depth = np.full((240, 320), np.nan)
    depth[120, 160] = 4.0  # value at the principal point
    # A lidar point straight ahead maps onto the optical axis -> pixel (cx, cy).
    pts = np.array([[4.0, 0.0, 0.0]])
    pred, true = lidar_depth_pairs(pts, _T_opt_lidar(), depth, K, D)
    assert pred == pytest.approx([4.0])
    assert true == pytest.approx([4.0])


def test_lidar_depth_pairs_rejects_behind_and_out_of_fov():
    K, D = _K(), np.zeros(5)
    depth = np.zeros((240, 320)) + 5.0
    pts = np.array(
        [
            [-2.0, 0.0, 0.0],  # behind the camera (opt z < 0)
            [1.0, 5.0, 0.0],  # far to the side -> off image
        ]
    )
    pred, true = lidar_depth_pairs(pts, _T_opt_lidar(), depth, K, D)
    assert len(pred) == 0 and len(true) == 0


def test_rescaler_recovers_metric_depth():
    """End-to-end: a scan whose returns span a depth range, with a depth map
    that is the truth pushed through a known affine, must be corrected back to
    the true metric depth."""
    K, D, T = _K(), np.zeros(5), _T_opt_lidar()
    # A grid of beams; the finite ones span forward ranges 1.5..4 m.
    angles = np.linspace(-0.5, 0.5, 12)
    ranges = np.linspace(1.5, 4.0, 12)
    scan = _Scan(
        list(ranges), angle_min=angles[0], angle_increment=angles[1] - angles[0]
    )

    resc = LidarDepthRescaler(K, D, T)
    pts = scan_to_points(scan.ranges, scan.angle_min, scan.angle_increment, 0.1, 8.0)
    opt = pts @ T[:3, :3].T + T[:3, 3]
    u = np.round(K[0, 0] * opt[:, 0] / opt[:, 2] + K[0, 2]).astype(int)
    v = np.round(K[1, 1] * opt[:, 1] / opt[:, 2] + K[1, 2]).astype(int)
    assert len(np.unique(u)) == len(u)  # distinct columns -> no pixel collisions

    # 1/true = a*(1/pred) + b, so with b=0 the model depth is pred = a*true.
    a_true, b_true = 2.0, 0.0
    depth = np.full((240, 320), np.nan)
    true_depth = opt[:, 2]
    for ui, vi, td in zip(u, v, true_depth):
        depth[vi, ui] = a_true * td

    corrected, (a, b, frac) = resc.rescale(depth, scan)
    assert a == pytest.approx(a_true, rel=1e-6)
    assert b == pytest.approx(b_true, abs=1e-6)
    assert frac == pytest.approx(1.0)
    for ui, vi, td in zip(u, v, true_depth):
        assert corrected[vi, ui] == pytest.approx(td, rel=1e-6)


def test_rescale_returns_none_on_too_few_pairs():
    K, D, T = _K(), np.zeros(5), _T_opt_lidar()
    resc = LidarDepthRescaler(K, D, T, min_pairs=8)
    depth = np.full((240, 320), 3.0)
    scan = _Scan([2.0], angle_min=0.0, angle_increment=0.01)  # one return
    assert resc.rescale(depth, scan) is None
    assert resc.last_npairs < resc.min_pairs


def test_rescale_returns_none_on_too_little_spread():
    K, D, T = _K(), np.zeros(5), _T_opt_lidar()
    resc = LidarDepthRescaler(K, D, T, min_pairs=3, min_spread=0.3)
    depth = np.full((240, 320), 3.0)
    # Twenty returns all at the same range -> zero true-depth spread.
    scan = _Scan([2.0] * 20, angle_min=-0.3, angle_increment=0.03)
    assert resc.rescale(depth, scan) is None


def test_rescale_rejects_pathological_negative_slope():
    resc = LidarDepthRescaler(_K(), np.zeros(5), _T_opt_lidar(), a_min=0.05)
    # As pred disparity rises, true disparity falls: a well-formed but inverted
    # scan whose slope the a_min guard must reject.
    pred = np.array([1.0, 2.0, 3.0, 4.0])
    true = np.array([4.0, 3.0, 2.0, 1.0])
    a, _, _ = fit_affine_disparity_theilsen(pred, true)
    assert a < resc.a_min
