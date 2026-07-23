"""Unit tests for camera<->floor geometry (ground_projection.py).

Covers the quaternion/transform helpers, the static-TF chain walk, the RANSAC
floor-plane fit and its None guards, the tilt-removal rotation, and the
GroundProjector pixel<->floor round trip.
"""

from types import SimpleNamespace

import numpy as np
import pytest

from mote_perception.ground_projection import (
    GroundProjector,
    chain_static_transforms,
    fit_ground_plane,
    level_rotation,
    quat_to_matrix,
    transform_to_matrix,
)


def _quat_from_axis_angle(axis, angle):
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    s = np.sin(angle / 2.0)
    return (*(axis * s), np.cos(angle / 2.0))


def test_quat_to_matrix_identity():
    np.testing.assert_allclose(quat_to_matrix(0, 0, 0, 1), np.eye(3), atol=1e-12)


def test_quat_to_matrix_90_about_z():
    x, y, z, w = _quat_from_axis_angle([0, 0, 1], np.pi / 2)
    R = quat_to_matrix(x, y, z, w)
    # +x rotates to +y.
    np.testing.assert_allclose(R @ [1, 0, 0], [0, 1, 0], atol=1e-9)
    np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-9)
    assert np.linalg.det(R) == pytest.approx(1.0)


def test_quat_to_matrix_normalizes_input():
    # An unnormalized quaternion is normalized internally -> still a rotation.
    # (0,0,2,2) normalizes to (0,0,1/√2,1/√2), i.e. 90 deg about z.
    R = quat_to_matrix(0, 0, 2.0, 2.0)
    np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-9)
    np.testing.assert_allclose(R @ [1, 0, 0], [0, 1, 0], atol=1e-9)


def test_transform_to_matrix():
    trans = SimpleNamespace(x=1.0, y=2.0, z=3.0)
    rot = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
    m = transform_to_matrix(trans, rot)
    np.testing.assert_allclose(m[:3, :3], np.eye(3), atol=1e-12)
    np.testing.assert_allclose(m[:3, 3], [1, 2, 3])
    np.testing.assert_allclose(m[3], [0, 0, 0, 1])


def _tf(child, parent, txyz=(0, 0, 0), quat=(0, 0, 0, 1)):
    return SimpleNamespace(
        child_frame_id=child,
        header=SimpleNamespace(frame_id=parent),
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=txyz[0], y=txyz[1], z=txyz[2]),
            rotation=SimpleNamespace(x=quat[0], y=quat[1], z=quat[2], w=quat[3]),
        ),
    )


def test_chain_static_transforms_composes_translations():
    # camera <- mount <- base, each a pure translation.
    tfs = [
        _tf("mount", "base", txyz=(1.0, 0.0, 0.0)),
        _tf("camera", "mount", txyz=(0.0, 0.0, 0.5)),
    ]
    m = chain_static_transforms(tfs, source="camera", target="base")
    # A point at the camera origin sits at base (1.0, 0, 0.5).
    np.testing.assert_allclose(m @ [0, 0, 0, 1], [1.0, 0.0, 0.5, 1.0], atol=1e-9)


def test_chain_static_transforms_source_equals_target_is_identity():
    tfs = [_tf("mount", "base", txyz=(1.0, 0.0, 0.0))]
    np.testing.assert_allclose(
        chain_static_transforms(tfs, "base", "base"), np.eye(4), atol=1e-12
    )


def test_chain_static_transforms_no_chain_raises():
    tfs = [_tf("mount", "base")]
    with pytest.raises(ValueError, match="no static TF chain"):
        chain_static_transforms(tfs, source="camera", target="base")


def _floor_points(a, b, c, n=400, seed=0):
    rng = np.random.default_rng(seed)
    x = rng.uniform(0.2, 1.8, n)  # forward spread comfortably > min_xspread
    y = rng.uniform(-1.0, 1.0, n)
    z = a * x + b * y + c
    return np.column_stack([x, y, z])


def test_fit_ground_plane_recovers_coefficients():
    a, b, c = 0.02, -0.01, 0.0
    xyz = _floor_points(a, b, c)
    fa, fb, fc, frac = fit_ground_plane(xyz, seed=0)
    assert fa == pytest.approx(a, abs=1e-3)
    assert fb == pytest.approx(b, abs=1e-3)
    assert fc == pytest.approx(c, abs=1e-3)
    assert frac == pytest.approx(1.0, abs=1e-6)


def test_fit_ground_plane_rejects_obstacle_outliers():
    a, b, c = 0.01, 0.0, 0.0
    floor = _floor_points(a, b, c, n=300, seed=1)
    # Low obstacle bases inside the |z| < band window but off the plane.
    rng = np.random.default_rng(2)
    obs = np.column_stack(
        [rng.uniform(0.2, 1.8, 80), rng.uniform(-1, 1, 80), np.full(80, 0.12)]
    )
    xyz = np.vstack([floor, obs])
    fa, fb, fc, frac = fit_ground_plane(xyz, seed=0)
    assert fa == pytest.approx(a, abs=2e-3)
    assert fb == pytest.approx(b, abs=2e-3)
    assert fc == pytest.approx(c, abs=2e-3)
    assert frac < 1.0  # the obstacle points are outliers


def test_fit_ground_plane_none_when_too_sparse():
    xyz = _floor_points(0.0, 0.0, 0.0, n=20)  # < min_inliers
    assert fit_ground_plane(xyz) is None


def test_fit_ground_plane_none_when_narrow_in_x():
    rng = np.random.default_rng(0)
    x = rng.uniform(0.9, 1.0, 200)  # x spread ~0.1 < min_xspread
    y = rng.uniform(-1, 1, 200)
    xyz = np.column_stack([x, y, np.zeros(200)])
    assert fit_ground_plane(xyz) is None


def test_level_rotation_maps_normal_to_z():
    a, b = 0.03, -0.02
    R = level_rotation(a, b)
    n = np.array([-a, -b, 1.0])
    n /= np.linalg.norm(n)
    np.testing.assert_allclose(R @ n, [0, 0, 1], atol=1e-9)
    np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-9)


def test_level_rotation_flat_floor_is_identity():
    np.testing.assert_allclose(level_rotation(0.0, 0.0), np.eye(3), atol=1e-12)


def test_level_rotation_is_pitch_roll_only_no_yaw():
    # The minimal tilt-removal must not spin the floor about z: a forward vector
    # keeps its heading (y stays ~0) apart from the pitch lift.
    R = level_rotation(0.05, 0.0)  # pure pitch
    fwd = R @ [1.0, 0.0, 0.0]
    assert fwd[1] == pytest.approx(0.0, abs=1e-9)


# Optical (z fwd, x right, y down) expressed in base (x fwd, y left, z up):
#   opt_x(right) -> base -y, opt_y(down) -> base -z, opt_z(fwd) -> base +x
R_BASE_OPT = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def _projector(height=0.3):
    K = np.array([[200.0, 0, 160.0], [0, 200.0, 120.0], [0, 0, 1.0]])
    T = np.eye(4)
    T[:3, :3] = R_BASE_OPT
    T[:3, 3] = [0.0, 0.0, height]
    return GroundProjector(K, np.zeros(5), 320, 240, T), K, T


def test_projector_camera_height():
    proj, _, _ = _projector(0.42)
    assert proj.camera_height == pytest.approx(0.42)


def test_pixel_rays_center_is_forward():
    proj, K, _ = _projector()
    rays = proj.pixel_rays()
    assert rays.shape == (320 * 240, 3)
    cx, cy = int(K[0, 2]), int(K[1, 2])
    center = rays[cy * 320 + cx]
    np.testing.assert_allclose(center, [0, 0, 1], atol=1e-6)


def test_back_project_center_ray():
    proj, _, T = _projector(0.3)
    # Depth map with a single non-trivial pixel at the principal point.
    depth = np.zeros((240, 320))
    depth[120, 160] = 2.0
    pts = proj.back_project(depth)
    p = pts[120 * 320 + 160]
    # Optical (0,0,2) -> base: R_BASE_OPT @ (0,0,2) + C = (2,0,0)+(0,0,0.3).
    np.testing.assert_allclose(p, [2.0, 0.0, 0.3], atol=1e-6)


def test_ground_pixel_round_trip():
    """A floor point projected to a pixel and back-projected at its true optical
    depth returns to the same floor point."""
    proj, K, T = _projector(0.3)
    floor_xy = np.array([2.0, 0.3])
    px = proj.ground_to_pixels(floor_xy).reshape(2)
    u, v = int(round(px[0])), int(round(px[1]))
    assert 0 <= u < 320 and 0 <= v < 240

    # True optical depth of that floor point.
    R, C = T[:3, :3], T[:3, 3]
    p_base = np.array([floor_xy[0], floor_xy[1], 0.0])
    depth_val = (R.T @ (p_base - C))[2]

    depth = np.zeros((240, 320))
    depth[v, u] = depth_val
    recovered = proj.back_project(depth)[v * 320 + u]
    np.testing.assert_allclose(recovered, p_base, atol=2e-2)


def test_ground_to_pixels_below_horizon():
    # The floor is beneath the camera, so ground points image in the lower half.
    proj, K, _ = _projector()
    px = proj.ground_to_pixels([2.0, 0.0]).reshape(2)
    assert px[1] > K[1, 2]  # v below the principal point


# --- pixels_to_ground: floor-ray grounding used by the L2 detector -----------
# A dedicated forward-level projector (camera at ~robot height, optical axis
# level, looking along +x) to exercise pixels_to_ground against known geometry
# and against ground_to_pixels.


def _forward_level_projector(height=0.10):
    K_flat = [200.0, 0.0, 320.0, 0.0, 200.0, 240.0, 0.0, 0.0, 1.0]
    D = [0.0] * 5
    T = np.array(
        [
            [0.0, 0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0, 0.0],
            [0.0, -1.0, 0.0, height],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    return GroundProjector(K_flat, D, 640, 480, T)


def test_pixels_to_ground_known_floor_point():
    proj = _forward_level_projector(height=0.10)
    # A pixel 20 rows below the principal point looks down atan(20/200) from
    # level; from 0.10 m up that ray meets the floor at x = 0.1 * 200 / 20 = 1 m.
    pt = proj.pixels_to_ground([[320.0, 260.0]])[0]
    assert pt == pytest.approx([1.0, 0.0, 0.0], abs=1e-9)


def test_pixels_to_ground_horizon_is_nan():
    proj = _forward_level_projector()
    # At or above the principal row the ray never descends to the floor.
    above, at = proj.pixels_to_ground([[320.0, 220.0], [320.0, 240.0]])
    assert np.isnan(above).all()
    assert np.isnan(at).all()


def test_pixels_to_ground_round_trip_with_ground_to_pixels():
    proj = _forward_level_projector(height=0.10)
    xy = np.array([[0.5, 0.1], [1.0, -0.2], [2.0, 0.0]])
    uv = proj.ground_to_pixels(xy)
    back = proj.pixels_to_ground(uv)
    assert back[:, :2] == pytest.approx(xy, abs=1e-6)
    assert back[:, 2] == pytest.approx(0.0, abs=1e-9)
