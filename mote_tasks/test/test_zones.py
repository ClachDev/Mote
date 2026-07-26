import math

import pytest

from mote_tasks.zones import (
    Polygon,
    append_zone,
    containing,
    load_zones,
    yaw_from_quaternion,
)

# An L: the unit-ish square [0, 4] x [0, 4] with its north-east quadrant
# removed, so the area centroid (1.667, 1.667) is inside but a naive bounding
# box or circle would claim the missing corner.
L_SHAPE = Polygon(((0, 0), (4, 0), (4, 2), (2, 2), (2, 4), (0, 4)))

# A U opening north: the centroid falls in the notch, outside the polygon.
U_SHAPE = Polygon(((0, 0), (6, 0), (6, 6), (4, 6), (4, 2), (2, 2), (2, 6), (0, 6)))


def test_yaw_from_quaternion_round_trip():
    for yaw in (-3.0, -1.5, 0.0, 0.7, 3.1):
        z, w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        assert yaw_from_quaternion(0.0, 0.0, z, w) == pytest.approx(yaw)


def test_append_zone_creates_replaces_and_loads(tmp_path):
    path = tmp_path / "zones.yaml"
    assert append_zone(path, "bin", 1.0, -2.0, 0.5) is False
    assert append_zone(path, "sofa", 0.25, 0.0, -1.0) is False
    assert append_zone(path, "bin", 1.5, -2.5, 0.5) is True

    zones = load_zones(str(path))
    assert sorted(zones) == ["bin", "sofa"]
    assert zones["bin"].pose.pose.position.x == pytest.approx(1.5)
    assert zones["bin"].pose.header.frame_id == "map"
    assert zones["bin"].footprint is None
    yaw = yaw_from_quaternion(
        0.0,
        0.0,
        zones["sofa"].pose.pose.orientation.z,
        zones["sofa"].pose.pose.orientation.w,
    )
    assert yaw == pytest.approx(-1.0)


def test_append_zone_with_radius_gives_footprint(tmp_path):
    path = tmp_path / "zones.yaml"
    append_zone(path, "kitchen", 2.0, 3.0, 0.0, radius=1.5)
    zones = load_zones(str(path))
    fp = zones["kitchen"].footprint
    assert fp is not None
    assert fp.radius == pytest.approx(1.5)
    assert fp.contains(2.4, 3.0) is True  # 0.4 m from centre
    assert fp.contains(4.0, 3.0) is False  # 2.0 m from centre


def test_load_zones_missing_coord_raises(tmp_path):
    path = tmp_path / "zones.yaml"
    path.write_text("zones:\n  bad: {x: 1.0}\n")
    with pytest.raises(ValueError):
        load_zones(str(path))


def test_containing_nearest_first(tmp_path):
    path = tmp_path / "zones.yaml"
    path.write_text(
        "zones:\n"
        "  pickup: {x: 0.0, y: 0.0}\n"  # no footprint -> never matches
        "  big:    {x: 0.0, y: 0.0, radius: 5.0}\n"
        "  small:  {x: 1.0, y: 0.0, radius: 2.0}\n"
        "  far:    {x: 20.0, y: 20.0, radius: 1.0}\n"
    )
    zones = load_zones(str(path))
    # (1.2, 0) is inside big (dist 1.2) and small (dist 0.2), nearest first;
    # pickup has no footprint so it never appears.
    assert containing(zones, 1.2, 0.0) == ["small", "big"]
    assert containing(zones, 10.0, 0.0) == []  # outside every footprint
    assert containing(zones, 0.0, 5.0) == ["big"]  # exactly on the radius counts


def test_polygon_contains_convex():
    square = Polygon(((0, 0), (2, 0), (2, 2), (0, 2)))
    assert square.contains(1.0, 1.0) is True
    assert square.contains(3.0, 1.0) is False
    assert square.contains(1.0, -0.001) is False
    assert square.contains(2.0, 1.0) is True  # on an edge
    assert square.contains(0.0, 0.0) is True  # on a vertex


def test_polygon_contains_concave():
    # The removed north-east quadrant is outside even though it is well inside
    # the bounding box and inside a circle drawn round the shape.
    assert L_SHAPE.contains(1.0, 1.0) is True
    assert L_SHAPE.contains(3.0, 1.0) is True
    assert L_SHAPE.contains(1.0, 3.0) is True
    assert L_SHAPE.contains(3.0, 3.0) is False
    assert L_SHAPE.contains(2.5, 2.5) is False


def test_polygon_winding_order_does_not_matter():
    reversed_l = Polygon(tuple(reversed(L_SHAPE.vertices)))
    for px, py in ((1.0, 1.0), (3.0, 1.0), (3.0, 3.0), (2.5, 2.5)):
        assert reversed_l.contains(px, py) == L_SHAPE.contains(px, py)


def test_representative_point_is_inside_a_u_shape():
    cx, cy = U_SHAPE.centroid()
    assert U_SHAPE.contains(cx, cy) is False  # the centroid sits in the notch
    rx, ry = U_SHAPE.representative_point()
    assert U_SHAPE.contains(rx, ry) is True


def test_polygon_zone_loads_and_contains(tmp_path):
    path = tmp_path / "zones.yaml"
    path.write_text(
        "zones:\n"
        "  ward: {x: 3.0, y: 0.5, yaw: 1.5,\n"
        "    polygon: [[0, 0], [4, 0], [4, 2], [2, 2], [2, 4], [0, 4]]}\n"
    )
    zones = load_zones(str(path))
    ward = zones["ward"]
    assert ward.pose.pose.position.x == pytest.approx(3.0)  # the taught pose wins
    assert ward.footprint.contains(3.0, 1.0) is True
    assert ward.footprint.contains(3.0, 3.0) is False
    assert containing(zones, 3.0, 3.0) == []
    assert containing(zones, 3.0, 1.0) == ["ward"]


def test_polygon_only_zone_derives_a_pose_inside_the_outline(tmp_path):
    path = tmp_path / "zones.yaml"
    path.write_text(
        "zones:\n"
        "  hall: {polygon: [[0, 0], [6, 0], [6, 6], [4, 6], [4, 2], [2, 2], [2, 6], [0, 6]]}\n"
    )
    zones = load_zones(str(path))
    p = zones["hall"].pose.pose.position
    assert zones["hall"].footprint.contains(p.x, p.y) is True


def test_polygon_beats_radius_when_a_zone_carries_both(tmp_path):
    path = tmp_path / "zones.yaml"
    path.write_text(
        "zones:\n"
        "  ward: {x: 1.0, y: 1.0, radius: 10.0,\n"
        "    polygon: [[0, 0], [2, 0], [2, 2], [0, 2]]}\n"
    )
    footprint = load_zones(str(path))["ward"].footprint
    assert isinstance(footprint, Polygon)
    assert footprint.contains(5.0, 5.0) is False  # the radius would have matched


@pytest.mark.parametrize(
    "polygon",
    ["[[0, 0], [1, 1]]", "[[0, 0], [1, 1], [2, 2, 2]]", "5"],
)
def test_load_zones_bad_polygon_raises(tmp_path, polygon):
    path = tmp_path / "zones.yaml"
    path.write_text(f"zones:\n  bad: {{x: 0.0, y: 0.0, polygon: {polygon}}}\n")
    with pytest.raises(ValueError):
        load_zones(str(path))


def test_append_zone_keeps_a_polygon_but_radius_replaces_it(tmp_path):
    path = tmp_path / "zones.yaml"
    path.write_text(
        "zones:\n  ward: {x: 1.0, y: 1.0, polygon: [[0, 0], [2, 0], [2, 2], [0, 2]]}\n"
    )
    # Re-teaching the pose must not silently un-room the zone.
    append_zone(path, "ward", 1.5, 0.5, 0.0)
    ward = load_zones(str(path))["ward"]
    assert isinstance(ward.footprint, Polygon)
    assert ward.pose.pose.position.x == pytest.approx(1.5)

    # ...but an explicit --radius is a deliberate new footprint.
    append_zone(path, "ward", 1.5, 0.5, 0.0, radius=3.0)
    assert load_zones(str(path))["ward"].footprint.radius == pytest.approx(3.0)
