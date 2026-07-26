import math

import pytest

from mote_tasks.zones import (
    append_zone,
    containing,
    load_zones,
    yaw_from_quaternion,
)


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
