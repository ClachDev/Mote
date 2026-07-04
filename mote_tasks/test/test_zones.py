import math

import pytest

from mote_tasks.zones import append_zone, load_zones, yaw_from_quaternion


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
    assert zones["bin"].pose.position.x == pytest.approx(1.5)
    assert zones["bin"].header.frame_id == "map"
    yaw = yaw_from_quaternion(
        0.0, 0.0, zones["sofa"].pose.orientation.z, zones["sofa"].pose.orientation.w
    )
    assert yaw == pytest.approx(-1.0)
