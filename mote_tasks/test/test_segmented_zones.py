"""The seam between map room segmentation and the task layer's zones.

``mote_bringup`` writes candidate rooms into a zones file and ``mote_tasks``
reads them back; the schema they agree on is documented in one place
(:mod:`mote_tasks.zones`) and implemented in two. These tests run the real
writer into the real loader, so a change to either that breaks the other fails
here rather than the first time an operator says "go to the kitchen".
"""

import numpy as np
import pytest
import yaml

from mote_bringup.map_cleanup.room_segmentation import MapGeometry, segment_rooms
from mote_bringup.map_cleanup.rooms_cli import merge_into_zones
from mote_bringup.map_cleanup.structure_extraction import FREE, OCCUPIED, UNKNOWN
from mote_tasks import zones as zones_lib

RES = 0.05


def two_rooms_with_a_door():
    """A 12 x 7 m building: two rooms sharing a wall with a 0.9 m doorway."""
    geometry = MapGeometry(RES, (0.0, 0.0), int(round(7 / RES)))
    occ = np.full((int(round(7 / RES)), int(round(12 / RES))), UNKNOWN, np.uint8)

    def box(x0, y0, x1, y1, value):
        (c0, r1), (c1, r0) = geometry.to_pixel(x0, y0), geometry.to_pixel(x1, y1)
        occ[int(round(r0)) : int(round(r1)), int(round(c0)) : int(round(c1))] = value

    for x0, x1 in ((0.5, 5.5), (5.65, 11.5)):
        box(x0 - 0.15, 0.35, x1 + 0.15, 6.65, OCCUPIED)
        box(x0, 0.5, x1, 6.5, FREE)
    box(5.5, 3.0, 5.65, 3.9, FREE)
    return occ, geometry


@pytest.fixture
def rooms():
    occ, geometry = two_rooms_with_a_door()
    result = segment_rooms(occ, geometry)
    assert len(result.rooms) == 2
    return result.rooms


def test_a_written_room_loads_back_as_a_zone_the_robot_can_be_in(tmp_path, rooms):
    path = tmp_path / "zones.yaml"
    added, skipped = merge_into_zones(path, rooms)
    assert (added, skipped) == (["room_01", "room_02"], [])

    loaded = zones_lib.load_zones(str(path))

    assert set(loaded) == {"room_01", "room_02"}
    for name, zone in loaded.items():
        assert isinstance(zone.footprint, zones_lib.Polygon)
        pose = zone.pose.pose.position
        assert zones_lib.containing(loaded, pose.x, pose.y)[0] == name

    left = zones_lib.containing(loaded, 3.0, 3.0)
    right = zones_lib.containing(loaded, 9.0, 3.0)
    assert left and right and left != right


def test_a_hand_taught_room_is_not_renamed_by_a_later_run(tmp_path, rooms):
    path = tmp_path / "zones.yaml"
    path.write_text(
        yaml.safe_dump(
            {
                "frame_id": "map",
                "zones": {
                    "kitchen": {"x": 3.0, "y": 3.0, "yaw": 0.0, "radius": 1.0},
                    "pickup": {"x": 9.0, "y": 3.0, "yaw": 0.0},
                },
            }
        )
    )

    added, skipped = merge_into_zones(path, rooms)

    assert len(skipped) == 1  # the room the kitchen already names
    assert len(added) == 1  # the room only a bare waypoint stands in
    loaded = zones_lib.load_zones(str(path))
    assert isinstance(loaded["kitchen"].footprint, zones_lib.Circle)
    assert set(loaded) == {"kitchen", "pickup", *added}


def test_running_it_twice_adds_nothing(tmp_path, rooms):
    path = tmp_path / "zones.yaml"
    merge_into_zones(path, rooms)
    before = path.read_text()

    added, skipped = merge_into_zones(path, rooms)

    assert added == []
    assert len(skipped) == 2
    assert path.read_text() == before
