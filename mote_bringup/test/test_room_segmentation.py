"""Room segmentation on synthetic occupancy grids (no map files, no ROS).

Every fixture is drawn here rather than loaded, so each test states the one
piece of geometry it is about: a doorway separates, a wide opening does not,
furniture is not a wall, a rotated building is the same building. Scoring
against the sim ladder's real SLAM maps is a separate harness --
``mote_simulation/test/room_segmentation_eval.py``.
"""

import cv2
import numpy as np
import pytest

from mote_bringup.map_cleanup.room_segmentation import (
    MapGeometry,
    RoomParams,
    dominant_rotation_deg,
    polygon_area,
    polygon_contains,
    segment_rooms,
)
from mote_bringup.map_cleanup.structure_extraction import FREE, OCCUPIED, UNKNOWN

RES = 0.05


def blank(width_m, height_m):
    """An all-unknown grid ``width_m`` x ``height_m`` plus its geometry."""
    shape = (int(round(height_m / RES)), int(round(width_m / RES)))
    return np.full(shape, UNKNOWN, np.uint8), MapGeometry(RES, (0.0, 0.0), shape[0])


def box(occ, geometry, x0, y0, x1, y1, value):
    (c0, r1), (c1, r0) = geometry.to_pixel(x0, y0), geometry.to_pixel(x1, y1)
    occ[int(round(r0)) : int(round(r1)), int(round(c0)) : int(round(c1))] = value


def room(occ, geometry, x0, y0, x1, y1, thickness=0.15):
    """A closed rectangular room: free inside, walls on the boundary."""
    t = thickness
    box(occ, geometry, x0 - t, y0 - t, x1 + t, y1 + t, OCCUPIED)
    box(occ, geometry, x0, y0, x1, y1, FREE)


def gap(occ, geometry, x0, y0, x1, y1):
    box(occ, geometry, x0, y0, x1, y1, FREE)


def names(result):
    return sorted(r.name for r in result.rooms)


def test_a_doorway_separates_two_rooms():
    occ, geometry = blank(12, 7)
    room(occ, geometry, 0.5, 0.5, 5.5, 6.5)
    room(occ, geometry, 5.65, 0.5, 11.5, 6.5)
    gap(occ, geometry, 5.5, 3.0, 5.65, 3.9)  # 0.9 m door in the shared wall

    result = segment_rooms(occ, geometry)

    assert names(result) == ["room_01", "room_02"]
    assert all(polygon_contains(r.polygon, *r.pose) for r in result.rooms)


def test_a_wide_opening_does_not():
    occ, geometry = blank(12, 7)
    room(occ, geometry, 0.5, 0.5, 5.5, 6.5)
    room(occ, geometry, 5.65, 0.5, 11.5, 6.5)
    gap(occ, geometry, 5.5, 2.0, 5.65, 5.0)  # 3 m archway, not a door

    result = segment_rooms(occ, geometry)

    assert len(result.rooms) == 1


def test_the_threshold_is_the_widest_gap_not_the_total():
    """Two doors in one wall are two doors, not one double-width opening."""
    occ, geometry = blank(12, 7)
    room(occ, geometry, 0.5, 0.5, 5.5, 6.5)
    room(occ, geometry, 5.65, 0.5, 11.5, 6.5)
    gap(occ, geometry, 5.5, 1.5, 5.65, 2.4)
    gap(occ, geometry, 5.5, 4.5, 5.65, 5.4)

    assert len(segment_rooms(occ, geometry).rooms) == 2


def test_furniture_does_not_cut_a_room_up():
    occ, geometry = blank(8, 8)
    room(occ, geometry, 0.5, 0.5, 7.5, 7.5)
    box(
        occ, geometry, 2.0, 3.5, 3.9, 4.4, OCCUPIED
    )  # a bed, wall-length but not a wall
    box(occ, geometry, 5.0, 3.5, 5.4, 3.9, OCCUPIED)  # a column

    result = segment_rooms(occ, geometry)

    assert len(result.rooms) == 1
    assert result.rooms[0].area_m2 > 40


def test_a_corridor_ring_is_not_proposed_because_it_would_claim_what_it_encircles():
    """A footprint is one outline, so a region with a room-sized hole is dropped."""
    occ, geometry = blank(16, 12)
    box(occ, geometry, 0.3, 0.3, 15.7, 11.7, FREE)  # open floor: the corridor ring
    room(occ, geometry, 4.0, 4.0, 12.0, 8.0)  # a block of rooms sitting in it
    gap(occ, geometry, 3.85, 5.5, 4.0, 6.4)

    result = segment_rooms(occ, geometry)

    assert result.encircling == 1
    assert [r.name for r in result.rooms] == ["room_01"]
    assert not polygon_contains(result.rooms[0].polygon, 1.0, 1.0)  # not the ring


def test_a_rotated_building_is_the_same_building():
    occ, geometry = blank(14, 10)
    room(occ, geometry, 1.0, 1.0, 6.0, 9.0)
    room(occ, geometry, 6.15, 1.0, 13.0, 9.0)
    gap(occ, geometry, 6.0, 4.5, 6.15, 5.4)

    upright = segment_rooms(occ, geometry)
    matrix = cv2.getRotationMatrix2D(
        (occ.shape[1] / 2.0, occ.shape[0] / 2.0), 20.0, 1.0
    )
    turned = cv2.warpAffine(
        occ,
        matrix,
        (occ.shape[1], occ.shape[0]),
        flags=cv2.INTER_NEAREST,
        borderValue=int(UNKNOWN),
    )
    result = segment_rooms(turned, geometry)

    assert dominant_rotation_deg(turned) == pytest.approx(-20.0, abs=1.5)
    assert result.rotation_deg == pytest.approx(-20.0, abs=1.5)
    assert len(result.rooms) == len(upright.rooms) == 2
    for got, want in zip(result.rooms, upright.rooms):
        assert polygon_area(got.polygon) == pytest.approx(
            polygon_area(want.polygon), rel=0.15
        )


def test_a_half_mapped_wall_still_separates():
    """The gap in it was never observed, so nothing is known to pass through."""
    occ, geometry = blank(12, 8)
    room(occ, geometry, 0.5, 0.5, 5.5, 7.5)
    room(occ, geometry, 5.65, 0.5, 11.5, 7.5)
    box(occ, geometry, 5.5, 0.5, 5.65, 4.0, UNKNOWN)  # lower half of it unseen

    assert len(segment_rooms(occ, geometry).rooms) == 2


def test_the_pose_is_the_open_middle_and_the_area_is_observed_free_space():
    occ, geometry = blank(9, 9)
    room(occ, geometry, 0.5, 0.5, 8.5, 8.5)

    (only,) = segment_rooms(occ, geometry).rooms

    assert only.pose == pytest.approx((4.5, 4.5), abs=0.3)
    assert only.clearance_m == pytest.approx(4.0, abs=0.2)
    assert only.area_m2 == pytest.approx(64.0, abs=1.0)
    assert polygon_area(only.polygon) == pytest.approx(64.0, rel=0.05)


def test_small_leftovers_are_dropped():
    occ, geometry = blank(10, 8)
    room(occ, geometry, 0.5, 0.5, 9.5, 6.0)
    room(occ, geometry, 0.5, 6.15, 1.4, 7.5)  # a 0.9 x 1.35 m cupboard
    gap(occ, geometry, 0.7, 6.0, 1.2, 6.15)

    result = segment_rooms(occ, geometry, RoomParams(min_room_area_m2=2.0))

    assert len(result.rooms) == 1


def test_geometry_round_trips_pixels_and_metres():
    geometry = MapGeometry(0.05, (-29.023, -19.126), 760)

    assert geometry.to_world(*geometry.to_pixel(3.5, -7.25)) == pytest.approx(
        (3.5, -7.25)
    )
    assert geometry.to_world(0, 760) == pytest.approx((-29.023, -19.126))
