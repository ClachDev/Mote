from types import SimpleNamespace

import pytest

from mote_tasks.trees.goto import parse_command

# parse_command reads the matched zone's `.pose`.
ZONES = {
    "kitchen": SimpleNamespace(pose="KITCHEN_POSE"),
    "pickup": SimpleNamespace(pose="PICKUP_POSE"),
}


def test_known_zone_yields_its_pose():
    assert parse_command(ZONES, ["goto", "kitchen"]) == "KITCHEN_POSE"


def test_goto_works_for_any_zone_not_just_rooms():
    # goto resolves against the same table fetch uses, so a plain waypoint works.
    assert parse_command(ZONES, ["goto", "pickup"]) == "PICKUP_POSE"


def test_unknown_zone_raises():
    with pytest.raises(ValueError):
        parse_command(ZONES, ["goto", "nowhere"])


def test_malformed_command_raises():
    with pytest.raises(ValueError):
        parse_command(ZONES, ["goto"])
    with pytest.raises(ValueError):
        parse_command(ZONES, ["goto", "kitchen", "extra"])
    with pytest.raises(ValueError):
        parse_command(ZONES, ["drive", "kitchen"])
