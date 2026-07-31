import pytest

from mote_tasks.trees.goto import parse_command
from mote_tasks.zones import Zone

# A real Zone, because parse_command reads the vocabulary as well as the pose.
# Zone does not care what `pose` is, so a sentinel still saves building a
# PoseStamped, and the vocabulary defaults come out right by construction.
ZONES = {
    "kitchen": Zone(
        "kitchen",
        "KITCHEN_POSE",
        kind="room",
        display_name="The Kitchen",
        aliases=("galley",),
    ),
    "pickup": Zone("pickup", "PICKUP_POSE", kind="pickup"),
    "server_room": Zone("server_room", "SERVER_POSE", kind="keepout", navigable=False),
}


def test_known_zone_yields_its_pose():
    assert parse_command(ZONES, ["goto", "kitchen"]) == "KITCHEN_POSE"


def test_goto_works_for_any_zone_not_just_rooms():
    # goto resolves against the same table fetch uses, so a plain waypoint works.
    assert parse_command(ZONES, ["goto", "pickup"]) == "PICKUP_POSE"


def test_alias_and_display_name_reach_the_zone():
    # What a dispatcher gets from /v1/zones is what it may say here.
    assert parse_command(ZONES, ["goto", "galley"]) == "KITCHEN_POSE"
    assert parse_command(ZONES, ["goto", "The Kitchen"]) == "KITCHEN_POSE"


def test_constraint_zone_is_refused_as_a_destination():
    # A keepout is in the vocabulary but is not a place to drive to, and the
    # refusal says which kind it is rather than pretending the name is unknown.
    with pytest.raises(ValueError, match="keepout"):
        parse_command(ZONES, ["goto", "server_room"])


def test_unknown_zone_lists_only_navigable_names():
    # The listed names are what a dispatcher may actually send.
    with pytest.raises(ValueError) as excinfo:
        parse_command(ZONES, ["goto", "nowhere"])
    assert "kitchen" in str(excinfo.value)
    assert "server_room" not in str(excinfo.value)


def test_malformed_command_raises():
    with pytest.raises(ValueError):
        parse_command(ZONES, ["goto"])
    with pytest.raises(ValueError):
        parse_command(ZONES, ["goto", "kitchen", "extra"])
    with pytest.raises(ValueError):
        parse_command(ZONES, ["drive", "kitchen"])
