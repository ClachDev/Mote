import pytest

from mote_tasks.trees.fetch import parse_command
from mote_tasks.zones import Zone

# A real Zone, because parse_command reads the vocabulary as well as the pose.
# Zone does not care what `pose` is, so a sentinel still saves building a
# PoseStamped.
ZONES = {
    "pickup": Zone("pickup", "PICKUP_POSE", kind="pickup"),
    "dropoff": Zone("dropoff", "DROPOFF_POSE", kind="dropoff", aliases=("the bin",)),
    "server_room": Zone("server_room", "SERVER_POSE", kind="keepout", navigable=False),
}


def test_zone_target_yields_pose_and_no_label():
    object_pose, object_label, drop_pose = parse_command(
        ZONES, ["fetch", "pickup", "dropoff"]
    )
    assert object_pose == "PICKUP_POSE"
    assert object_label is None
    assert drop_pose == "DROPOFF_POSE"


def test_label_target_yields_label_and_no_pose():
    object_pose, object_label, drop_pose = parse_command(
        ZONES, ["fetch", "red_box", "dropoff"]
    )
    assert object_pose is None
    assert object_label == "red box"
    assert drop_pose == "DROPOFF_POSE"


def test_malformed_command_raises():
    with pytest.raises(ValueError):
        parse_command(ZONES, ["fetch", "pickup"])
    with pytest.raises(ValueError):
        parse_command(ZONES, ["grab", "pickup", "dropoff"])


def test_unknown_drop_zone_raises():
    with pytest.raises(ValueError):
        parse_command(ZONES, ["fetch", "pickup", "nowhere"])


def test_drop_zone_alias_resolves():
    _, _, drop_pose = parse_command(ZONES, ["fetch", "red_box", "the bin"])
    assert drop_pose == "DROPOFF_POSE"


def test_constraint_zone_is_refused_rather_than_read_as_a_label():
    # The dangerous shape: falling through to the label branch would send the
    # detector hunting for an object called "server room" and look like success.
    with pytest.raises(ValueError, match="keepout"):
        parse_command(ZONES, ["fetch", "server_room", "dropoff"])
    with pytest.raises(ValueError):
        parse_command(ZONES, ["fetch", "red_box", "server_room"])
