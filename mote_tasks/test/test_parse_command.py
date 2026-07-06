import pytest

from mote_tasks.trees.fetch import parse_command

ZONES = {"pickup": "PICKUP_POSE", "dropoff": "DROPOFF_POSE"}


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
