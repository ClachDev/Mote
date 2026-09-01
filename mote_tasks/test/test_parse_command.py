"""``fetch``'s input: two properties, one of which is deliberately untyped.

``destination`` names a place and ``$ref``s the zone reference, so the schema
checks its shape. ``target`` is a plain string in the standard registry,
because it is either a zone name or an open-vocabulary object label and no
schema can tell which — so the branch between them is here, and so is the case
that makes it worth testing.
"""

import pytest

from mote_tasks.trees.fetch import prepare
from mote_tasks.zones import Zone, ZoneUnresolved

# A real Zone, because resolution reads the vocabulary as well as the pose.
# Zone does not care what `pose` is, so a sentinel still saves building a
# PoseStamped.
ZONES = {
    "pickup": Zone("pickup", "PICKUP_POSE"),
    "the bin": Zone("the bin", "DROPOFF_POSE"),
    "server_room": Zone("server_room", "SERVER_POSE", navigable=False),
}


def a_fetch(target, destination="the bin"):
    return prepare(ZONES, {"target": target, "destination": destination})


def test_zone_target_yields_pose_and_no_label():
    object_pose, object_label, drop_pose, drop = a_fetch("pickup")
    assert object_pose == "PICKUP_POSE"
    assert object_label is None
    assert drop_pose == "DROPOFF_POSE"
    assert drop.name == "the bin"


def test_label_target_yields_label_and_no_pose():
    object_pose, object_label, drop_pose, _ = a_fetch("red_box")
    assert object_pose is None
    assert object_label == "red box"
    assert drop_pose == "DROPOFF_POSE"


def test_unknown_destination_is_unresolved_not_a_label():
    with pytest.raises(ZoneUnresolved) as excinfo:
        a_fetch("pickup", "nowhere")
    assert excinfo.value.reason == "unknown_name"


def test_a_destination_name_with_a_space_in_it_resolves():
    # A place-name is what a person writes, so `the bin` is a name and not two
    # words needing an alias to join them up.
    assert a_fetch("red_box", "The  Bin")[2] == "DROPOFF_POSE"


def test_constraint_zone_is_refused_rather_than_read_as_a_label():
    # The dangerous shape: falling through to the label branch would send the
    # detector hunting for an object called "server room" and look like success.
    with pytest.raises(ZoneUnresolved, match="not a destination") as excinfo:
        a_fetch("server_room")
    assert excinfo.value.reason == "not_navigable"
    with pytest.raises(ZoneUnresolved):
        a_fetch("red_box", "server_room")
