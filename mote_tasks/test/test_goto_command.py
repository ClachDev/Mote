"""``goto``'s input, from a well-formed object to a pose or a typed refusal.

Everything about the *shape* of the input — is ``target`` present, is it a
lowercase name — is the capability's ``input_schema``, checked before this
function sees it and covered in ``test_spec_capability.py``. What is left here
is the question a schema cannot ask: does *this robot* know the place.
"""

import pytest

from mote_tasks.trees.goto import prepare
from mote_tasks.zones import Zone, ZoneUnresolved

# A real Zone, because resolution reads the vocabulary as well as the pose.
# Zone does not care what `pose` is, so a sentinel still saves building a
# PoseStamped, and the vocabulary defaults come out right by construction.
ZONES = {
    "the kitchen": Zone("the kitchen", "KITCHEN_POSE", note="the good kettle"),
    "pickup": Zone("pickup", "PICKUP_POSE"),
    "server_room": Zone("server_room", "SERVER_POSE", navigable=False),
}


def test_known_zone_yields_its_pose():
    pose, zone = prepare(ZONES, {"target": "the kitchen"})
    assert pose == "KITCHEN_POSE"
    assert zone.name == "the kitchen"


def test_goto_works_for_any_zone_not_just_rooms():
    # goto resolves against the same table fetch uses, so a plain waypoint works.
    assert prepare(ZONES, {"target": "pickup"})[0] == "PICKUP_POSE"


def test_the_name_is_reached_however_it_is_capitalised_or_spaced():
    # A place-name is what an operator writes, so it has to survive being
    # written by hand. What a dispatcher gets from /v1/zones is what it may say.
    assert prepare(ZONES, {"target": "The Kitchen"})[0] == "KITCHEN_POSE"
    assert prepare(ZONES, {"target": "the  KITCHEN"})[0] == "KITCHEN_POSE"


def test_constraint_zone_is_refused_with_its_own_reason():
    # A keepout is in the vocabulary but is not a place to drive to. The reason
    # is `not_navigable` rather than `unknown_name`, because an operator does a
    # different thing about each: one is a typo, the other is a zone doing
    # exactly what it was drawn to do.
    with pytest.raises(ZoneUnresolved, match="not a destination") as excinfo:
        prepare(ZONES, {"target": "server_room"})
    assert excinfo.value.reason == "not_navigable"


def test_unknown_zone_lists_only_navigable_names():
    # The listed names are what a dispatcher may actually send.
    with pytest.raises(ZoneUnresolved) as excinfo:
        prepare(ZONES, {"target": "nowhere"})
    assert excinfo.value.reason == "unknown_name"
    assert "kitchen" in str(excinfo.value)
    assert "server_room" not in str(excinfo.value)
