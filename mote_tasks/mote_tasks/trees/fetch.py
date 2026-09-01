"""The fetch tree: wait for a task, acquire the object's pose, drive to it,
pick it up, drive to the drop zone, place it.

The task server owns the blackboard keys: ``task`` (a human-readable task
string, None when idle), the two poses, and ``object_label``. A zone target
arrives with ``object_pose`` already set and AcquireObject passes through; a
label target arrives with ``object_label`` set instead and AcquireObject asks
the detector for a matching pose. The tree idles in WaitForTask until ``task``
is set, runs the mission, and the server clears ``task`` when the root reports
SUCCESS or FAILURE.
"""

import py_trees

from mote_tasks import zones as mote_zones
from mote_tasks.behaviours.manipulation import TimedStub
from mote_tasks.behaviours.nav import DriveTo
from mote_tasks.behaviours.perception import AcquireObject
from mote_tasks.trees.common import WaitForTask

OBJECT_POSE_KEY = "object_pose"
OBJECT_LABEL_KEY = "object_label"
DROP_POSE_KEY = "drop_pose"


def prepare(zones: dict, payload_input: dict):
    """``(object_pose, object_label, drop_pose, drop_zone)`` for a ``fetch``.

    ``destination`` names a zone. ``target`` is the registry's deliberately
    untyped half: a zone name when it matches one, otherwise an
    open-vocabulary object label for the detector (underscores become spaces,
    so ``red_box`` looks for "red box"). A target naming a *non-navigable* zone
    is neither — falling through to the label branch would send the detector
    hunting for an object called "keepout" — so it is refused with the zone's
    own reason.
    """
    drop = mote_zones.destination(
        zones, payload_input["destination"], where="destination"
    )
    target = payload_input["target"]
    zone, reason = mote_zones.resolve_reason(zones, target)
    if reason == "not_navigable":
        raise mote_zones.ZoneUnresolved(
            reason, f"target {zone.name!r} is a {zone.kind} zone, not a destination"
        )
    if zone is not None:
        return zone.pose, None, drop.pose, drop
    return None, target.replace("_", " "), drop.pose, drop


def create_fetch_tree(
    pick_duration: float = 3.0, place_duration: float = 3.0
) -> py_trees.trees.BehaviourTree:
    root = py_trees.composites.Sequence(
        name="fetch",
        memory=True,
        children=[
            WaitForTask(),
            AcquireObject("acquire_object", OBJECT_POSE_KEY, OBJECT_LABEL_KEY),
            DriveTo("drive_to_object", OBJECT_POSE_KEY),
            TimedStub("pick", pick_duration),
            DriveTo("drive_to_drop", DROP_POSE_KEY),
            TimedStub("place", place_duration),
        ],
    )
    return py_trees.trees.BehaviourTree(root)
