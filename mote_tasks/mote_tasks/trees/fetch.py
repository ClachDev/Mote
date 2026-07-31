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

COMMAND = "fetch"


def parse_command(zones: dict, words: list):
    """Parse ``fetch <target> <drop_zone>`` against known zones.

    Returns ``(object_pose, object_label, drop_pose)``. A ``target`` that names
    a zone yields that pose and no label; any other target is an open-vocabulary
    object label (underscores become spaces) with no pose, left for the detector
    to resolve. Raises ValueError with a user-facing message when the command is
    malformed or the drop zone is unknown.
    """
    if len(words) != 3 or words[0] != COMMAND:
        raise ValueError(f"expected: {COMMAND} <target> <drop_zone>")
    target, drop = words[1], words[2]
    drop_zone = mote_zones.resolve(zones, drop)
    if drop_zone is None or not drop_zone.navigable:
        known = sorted(name for name, z in zones.items() if z.navigable)
        raise ValueError(f"unknown drop zone '{drop}', have {known}")
    # A target naming a zone is a place to drive to; anything else is a label
    # for the detector. A *non-navigable* zone name is neither — falling
    # through would send the detector hunting for an object called "keepout".
    target_zone = mote_zones.resolve(zones, target)
    if target_zone is not None:
        if not target_zone.navigable:
            raise ValueError(
                f"zone '{target_zone.name}' is a {target_zone.kind} zone, "
                "not a destination"
            )
        return target_zone.pose, None, drop_zone.pose
    return None, target.replace("_", " "), drop_zone.pose


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
