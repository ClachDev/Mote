"""The goto tree: wait for a mission, drive to a named zone's pose.

``goto`` takes one input, ``target``, which names a place and never a
coordinate (see mote_tasks.zones, and zone/v0's invariant: names are portable
between robots and coordinates are not). The mission is a single DriveTo to the
zone's pose, so success is exactly Nav2 reaching it. The task server writes the
resolved pose to the ``goto_pose`` blackboard key and a human-readable summary
to ``task``; the tree idles in WaitForTask until ``task`` is set and the server
clears it when the root reports SUCCESS or FAILURE.
"""

import py_trees

from mote_tasks import zones as mote_zones
from mote_tasks.behaviours.nav import DriveTo
from mote_tasks.trees.common import WaitForTask

GOTO_POSE_KEY = "goto_pose"


def prepare(zones: dict, payload_input: dict):
    """``(pose, zone)`` for a validated ``goto`` input.

    The input has already been checked against the capability's own
    ``input_schema``, so ``target`` is a well-formed zone name; what is left is
    whether *this robot* knows the place, which is the question the schema
    cannot ask. A target may be a zone's name, a display name or an alias —
    the resolution is zone/v0's, case-insensitive and whitespace-normalised.
    """
    zone = mote_zones.destination(zones, payload_input["target"], where="target")
    return zone.pose, zone


def create_goto_tree() -> py_trees.trees.BehaviourTree:
    root = py_trees.composites.Sequence(
        name="goto",
        memory=True,
        children=[
            WaitForTask(),
            DriveTo("drive_to_zone", GOTO_POSE_KEY),
        ],
    )
    return py_trees.trees.BehaviourTree(root)
