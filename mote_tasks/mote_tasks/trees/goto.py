"""The goto tree: wait for a task, drive to a named zone's pose.

``goto <zone>`` is place-based language ("go to the kitchen"): the target is a
named place (see mote_tasks.zones) — the same table fetch uses. The mission is
a single DriveTo to the zone's pose, so success is exactly Nav2 reaching it.
The task server writes the resolved pose to the ``goto_pose`` blackboard key
and the human-readable task string to ``task``; the tree idles in WaitForTask
until ``task`` is set and the server clears it when the root reports SUCCESS or
FAILURE.
"""

import py_trees

from mote_tasks.behaviours.nav import DriveTo
from mote_tasks.trees.common import WaitForTask

GOTO_POSE_KEY = "goto_pose"

COMMAND = "goto"


def parse_command(zones: dict, words: list):
    """Parse a ``goto <zone>`` command against known zones.

    Returns the zone's PoseStamped; raises ValueError with a user-facing
    message when the command is malformed or the zone is unknown.
    """
    if len(words) != 2 or words[0] != COMMAND:
        raise ValueError(f"expected: {COMMAND} <zone>")
    name = words[1]
    if name not in zones:
        raise ValueError(f"unknown zone '{name}', have {sorted(zones)}")
    return zones[name].pose


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
