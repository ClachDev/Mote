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

from mote_tasks.behaviours.manipulation import TimedStub
from mote_tasks.behaviours.nav import DriveTo
from mote_tasks.behaviours.perception import AcquireObject

TASK_KEY = "task"
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
    if drop not in zones:
        raise ValueError(f"unknown drop zone '{drop}', have {sorted(zones)}")
    if target in zones:
        return zones[target], None, zones[drop]
    return None, target.replace("_", " "), zones[drop]


class WaitForTask(py_trees.behaviour.Behaviour):
    """Idle (RUNNING) until the task server writes a task to the blackboard."""

    def __init__(self, name: str = "wait_for_task"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(TASK_KEY, access=py_trees.common.Access.READ)

    def update(self):
        if self.blackboard.exists(TASK_KEY) and self.blackboard.get(TASK_KEY):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


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
