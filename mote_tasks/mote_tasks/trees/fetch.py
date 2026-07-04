"""The fetch tree: wait for a task, drive to the object, pick it up,
drive to the drop zone, place it.

The task server owns the blackboard keys: ``task`` (a human-readable task
string, None when idle) plus the two poses. The tree idles in WaitForTask
until ``task`` is set, runs the mission, and the server clears ``task``
when the root reports SUCCESS or FAILURE.
"""

import py_trees

from mote_tasks.behaviours.manipulation import TimedStub
from mote_tasks.behaviours.nav import DriveTo

TASK_KEY = "task"
OBJECT_POSE_KEY = "object_pose"
DROP_POSE_KEY = "drop_pose"


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
            DriveTo("drive_to_object", OBJECT_POSE_KEY),
            TimedStub("pick", pick_duration),
            DriveTo("drive_to_drop", DROP_POSE_KEY),
            TimedStub("place", place_duration),
        ],
    )
    return py_trees.trees.BehaviourTree(root)
