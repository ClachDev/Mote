"""Shared behaviour-tree pieces used by every mission.

The task server writes a human-readable task string to the ``task`` blackboard
key when a command is accepted and clears it when the mission ends, so every
mission tree opens by idling in :class:`WaitForTask` until that key is set.
"""

import py_trees

TASK_KEY = "task"


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
