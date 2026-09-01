"""Shared behaviour-tree pieces used by every mission.

The task server writes a human-readable task string to the ``task`` blackboard
key when a mission is accepted and clears it when the mission ends, so every
mission tree opens by idling in :class:`WaitForTask` until that key is set.

The other shared key is how a failure gets a *type*. py_trees gives the server
one bit — the root reported FAILURE — and the name of the tip it stopped at,
which is a sentence for a human and nothing for a dispatcher. mission/v0 wants
a class and a recoverability, and only the behaviour that failed knows them: a
Nav2 goal that aborted is ``obstructed`` and worth retrying, one Nav2 refused
outright is ``unreachable`` and is not, and a detector that never saw the
object is ``timeout``. So a behaviour states it on the way out through
:func:`report_failure`, and the server reads and clears it.

A tree that fails without one is reported as ``internal``, which is the honest
reading: a behaviour that failed and said nothing about why is a gap in this
software, not a fact about the building.
"""

import py_trees

TASK_KEY = "task"

#: Where a failing behaviour leaves its ``(class, detail, recoverable)`` for
#: the task server to turn into a mission/v0 failure.
FAILURE_KEY = "mission_failure"


def report_failure(blackboard, failure_class: str, detail: str, recoverable=None):
    """Record why this behaviour is about to return FAILURE.

    ``recoverable`` is passed through to ``mission.failure`` untouched, so the
    spec's rule that the "depends" classes must state it lands on the behaviour
    that knows — not on the server, which does not.
    """
    blackboard.set(
        FAILURE_KEY,
        {"class": failure_class, "detail": detail, "recoverable": recoverable},
    )


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
