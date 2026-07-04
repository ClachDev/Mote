"""Arm behaviours. Until the SO-101 is actuated these are timed stand-ins
that hold the tree in place where a real pick/place will slot in."""

import py_trees
from rclpy.duration import Duration


class TimedStub(py_trees.behaviour.Behaviour):
    """Log once, stay RUNNING for ``duration`` seconds, then succeed."""

    def __init__(self, name: str, duration: float = 3.0):
        super().__init__(name)
        self.duration = duration

    def setup(self, **kwargs):
        self.node = kwargs["node"]

    def initialise(self):
        self.deadline = self.node.get_clock().now() + Duration(seconds=self.duration)
        self.node.get_logger().info(
            f"{self.name} (stub): holding for {self.duration:.1f} s"
        )

    def update(self):
        if self.node.get_clock().now() < self.deadline:
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS
