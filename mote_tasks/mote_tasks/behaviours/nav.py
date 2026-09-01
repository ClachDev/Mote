"""Behaviours that drive the base through Nav2."""

import py_trees
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration

from mote_tasks.trees.common import FAILURE_KEY, report_failure


class DriveTo(py_trees.behaviour.Behaviour):
    """Send the blackboard pose under ``pose_key`` to Nav2 and await the result.

    RUNNING while the goal is in flight, SUCCESS when Nav2 reports the goal
    succeeded, FAILURE on rejection, abortion, or if the action server never
    appears within ``server_timeout`` seconds. A goal still in flight is
    cancelled when the behaviour is preempted.

    Each of those three failures leaves a *typed* reason for the task server
    (``trees.common.report_failure``), because they are three different things
    to a dispatcher. Nav2 refusing the goal means no route exists to it and
    resending will be refused identically; Nav2 aborting means it tried,
    exhausted its recoveries and stopped, which is what a blocked corridor
    looks like and is worth retrying; no action server at all is this
    software not running, and is neither.
    """

    def __init__(self, name: str, pose_key: str, server_timeout: float = 10.0):
        super().__init__(name)
        self.pose_key = pose_key
        self.server_timeout = server_timeout
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(pose_key, access=py_trees.common.Access.READ)
        self.blackboard.register_key(FAILURE_KEY, access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.client = ActionClient(self.node, NavigateToPose, "navigate_to_pose")

    def initialise(self):
        self.send_future = None
        self.goal_handle = None
        self.result_future = None
        self.server_deadline = self.node.get_clock().now() + Duration(
            seconds=self.server_timeout
        )

    def update(self):
        if self.send_future is None:
            if not self.client.server_is_ready():
                if self.node.get_clock().now() > self.server_deadline:
                    detail = (
                        f"{self.name}: no navigate_to_pose action server within "
                        f"{self.server_timeout:.0f}s"
                    )
                    self.node.get_logger().error(detail)
                    report_failure(self.blackboard, "internal", detail)
                    return py_trees.common.Status.FAILURE
                return py_trees.common.Status.RUNNING
            goal = NavigateToPose.Goal()
            goal.pose = self.blackboard.get(self.pose_key)
            goal.pose.header.stamp = self.node.get_clock().now().to_msg()
            p = goal.pose.pose.position
            self.node.get_logger().info(f"{self.name}: -> ({p.x:.2f}, {p.y:.2f})")
            self.send_future = self.client.send_goal_async(goal)
            return py_trees.common.Status.RUNNING

        if self.goal_handle is None:
            if not self.send_future.done():
                return py_trees.common.Status.RUNNING
            self.goal_handle = self.send_future.result()
            if not self.goal_handle.accepted:
                detail = f"{self.name}: Nav2 rejected the goal"
                self.node.get_logger().error(detail)
                report_failure(self.blackboard, "unreachable", detail)
                return py_trees.common.Status.FAILURE
            self.result_future = self.goal_handle.get_result_async()
            return py_trees.common.Status.RUNNING

        if not self.result_future.done():
            return py_trees.common.Status.RUNNING
        status = self.result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            return py_trees.common.Status.SUCCESS
        detail = f"{self.name}: Nav2 ended the goal with status {status}"
        self.node.get_logger().error(detail)
        # Aborted after Nav2 exhausted its own recoveries: the route existed
        # when it was planned. Recoverable, because corridors clear.
        report_failure(self.blackboard, "obstructed", detail)
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        in_flight = (
            self.goal_handle is not None
            and self.result_future is not None
            and not self.result_future.done()
        )
        if in_flight:
            self.node.get_logger().info(f"{self.name}: cancelling in-flight goal")
            self.goal_handle.cancel_goal_async()
