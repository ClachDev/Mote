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
    cancelled when the behaviour is preempted — including one Nav2 has not
    yet answered, which is cancelled the moment it is accepted.

    Stopping is asynchronous: the cancel goes out when the behaviour is halted,
    and Nav2 ends the goal some time after. :meth:`halting` says whether that
    has happened, which is what lets the task server report a mission
    ``cancelled`` only once the robot has actually stopped being driven.

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
        self.send_future = None
        self.goal_handle = None
        self.result_future = None
        #: The goal-send a halt caught unanswered, until Nav2 answers it.
        self.unanswered = None
        #: The result of a goal this behaviour cancelled, until Nav2 ends it.
        self.stopping = None

    def initialise(self):
        self.send_future = None
        self.goal_handle = None
        self.result_future = None
        self.unanswered = None
        self.stopping = None
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
        if self.send_future is not None and self.goal_handle is None:
            # Sent and not yet answered. Dropping it here would let Nav2 accept
            # it a moment later and drive a mission nobody holds any more.
            self.node.get_logger().info(f"{self.name}: cancelling unanswered goal")
            self.unanswered = self.send_future
            self.unanswered.add_done_callback(self._cancel_once_answered)
            return
        in_flight = (
            self.goal_handle is not None
            and self.result_future is not None
            and not self.result_future.done()
        )
        if in_flight:
            self.node.get_logger().info(f"{self.name}: cancelling in-flight goal")
            self.goal_handle.cancel_goal_async()
            self.stopping = self.result_future

    def _cancel_once_answered(self, future):
        """Runs on the executor once Nav2 answers a goal a halt caught in flight."""
        try:
            handle = future.result()
        except Exception as exc:  # the send itself failed: nothing to stop
            self.node.get_logger().warning(f"{self.name}: goal send failed: {exc}")
            handle = None
        if handle is not None and handle.accepted:
            handle.cancel_goal_async()
            self.stopping = handle.get_result_async()
        self.unanswered = None

    def halting(self) -> bool:
        """Whether a goal this behaviour sent is still alive after a halt.

        True from the halt until Nav2 has ended the goal: bt_navigator has
        halted its tree and the controller has accepted the cancel. The
        controller's zero velocity follows within one of its periods, and can
        land just after the goal ends (measured in the sim over five runs:
        16-48 ms after the cancel, once 16 ms after the goal ended). False
        when there was nothing to stop.
        """
        if self.unanswered is not None:
            return True
        return self.stopping is not None and not self.stopping.done()
