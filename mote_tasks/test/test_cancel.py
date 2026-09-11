"""Cancelling a mission: the tree halts, Nav2's goal ends, the lane frees.

The mock Nav2 here does not succeed at once, as the other tree tests' does: it
holds each goal open until the goal is cancelled or the test releases it,
which is what a robot halfway down a corridor looks like to the task server.
That is the case a cancel exists for, and the only one where "cancelled means
it stopped" can be told apart from "cancelled means a cancel was sent".
"""

import json
import os
import random

import py_trees
import pytest
import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.task import Future
from std_msgs.msg import String

import mission_harness as harness
from mote_bringup.spec import mission

from mote_tasks.behaviours.nav import DriveTo
from mote_tasks.behaviours.perception import LABELS_QOS
from mote_tasks.task_server import TaskServer

ZONES = """\
frame_id: map
zones:
  kitchen: {x: -1.5, y: 0.5, yaw: 3.14}
  lab: {x: 4.0, y: -2.0}
"""


class HoldingNav(Node):
    """A Nav2 that drives for as long as nobody stops it.

    ``honour_cancel=False`` accepts a cancel and then never ends the goal,
    which is what a wedged controller server looks like from outside.
    """

    def __init__(self, honour_cancel=True):
        super().__init__("holding_nav")
        self.honour_cancel = honour_cancel
        self.goals = []
        self.cancels = 0
        self.ended = []
        self.statuses = []
        #: Goal ends and status arrivals, in the order this node saw them.
        self.timeline = []
        self.labels = []
        self.releases = []
        self.server = ActionServer(
            self,
            NavigateToPose,
            "navigate_to_pose",
            self.execute,
            cancel_callback=self.on_cancel,
        )
        self.command_pub = self.create_publisher(String, "task/command", 1)
        self.cancel_pub = self.create_publisher(String, "task/cancel", 10)
        self.create_subscription(String, "task/status", self._on_status, 10)
        self.create_subscription(String, "detect/labels", self._on_labels, LABELS_QOS)
        harness.localise(self, ("base_footprint", "base_link"))

    async def execute(self, goal_handle):
        self.goals.append(goal_handle.request.pose)
        release = Future()
        self.releases.append(release)
        await release
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.ended.append("canceled")
        else:
            goal_handle.succeed()
            self.ended.append("succeeded")
        self.timeline.append(f"goal {self.ended[-1]}")
        return NavigateToPose.Result()

    def _on_status(self, msg):
        status = json.loads(msg.data)
        self.statuses.append(status)
        self.timeline.append(f"status {status['state']}")

    def on_cancel(self, _goal_handle):
        self.cancels += 1
        if self.honour_cancel:
            self.releases[-1].set_result(True)
        return CancelResponse.ACCEPT

    def release(self):
        """Let the goal in flight reach its pose."""
        self.releases[-1].set_result(True)

    def _on_labels(self, msg):
        self.labels.append(msg.data)


@pytest.fixture
def ros():
    # Nothing outside this test may reach its task_server and mock nodes: a high
    # DDS domain keeps a live robot/sim session on this machine out, and a
    # per-process namespace keeps sibling test sessions out.
    os.environ["ROS_DOMAIN_ID"] = str(random.randint(60, 100))
    rclpy.init(args=["--ros-args", "-r", f"__ns:=/test_{os.getpid()}"])
    yield
    rclpy.shutdown()


def world(tmp_path, honour_cancel=True, **params):
    zones_file = tmp_path / "zones.yaml"
    zones_file.write_text(ZONES)
    overrides = [
        Parameter("zones_file", value=str(zones_file)),
        Parameter("platform_id", value=harness.PLATFORM),
    ] + [Parameter(name, value=value) for name, value in params.items()]
    server = TaskServer(parameter_overrides=overrides)
    nav = HoldingNav(honour_cancel=honour_cancel)
    executor = SingleThreadedExecutor()
    executor.add_node(server)
    executor.add_node(nav)
    harness.ready(executor, server, nav.command_pub)
    assert harness.spin_until(
        executor, lambda: nav.cancel_pub.get_subscription_count() > 0
    ), "task_server never subscribed to task/cancel"
    return server, nav, executor


def of(statuses, mission_id) -> list:
    return [status for status in statuses if status["id"] == mission_id]


def test_a_cancel_stops_the_drive_and_frees_the_lane(ros, tmp_path):
    server, nav, executor = world(tmp_path)

    first = harness.send(nav.command_pub, "goto", {"target": "kitchen"})
    assert harness.spin_until(executor, lambda: len(nav.goals) == 1), nav.statuses

    harness.cancel(
        nav.cancel_pub, first["id"], issued_by="ui:tester", reason="wrong room"
    )
    assert harness.spin_until(
        executor, lambda: mission.CANCELLED in harness.states(nav.statuses)
    ), nav.statuses

    # Nav2 was asked to stop, and had stopped, before the status went out:
    # `cancelled` is reported off the goal's end, not off the request.
    assert nav.cancels == 1
    assert nav.timeline == ["status accepted", "goal canceled", "status cancelled"]
    done = of(nav.statuses, first["id"])[-1]
    assert (done["state"], done["terminal"], done["failure"]) == (
        mission.CANCELLED,
        True,
        None,
    )
    assert "cancelled by ui:tester: wrong room" in done["detail"]
    assert server.mission is None

    # The lane is free at once: the next mission is taken, not refused busy.
    second = harness.send(nav.command_pub, "goto", {"target": "lab"})
    assert harness.spin_until(executor, lambda: of(nav.statuses, second["id"])), (
        nav.statuses
    )
    assert harness.states(of(nav.statuses, second["id"])) == [mission.ACCEPTED]

    # And it drives: the halted tree restarts from WaitForTask, not mid-drive.
    assert harness.spin_until(executor, lambda: len(nav.goals) == 2), nav.goals
    assert nav.goals[1].pose.position.x == pytest.approx(4.0)
    nav.release()
    assert harness.spin_until(
        executor,
        lambda: mission.SUCCEEDED in harness.states(of(nav.statuses, second["id"])),
    ), nav.statuses

    # Exactly one terminal status per mission, and a late cancel is a no-op.
    harness.cancel(nav.cancel_pub, first["id"])
    harness.spin_until(executor, lambda: False, timeout=0.5)
    assert harness.states(of(nav.statuses, first["id"])) == [
        mission.ACCEPTED,
        mission.CANCELLED,
    ]
    assert nav.cancels == 1

    server.destroy_node()
    nav.destroy_node()


def test_a_cancel_naming_another_mission_is_a_no_op(ros, tmp_path):
    server, nav, executor = world(tmp_path)

    running = harness.send(nav.command_pub, "goto", {"target": "kitchen"})
    assert harness.spin_until(executor, lambda: len(nav.goals) == 1)

    harness.cancel(nav.cancel_pub, "somebody-elses-mission")
    harness.spin_until(executor, lambda: False, timeout=0.5)
    assert nav.cancels == 0
    assert not of(nav.statuses, "somebody-elses-mission")
    assert server.mission["id"] == running["id"]

    nav.release()
    assert harness.spin_until(
        executor, lambda: mission.SUCCEEDED in harness.states(nav.statuses)
    ), nav.statuses

    server.destroy_node()
    nav.destroy_node()


def test_a_cancel_nav2_never_confirms_fails_the_mission_rather_than_lying(
    ros, tmp_path
):
    """``cancelled`` means it stopped. If Nav2 accepts the cancel and the goal
    never ends, the task server does not know that, and must not say it."""
    server, nav, executor = world(tmp_path, honour_cancel=False, cancel_timeout=1.0)

    stuck = harness.send(nav.command_pub, "goto", {"target": "kitchen"})
    assert harness.spin_until(executor, lambda: len(nav.goals) == 1)
    harness.cancel(nav.cancel_pub, stuck["id"])

    # Held while the goal is alive: a mission sent now is refused busy.
    assert harness.spin_until(executor, lambda: nav.cancels == 1)
    blocked = harness.send(nav.command_pub, "goto", {"target": "lab"})
    assert harness.spin_until(executor, lambda: of(nav.statuses, blocked["id"]))
    assert harness.failures(of(nav.statuses, blocked["id"])) == [
        (mission.REJECTED, mission.BUSY)
    ]

    assert harness.spin_until(
        executor,
        lambda: harness.failures(of(nav.statuses, stuck["id"])),
        timeout=10.0,
    ), nav.statuses
    ended = of(nav.statuses, stuck["id"])[-1]
    assert (ended["state"], ended["failure"]["class"]) == (
        mission.FAILED,
        mission.INTERNAL,
    )
    assert "drive_to_zone" in ended["failure"]["detail"]
    assert mission.CANCELLED not in harness.states(nav.statuses)
    assert server.mission is None

    server.destroy_node()
    nav.destroy_node()


def test_a_fetch_cancelled_while_searching_stops_the_detector(ros, tmp_path):
    """No Nav2 goal to wait for: the cancel is immediate, and the behaviour
    that was running still cleans up after itself."""
    server, nav, executor = world(tmp_path)

    fetch = harness.send(
        nav.command_pub, "fetch", {"target": "red_box", "destination": "kitchen"}
    )
    assert harness.spin_until(executor, lambda: nav.labels == ["red box"]), nav.labels

    harness.cancel(nav.cancel_pub, fetch["id"])
    assert harness.spin_until(
        executor, lambda: mission.CANCELLED in harness.states(nav.statuses)
    ), nav.statuses
    assert harness.spin_until(executor, lambda: nav.labels[-1:] == [""]), nav.labels
    assert nav.goals == []

    server.destroy_node()
    nav.destroy_node()


# -- DriveTo, without a graph ----------------------------------------------


class FakeHandle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.cancelled = False
        self.result = Future()

    def cancel_goal_async(self):
        self.cancelled = True
        return Future()

    def get_result_async(self):
        return self.result


class FakeLogger:
    def info(self, *_args, **_kwargs):
        pass

    warning = error = info


class FakeNode:
    def get_logger(self):
        return FakeLogger()


def a_drive(send_future, handle=None):
    """A DriveTo part-way through ``update()``: sent, and maybe answered."""
    drive = DriveTo("drive", "pose")
    drive.node = FakeNode()
    drive.send_future = send_future
    drive.goal_handle = handle
    drive.result_future = None if handle is None else handle.get_result_async()
    drive.unanswered = None
    drive.stopping = None
    return drive


def test_a_halt_before_nav2_answers_cancels_the_goal_once_it_is_accepted():
    """The window a halt can fall into: the goal is sent and Nav2 has not
    answered. Dropping the future there would leave Nav2 to accept a goal a
    moment later and drive it with no mission holding the lane."""
    sent = Future()
    drive = a_drive(sent)
    drive.stop(py_trees.common.Status.INVALID)
    assert drive.halting(), "a goal Nav2 may yet accept was forgotten"

    handle = FakeHandle(accepted=True)
    sent.set_result(handle)
    assert handle.cancelled, "the late goal was accepted and never cancelled"
    assert drive.halting(), "reported stopped before Nav2 ended the goal"

    handle.result.set_result(object())
    assert not drive.halting()


def test_a_halt_before_nav2_refuses_the_goal_has_nothing_to_stop():
    sent = Future()
    drive = a_drive(sent)
    drive.stop(py_trees.common.Status.INVALID)
    sent.set_result(FakeHandle(accepted=False))
    assert not drive.halting()


def test_a_halt_mid_drive_waits_for_the_goal_to_end():
    handle = FakeHandle()
    drive = a_drive(Future(), handle)
    drive.stop(py_trees.common.Status.INVALID)
    assert handle.cancelled
    assert drive.halting()
    handle.result.set_result(object())
    assert not drive.halting()


def test_a_behaviour_that_never_sent_a_goal_has_nothing_to_stop():
    drive = a_drive(None)
    drive.stop(py_trees.common.Status.INVALID)
    assert not drive.halting()
