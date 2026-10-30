"""A promoted map waits for the mission in flight, and brings its zones with it.

The real task server and the real ``map_reloader``, with a fake ``map_server``
and a mock Nav2 whose goal is held until the test lets it finish. What is
proved is the rule the reloader exists to keep: Nav2 is never handed a new map
under a live goal, and the lane holder — not the reloader's guess — says when
the goal is over.
"""

import os
import random
import threading
import time

import pytest
import rclpy
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import LoadMap
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

import mission_harness as harness
from mote_bringup import map_reload
from mote_bringup.map_reloader import LATCHED, MapReloader
from mote_bringup.spec import mission

from mote_tasks.task_server import TaskServer

OLD, NEW = "20260802T203339", "20260921T133157"

OLD_ZONES = "frame_id: map\nzones:\n  kitchen: {x: 1.0, y: 2.0}\n"
NEW_ZONES = (
    "frame_id: map\nzones:\n  kitchen: {x: 1.0, y: 2.0}\n"
    "  store_room: {x: -3.0, y: 0.5}\n"
)


class Robot(Node):
    """Nav2's two faces here — the goal server and map_server — plus the agent."""

    def __init__(self):
        super().__init__("robot")
        self.statuses = []
        self.loads = []
        self.serving = []
        self.release = threading.Event()
        group = ReentrantCallbackGroup()
        self.nav = ActionServer(
            self,
            NavigateToPose,
            "navigate_to_pose",
            self._execute,
            callback_group=group,
        )
        self.create_service(
            LoadMap, "map_server/load_map", self._load, callback_group=group
        )
        self.command_pub = self.create_publisher(String, "task/command", 1)
        self.installed_pub = self.create_publisher(
            String, map_reload.INSTALLED_TOPIC, LATCHED
        )
        self.create_subscription(
            String,
            map_reload.SERVING_TOPIC,
            lambda m: self.serving.append(map_reload.parse(m.data)),
            LATCHED,
        )
        harness.collect(self, self.statuses)
        harness.localise(self)

    def _execute(self, goal_handle):
        # Holds the goal the way a robot crossing a building does.
        self.release.wait(timeout=30.0)
        goal_handle.succeed()
        return NavigateToPose.Result()

    def _load(self, request, response):
        self.loads.append(request.map_url)
        response.result = LoadMap.Response.RESULT_SUCCESS
        return response

    def served(self):
        return self.serving[-1]["revision"] if self.serving else None


@pytest.fixture
def ros():
    os.environ["ROS_DOMAIN_ID"] = str(random.randint(60, 100))
    rclpy.init(args=["--ros-args", "-r", f"__ns:=/test_{os.getpid()}"])
    yield
    rclpy.shutdown()


@pytest.fixture
def floor(tmp_path, monkeypatch):
    monkeypatch.setenv("MOTE_HOME", str(tmp_path))
    from mote_bringup import sites

    fdir = sites.floor_dir("home", "ground")
    for rev in (OLD, NEW):
        (fdir / "maps" / rev).mkdir(parents=True)
        (fdir / "maps" / rev / "map.yaml").write_text("image: map.png\n")
    sites._publish_revision(fdir, OLD)
    (fdir / "zones.yaml").write_text(OLD_ZONES)
    return fdir


@pytest.fixture
def rig(ros, floor):
    server = TaskServer(
        parameter_overrides=[
            Parameter("zones_file", value=str(floor)),
            Parameter("platform_id", value=harness.PLATFORM),
            Parameter("tick_period", value=0.05),
        ]
    )
    reloader = MapReloader(
        parameter_overrides=[
            Parameter("map", value=str(floor / "map" / "map.yaml")),
            Parameter("poll_period", value=0.1),
        ]
    )
    robot = Robot()
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (server, reloader, robot):
        executor.add_node(node)
    yield executor, server, reloader, robot
    robot.release.set()
    reloader.stop()
    executor.shutdown()
    for node in (server, reloader, robot):
        node.destroy_node()


def spin_until(executor, condition, timeout=20.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return True
        executor.spin_once(timeout_sec=0.05)
    return condition()


def spin(executor, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)


def install(floor, robot):
    """What the agent's pull does: flip the floor, replace its zones, say so."""
    from mote_bringup import sites

    sites._publish_revision(floor, NEW)
    (floor / "zones.yaml").write_text(NEW_ZONES)
    robot.installed_pub.publish(
        String(data=map_reload.installed("home", "ground", NEW))
    )


def test_a_promotion_during_a_mission_loads_when_the_mission_ends(rig, floor):
    executor, server, _, robot = rig
    harness.ready(executor, server, robot.command_pub)
    assert spin_until(executor, lambda: robot.served() == OLD)

    sent = harness.send(robot.command_pub, "goto", {"target": "kitchen"})
    assert spin_until(
        executor, lambda: mission.ACCEPTED in harness.states(robot.statuses)
    ), robot.statuses

    install(floor, robot)
    spin(executor, 1.5)
    assert robot.loads == [], "the map changed under a live goal"
    assert robot.served() == OLD
    assert "store_room" not in server.zones

    robot.release.set()
    assert spin_until(executor, lambda: robot.served() == NEW), robot.serving
    assert robot.statuses[-1]["id"] == sent["id"]
    assert robot.statuses[-1]["state"] == mission.SUCCEEDED
    assert robot.loads == [str(floor / "maps" / NEW / "map.yaml")]
    # The promoted revision's zones came with it, with no restart.
    assert spin_until(executor, lambda: "store_room" in server.zones)


def test_an_idle_robot_loads_at_once_and_goto_reaches_a_new_zone(rig, floor):
    executor, server, _, robot = rig
    harness.ready(executor, server, robot.command_pub)
    assert spin_until(executor, lambda: robot.served() == OLD)

    install(floor, robot)
    assert spin_until(executor, lambda: robot.served() == NEW)
    assert spin_until(executor, lambda: "store_room" in server.zones)

    robot.release.set()
    harness.send(robot.command_pub, "goto", {"target": "store_room"})
    assert spin_until(
        executor, lambda: mission.SUCCEEDED in harness.states(robot.statuses)
    ), robot.statuses


def test_a_mission_sent_while_the_map_loads_is_refused_as_busy(rig):
    executor, server, _, robot = rig
    harness.ready(executor, server, robot.command_pub)
    assert spin_until(executor, lambda: server.map_revision == OLD)
    # Stand in for the reloader mid-load: the lane is held by the map.
    server.on_map_serving(
        String(data=map_reload.serving("home", "ground", OLD, loading=NEW))
    )
    harness.send(robot.command_pub, "goto", {"target": "kitchen"})
    assert spin_until(executor, lambda: harness.failures(robot.statuses))
    assert harness.failures(robot.statuses)[-1] == (mission.REJECTED, mission.BUSY)
    assert NEW in robot.statuses[-1]["failure"]["detail"]
    assert robot.statuses[-1]["failure"]["recoverable"] is True
