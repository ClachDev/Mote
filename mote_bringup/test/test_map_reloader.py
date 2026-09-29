"""``map_reloader`` against a fake map_server, a fake lane and a fake AMCL.

No Nav2: what is under test is the node's own decisions — which revision it
loads, when it may, what it reports, and that the pose estimate goes back to
AMCL afterwards. The same node against a real Nav2 is the sim check
(``mote_simulation/test/map_reload/``).
"""

import math
import os
import random
import time

import pytest

rclpy = pytest.importorskip("rclpy")

from geometry_msgs.msg import PoseWithCovarianceStamped  # noqa: E402
from nav2_msgs.srv import LoadMap  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

from mote_bringup import map_reload  # noqa: E402
from mote_bringup.map_reloader import LATCHED, MapReloader  # noqa: E402

OLD, NEW = "20260802T203339", "20260921T133157"


# ---- the ROS-free half --------------------------------------------------


@pytest.fixture
def home(tmp_path, monkeypatch):
    monkeypatch.setenv("MOTE_HOME", str(tmp_path))
    from mote_bringup import sites

    floor = sites.floor_dir("home", "ground")
    for rev in (OLD, NEW):
        (floor / "maps" / rev).mkdir(parents=True)
        (floor / "maps" / rev / "map.yaml").write_text("image: map.png\n")
    sites._publish_revision(floor, OLD)
    return floor


def test_the_launch_map_path_names_its_revision_through_the_symlink(home):
    assert map_reload.locate(str(home / "map" / "map.yaml")) == ("home", "ground", OLD)


def test_a_map_outside_the_sites_tree_has_no_floor(tmp_path, home):
    stray = tmp_path / "elsewhere.yaml"
    stray.write_text("image: x.png\n")
    assert map_reload.locate(str(stray)) is None
    assert map_reload.locate("") is None


def test_only_an_installation_on_the_served_floor_is_wanted():
    served = ("home", "ground", OLD)
    same = {"site": "home", "floor": "ground", "revision": NEW}
    assert map_reload.wanted(served, same) == NEW
    assert map_reload.wanted(served, {**same, "revision": OLD}) is None
    assert map_reload.wanted(served, {**same, "floor": "upstairs"}) is None
    assert map_reload.wanted(served, None) is None
    assert map_reload.wanted((None, None, None), same) is None


# ---- the node -----------------------------------------------------------


class Robot(Node):
    """map_server, the task server's lane, AMCL and the fleet agent, faked."""

    def __init__(self, load_result=LoadMap.Response.RESULT_SUCCESS):
        super().__init__("robot")
        self.loads = []
        self.load_result = load_result
        self.idle = True
        self.lane_asked = 0
        self.initial_poses = []
        self.serving = []
        self.create_service(LoadMap, "map_server/load_map", self._load)
        self.lane = None
        self.amcl_pub = self.create_publisher(
            PoseWithCovarianceStamped, "amcl_pose", LATCHED
        )
        self.installed_pub = self.create_publisher(
            String, map_reload.INSTALLED_TOPIC, LATCHED
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "initialpose", self._initial_pose, 10
        )
        self.create_subscription(
            String,
            map_reload.SERVING_TOPIC,
            lambda m: self.serving.append(map_reload.parse(m.data)),
            LATCHED,
        )

    def open_lane(self):
        self.lane = self.create_service(Trigger, map_reload.IDLE_SERVICE, self._idle)

    def _idle(self, _request, response):
        self.lane_asked += 1
        response.success = self.idle
        response.message = "idle" if self.idle else "mission m-1 (goto) holds lane"
        return response

    def _load(self, request, response):
        self.loads.append(request.map_url)
        response.result = self.load_result
        return response

    def _initial_pose(self, msg):
        # AMCL takes an initial pose and publishes its estimate from it.
        self.initial_poses.append(msg)
        self.amcl(msg.pose.pose.position.x, msg.pose.pose.position.y, 0.3)

    def amcl(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)
        self.amcl_pub.publish(msg)

    def install(self, revision, floor="ground"):
        self.installed_pub.publish(
            String(data=map_reload.installed("home", floor, revision))
        )


@pytest.fixture
def ros():
    os.environ["ROS_DOMAIN_ID"] = str(random.randint(60, 100))
    rclpy.init(args=["--ros-args", "-r", f"__ns:=/test_{os.getpid()}"])
    yield
    rclpy.shutdown()


@pytest.fixture
def rig(ros, home):
    made = []

    def build(**robot_kwargs):
        robot = Robot(**robot_kwargs)
        reloader = MapReloader(
            parameter_overrides=[
                Parameter("map", value=str(home / "map" / "map.yaml")),
                Parameter("poll_period", value=0.1),
                Parameter("settle", value=0.1),
                Parameter("pose_wait", value=2.0),
            ]
        )
        executor = SingleThreadedExecutor()
        executor.add_node(robot)
        executor.add_node(reloader)
        made.append((executor, robot, reloader))
        return executor, robot, reloader

    yield build
    for executor, robot, reloader in made:
        reloader.stop()
        executor.shutdown()
        reloader.destroy_node()
        robot.destroy_node()


def spin(executor, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)


def spin_until(executor, condition, timeout=10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return True
        executor.spin_once(timeout_sec=0.02)
    return condition()


def serving(robot):
    return robot.serving[-1] if robot.serving else {}


def test_it_reports_the_revision_the_launch_loaded(rig):
    executor, robot, _ = rig()
    assert spin_until(executor, lambda: serving(robot))
    assert serving(robot) == {
        "site": "home",
        "floor": "ground",
        "revision": OLD,
        "loading": None,
        "error": None,
    }


def test_an_idle_robot_loads_the_installed_revision_and_keeps_its_pose(rig, home):
    executor, robot, _ = rig()
    robot.open_lane()
    robot.amcl(2.5, -1.25, 0.3)
    spin(executor, 0.3)

    robot.install(NEW)
    assert spin_until(executor, lambda: serving(robot).get("revision") == NEW)
    assert robot.loads == [str(home / "maps" / NEW / "map.yaml")]
    # The load was announced before it happened, so the lane was held.
    assert any(s["loading"] == NEW for s in robot.serving)
    assert serving(robot)["loading"] is None

    assert spin_until(executor, lambda: robot.initial_poses)
    carried = robot.initial_poses[0].pose.pose.position
    assert (carried.x, carried.y) == (2.5, -1.25)


def test_a_mission_in_flight_defers_the_load_until_it_ends(rig):
    executor, robot, _ = rig()
    robot.open_lane()
    robot.idle = False
    robot.install(NEW)
    assert spin_until(executor, lambda: robot.lane_asked >= 3)
    assert robot.loads == []
    assert serving(robot).get("revision") == OLD

    robot.idle = True
    assert spin_until(executor, lambda: serving(robot).get("revision") == NEW)
    assert len(robot.loads) == 1


def test_with_no_task_server_there_is_no_mission_to_wait_for(rig):
    executor, robot, _ = rig()
    robot.install(NEW)
    assert spin_until(executor, lambda: serving(robot).get("revision") == NEW)


def test_a_failed_load_keeps_the_old_map_and_says_why(rig):
    executor, robot, _ = rig(load_result=LoadMap.Response.RESULT_INVALID_MAP_DATA)
    robot.amcl(1.0, 1.0, 0.0)
    robot.install(NEW)
    assert spin_until(executor, lambda: serving(robot).get("error"))
    assert serving(robot).get("revision") == OLD
    assert "invalid map data" in serving(robot)["error"]
    assert robot.initial_poses == []
    # Not retried in a loop: the same revision waits for a fresh install.
    spin(executor, 0.5)
    assert len(robot.loads) == 1
    robot.install(NEW)
    assert spin_until(executor, lambda: len(robot.loads) == 2)


def test_an_installation_on_another_floor_is_not_loaded(rig):
    executor, robot, _ = rig()
    robot.install(NEW, floor="upstairs")
    spin(executor, 0.6)
    assert robot.loads == []
    assert serving(robot).get("revision") == OLD


def test_no_map_server_leaves_nav_as_it_was(rig):
    executor, robot, reloader = rig()
    reloader.load_client.wait_for_service = lambda timeout_sec=None: False
    robot.install(NEW)
    assert spin_until(executor, lambda: serving(robot).get("error"))
    assert "not available" in serving(robot)["error"]
    assert serving(robot).get("revision") == OLD


def test_the_worker_stops_with_the_node(rig):
    _, _, reloader = rig()
    reloader.stop()
    assert not reloader._worker.is_alive()
