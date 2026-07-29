"""The drive mux's config agrees with the things it arbitrates between.

`twist_mux.yaml` is a table of topic names, priorities and timeouts that only
means anything relative to files it does not mention: the controller it feeds,
the Nav2 launch that publishes one of its inputs, and the Foxglove seam that
publishes the other. Every one of those relationships fails silently — a topic
renamed on one side leaves a mux input nobody publishes and a robot that does
not move, and a timeout changed on either side quietly removes the guaranteed
stop between an operator letting go and Nav2 driving again.

The arbitration itself — who actually wins, and what happens when a source stops
— is measured against a real mux in `test_twist_mux_arbitration.py`.
"""

import json
import pathlib
import sys

import pytest
import yaml
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch.utilities import perform_substitutions
from launch_ros.actions import Node

REPO = pathlib.Path(__file__).resolve().parents[2]
CONFIG = REPO / "mote_bringup" / "config" / "twist_mux.yaml"
CONTROLLERS = REPO / "mote_bringup" / "config" / "controllers.yaml"
NAV2_LAUNCH = REPO / "mote_bringup" / "launch" / "nav2_launch.py"
FOXGLOVE_LAUNCH = REPO / "mote_bringup" / "launch" / "foxglove_launch.py"
LAYOUT = REPO / "mote_bringup" / "foxglove" / "mote.json"
RVIZ = REPO / "mote_bringup" / "config" / "mote.rviz"
RECORD = REPO / "mote_bringup" / "config" / "record.yaml"
PIXI = REPO / "pixi.toml"

sys.path.insert(0, str(REPO / "mote_bringup" / "launch"))
import foxglove_launch  # noqa: E402
import mote_launch  # noqa: E402
import twist_mux_launch  # noqa: E402


@pytest.fixture
def context():
    ctx = LaunchContext()
    ctx.launch_configurations.update(
        {
            "map": "/tmp/test_map.yaml",
            "localisation": "true",
            "use_sim_time": "false",
            "teleop": "true",
            "teleop_topic": "/cmd_vel_teleop",
            "port": "8765",
            "address": "127.0.0.1",
        }
    )
    return ctx


def _remaps(node, context):
    """{from: to} for a launch_ros Node. launch exposes no public accessor."""
    return {
        perform_substitutions(context, list(src)): perform_substitutions(
            context, list(dst)
        )
        for src, dst in node._Node__remappings
    }


@pytest.fixture(scope="module")
def mux():
    return yaml.safe_load(CONFIG.read_text())["twist_mux"]["ros__parameters"]


@pytest.fixture(scope="module")
def diff_drive():
    cfg = yaml.safe_load(CONTROLLERS.read_text())
    return cfg["diff_drive_controller"]["ros__parameters"]


def test_teleop_outranks_navigation(mux):
    topics = mux["topics"]
    assert topics["teleop"]["priority"] > topics["navigation"]["priority"]


def test_priorities_are_in_the_range_twist_mux_accepts(mux):
    """Outside [1, 255] twist_mux clamps, or never selects the topic at all."""
    for name, entry in {**mux["topics"], **mux["locks"]}.items():
        assert 1 <= entry["priority"] <= 255, name


def test_letting_go_stops_the_robot_before_nav2_resumes(mux, diff_drive):
    """The property the whole design turns on.

    After the operator's last command the controller halts the wheels at
    cmd_vel_timeout, and the mux keeps Nav2 masked until the teleop input
    expires. With the teleop timeout the longer of the two there is always a
    stopped robot in between; invert them and the robot hands straight back
    mid-motion.
    """
    assert mux["topics"]["teleop"]["timeout"] > diff_drive["cmd_vel_timeout"]


def test_the_pause_lock_stops_navigation_and_not_teleop(mux):
    lock = mux["locks"]["pause_navigation"]["priority"]
    topics = mux["topics"]
    # twist_mux masks a topic whose priority is strictly below the lock's.
    assert topics["navigation"]["priority"] < lock
    assert topics["teleop"]["priority"] >= lock


def test_the_pause_lock_is_state_not_a_heartbeat(mux):
    """A non-zero timeout would engage the lock whenever nobody is publishing."""
    assert mux["locks"]["pause_navigation"]["timeout"] == 0.0


def test_the_teleop_relay_publishes_the_teleop_input(mux, context):
    description = foxglove_launch.generate_launch_description()
    relay = next(
        e
        for e in description.entities
        if isinstance(e, Node) and "twist_relay" in str(e._Node__node_executable)
    )
    assert _remaps(relay, context)["cmd_vel_out"] == mux["topics"]["teleop"]["topic"]


def test_the_base_launches_the_mux(context):
    """A base without it leaves Nav2 publishing a topic nobody forwards."""
    # An include's file path is only ever available as substitutions, so the
    # included descriptions are expanded and searched for the node itself —
    # which also catches the mux being included behind a condition nobody sets.
    found = []
    for entity in mote_launch.generate_launch_description().entities:
        if not isinstance(entity, IncludeLaunchDescription):
            continue
        included = entity.launch_description_source.get_launch_description(context)
        for node in included.entities:
            if isinstance(node, Node) and "twist_mux" in str(node._Node__package):
                found.append((entity.condition, node))
    assert len(found) == 1, "the base should launch exactly one drive mux"
    assert found[0][0] is None, "the drive path must not be optional"


def test_the_keyboard_teleop_task_publishes_the_teleop_input(mux):
    line = next(
        ln for ln in PIXI.read_text().splitlines() if ln.startswith("teleop = ")
    )
    assert mux["topics"]["teleop"]["topic"] in line


def test_the_rviz_teleop_panel_publishes_the_teleop_input(mux):
    """The bench equivalent of the Foxglove panel, and the same seam."""
    panel = yaml.safe_load(RVIZ.read_text())["Panels"]
    teleop = next(p for p in panel if "TeleopPanel" in p.get("Class", ""))
    assert teleop["Topic"] == mux["topics"]["teleop"]["topic"]
    assert teleop["Stamped"] is True


def test_the_mux_output_is_the_controllers_topic(context):
    description = twist_mux_launch.generate_launch_description()
    node = next(e for e in description.entities if isinstance(e, Node))
    assert _remaps(node, context)["cmd_vel_out"] == "/diff_drive_controller/cmd_vel"


@pytest.mark.parametrize("source", [FOXGLOVE_LAUNCH, NAV2_LAUNCH, RVIZ, PIXI])
def test_nothing_else_still_publishes_the_controllers_topic(source):
    """The point of the mux is that the controller has one publisher."""
    assert "/diff_drive_controller/cmd_vel" not in source.read_text()


def test_the_lock_panel_publishes_the_lock_topic(mux):
    panel = json.loads(LAYOUT.read_text())["configById"]["Publish!navlock"]
    assert panel["topicName"] == mux["locks"]["pause_navigation"]["topic"]
    assert panel["datatype"] == "std_msgs/msg/Bool"


def test_the_bag_records_both_mux_inputs(mux):
    topics = yaml.safe_load(RECORD.read_text())["streams"]["lite"]["topics"]
    for entry in mux["topics"].values():
        assert entry["topic"] in topics
