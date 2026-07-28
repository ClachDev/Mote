"""The wheel-odometry prior survives being composed into a container.

`localization_launch.py` holds two halves of one mechanism: a relay that writes
the inverted wheel pose to a TF leaf, and kinematic_icp, which reads that leaf
as its motion prior. Both are components in one container now, and both of the
ways that arrangement breaks are silent. A composable node loaded without a
`name` is matched against no parameter section and takes defaults; a plugin
string naming no registered component is one line in a container log while
everything else comes up around it. Either way the stack starts, TF looks
populated, and kinematic_icp simply runs without a prior.

The same is true of the frame name itself: the relay writing `odom_wheel` while
kinematic_icp waits on something else is not an error anywhere, so the two are
checked against each other here rather than trusted to stay in step.
"""

import pathlib
import sys

import pytest
from ament_index_python.resources import get_resource
from launch import LaunchContext
from launch.utilities import perform_substitutions
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.utilities import evaluate_parameters

REPO = pathlib.Path(__file__).resolve().parents[2]

sys.path.insert(0, str(REPO / "mote_bringup" / "launch"))
import localization_launch  # noqa: E402

RELAY = "odom_tf_relay"
ICP = "online_node"


@pytest.fixture
def context():
    ctx = LaunchContext()
    ctx.launch_configurations.update({"use_sim_time": "false"})
    return ctx


@pytest.fixture
def description():
    return localization_launch.generate_launch_description()


def _load_action(description):
    return next(e for e in description.entities if isinstance(e, LoadComposableNodes))


def _composables(description):
    return _load_action(description)._LoadComposableNodes__composable_node_descriptions


def _loaded(description, context):
    """{node name: composable node description}."""
    return {
        (perform_substitutions(context, n.node_name) if n.node_name else ""): n
        for n in _composables(description)
    }


def _params(node, context):
    return evaluate_parameters(context, node.parameters)[0]


@pytest.mark.parametrize(
    "package,plugin",
    [
        ("mote_nav", "mote_nav::OdomTfRelay"),
        # The component lives in the `kinematic_icp` ament package but registers
        # under its `kinematic_icp_ros` namespace, so the pair is checked
        # against the index rather than derived from either name.
        ("kinematic_icp", "kinematic_icp_ros::OnlineNode"),
    ],
)
def test_plugin_is_a_registered_component(package, plugin):
    content, _ = get_resource("rclcpp_components", package)
    registered = {line.split(";")[0] for line in content.splitlines() if line.strip()}
    assert plugin in registered, f"{plugin} is not a component of {package}"


def test_both_halves_are_loaded_and_named(description, context):
    loaded = _loaded(description, context)
    assert set(loaded) == {RELAY, ICP}


def test_the_relay_reads_the_controller_odometry(description, context):
    relay = _loaded(description, context)[RELAY]
    # Each half of a remapping is already a sequence of substitutions.
    remaps = {
        perform_substitutions(context, list(src)): perform_substitutions(
            context, list(dst)
        )
        for src, dst in relay.remappings
    }
    assert remaps["odom_in"] == "/diff_drive_controller/odom"


def test_icp_reads_the_leaf_the_relay_writes(description, context):
    """The one coupling that fails without failing anything."""
    loaded = _loaded(description, context)
    written = _params(loaded[RELAY], context)["child_frame"]
    read = _params(loaded[ICP], context)["wheel_odom_frame"]
    assert written == read == localization_launch.WHEEL_ODOM_FRAME


def test_icp_still_owns_the_odom_edge(description, context):
    """Composition must not change who publishes odom->base."""
    icp = _params(_loaded(description, context)[ICP], context)
    assert icp["publish_odom_tf"] is True
    assert icp["invert_odom_tf"] is False
    assert icp["lidar_odom_frame"] == "odom"
    assert icp["base_frame"] == "base_footprint"


def test_nothing_is_launched_as_its_own_process_but_the_container(description):
    """The relay is a component now; a stray Node action would run it twice."""
    nodes = [e for e in description.entities if isinstance(e, Node)]
    assert len(nodes) == 1
    assert "rclcpp_components" in str(nodes[0]._Node__package)


def test_the_container_is_isolated_not_shared(description):
    """Each component needs the executor it had as a process of its own.

    kinematic_icp blocks in its scan callback while it registers a frame; on a
    shared-executor container that would stall the relay behind it.
    """
    container = next(e for e in description.entities if isinstance(e, Node))
    assert "component_container_isolated" in str(container._Node__node_executable)
