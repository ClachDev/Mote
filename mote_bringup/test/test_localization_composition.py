"""The wheel-odometry prior and the velocity gate survive being composed.

`localization_launch.py` holds three parts of one mechanism: a relay that writes
the inverted wheel pose to a TF leaf, kinematic_icp, which reads that leaf as
its motion prior, and `icp_odom_gate`, which accumulates kinematic_icp's
increments into odom->base while refusing any the drive could not have produced.
All three are components in one container, and every way that arrangement breaks
is silent. A composable node loaded without a `name` is matched against no
parameter section and takes defaults; a plugin string naming no registered
component is one line in a container log while everything else comes up around
it. Either way the stack starts and TF looks populated.

So the couplings are checked against each other here rather than trusted to stay
in step, because none of them fails loudly:

* the relay writing `odom_wheel` while kinematic_icp waits on something else
  costs the scan match its prior and reports nothing;
* the gate subscribing to a topic kinematic_icp does not publish leaves
  odom->base simply absent, which reads as a TF timing problem;
* kinematic_icp broadcasting odom->base again would put two publishers on one
  edge, and the ungated one would win roughly half the time.
"""

import pathlib
import sys

import pytest
import yaml
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
GATE = "icp_odom_gate"


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
        ("mote_nav", "mote_nav::IcpOdomGate"),
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


def test_every_part_is_loaded_and_named(description, context):
    loaded = _loaded(description, context)
    assert set(loaded) == {RELAY, ICP, GATE}


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


def test_the_gate_owns_the_odom_edge_alone(description, context):
    """Exactly one publisher of odom->base, and it is the gate.

    A TF broadcast cannot be retracted, so a gate downstream of kinematic_icp's
    own broadcast would be no gate at all: the bad transform would already be on
    the wire. Leaving `publish_odom_tf` on is the whole failure.
    """
    loaded = _loaded(description, context)
    icp = _params(loaded[ICP], context)
    gate = _params(loaded[GATE], context)
    assert icp["publish_odom_tf"] is False
    assert icp["lidar_odom_frame"] == localization_launch.ICP_ODOM_FRAME
    assert icp["lidar_odom_frame"] != gate["odom_frame"]
    assert gate["odom_frame"] == "odom"
    assert gate["base_frame"] == icp["base_frame"] == "base_footprint"


def test_the_gate_reads_the_topic_kinematic_icp_publishes(description, context):
    """Its namespace is part of the topic name, so the two must be read together."""
    loaded = _loaded(description, context)
    gate = loaded[GATE]
    remaps = {
        perform_substitutions(context, list(src)): perform_substitutions(
            context, list(dst)
        )
        for src, dst in gate.remappings
    }
    namespace = perform_substitutions(context, loaded[ICP].node_namespace)
    assert remaps["odom_in"] == f"/{namespace}/lidar_odometry"


def test_the_gate_and_icp_read_the_same_wheel_leaf(description, context):
    """Both take the wheel prior from TF; disagreeing costs the gate its fallback."""
    loaded = _loaded(description, context)
    assert (
        _params(loaded[GATE], context)["wheel_odom_frame"]
        == _params(loaded[ICP], context)["wheel_odom_frame"]
        == localization_launch.WHEEL_ODOM_FRAME
    )


def test_the_gate_bounds_itself_by_the_measured_hardware_envelope(description, context):
    """The same two numbers the Nav2 wheel-speed critic uses, from robot.yaml.

    A gate carrying its own copy would drift from the critic, and the two would
    then disagree about what the drive can do.
    """
    with open(REPO / "mote_description" / "config" / "robot.yaml") as f:
        robot = yaml.safe_load(f)
    gate = _params(_loaded(description, context)[GATE], context)
    assert gate["max_wheel_speed"] == robot["max_wheel_speed"]
    assert gate["wheel_separation"] == robot["wheel_separation"]
    # Measured: legitimate intervals reach x1.13 of the envelope and the
    # mildest excursion sits at x1.25, so the tolerance must land between.
    assert 1.13 < gate["tolerance"] < 1.25


def test_nothing_is_launched_as_its_own_process_but_the_container(description):
    """The relay is a component now; a stray Node action would run it twice."""
    nodes = [e for e in description.entities if isinstance(e, Node)]
    assert len(nodes) == 1
    assert "rclcpp_components" in str(nodes[0]._Node__package)


def test_the_container_is_isolated_not_shared(description):
    """Each component needs the executor it had as a process of its own.

    kinematic_icp blocks in its scan callback while it registers a frame, and
    the gate blocks waiting on TF for the wheel prior of a rejected increment;
    on a shared-executor container either would stall the relay behind it.
    """
    container = next(e for e in description.entities if isinstance(e, Node))
    assert "component_container_isolated" in str(container._Node__node_executable)
