"""Nav2 is composed into one container, and composition fails quietly.

Two of the ways it fails leave a stack that starts, reports every node
`active`, and navigates worse than it should — with no error anywhere. A
composable node loaded without a `name` is matched against no section of the
parameter file and silently takes library defaults; a plugin string that names
no registered component is one buried line in a container log while the rest of
the stack comes up around it. Both are caught here, against the real
`nav2_params.yaml` and the real ament index, with no container to launch.

The third failure is loud but total: an action may only be executed once, so
the component loads must be rebuilt on every container respawn or the whole
launch dies the first time Nav2 crashes. That is the same trap
`test_launch_utils.py` covers for the controller spawners.
"""

import pathlib
import sys

import pytest
import yaml
from ament_index_python.resources import get_resource
from launch import LaunchContext
from launch.actions import OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.utilities import perform_substitutions
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.utilities import evaluate_parameters

REPO = pathlib.Path(__file__).resolve().parents[2]
NAV2_PARAMS = REPO / "mote_bringup" / "config" / "nav2_params.yaml"
TWIST_MUX = REPO / "mote_bringup" / "config" / "twist_mux.yaml"

sys.path.insert(0, str(REPO / "mote_bringup" / "launch"))
import nav2_launch  # noqa: E402

SERVERS = nav2_launch.LOCALIZATION_SERVERS + nav2_launch.NAVIGATION_SERVERS
LIFECYCLE_MANAGER = (
    "nav2_lifecycle_manager",
    "nav2_lifecycle_manager::LifecycleManager",
)
# Configured in nav2_params.yaml but launched by nothing: `save-map` runs the
# map_saver CLI, not a map_saver server.
UNLAUNCHED_SECTIONS = {"map_saver"}


@pytest.fixture(scope="module")
def params():
    return yaml.safe_load(NAV2_PARAMS.read_text())


@pytest.fixture
def context():
    """A context with the launch arguments already resolved.

    The composable node descriptions carry `LaunchConfiguration`s, so they can
    only be evaluated against a context that has them.
    """
    ctx = LaunchContext()
    ctx.launch_configurations.update(
        {"map": "/tmp/test_map.yaml", "localisation": "true", "use_sim_time": "false"}
    )
    return ctx


@pytest.fixture
def description():
    return nav2_launch.generate_launch_description()


def _start_handler(description):
    handler = next(
        e for e in description.entities if isinstance(e, RegisterEventHandler)
    )
    assert isinstance(handler.event_handler, OnProcessStart)
    # Same private accessor as test_launch_utils: launch exposes no public one.
    return handler.event_handler._OnActionEventBase__actions_on_event


def _load_actions(description, context):
    """The LoadComposableNodes the container's start handler produces."""
    registered = _start_handler(description)
    assert len(registered) == 1
    return registered[0].execute(context)


def _composables(load_action):
    return load_action._LoadComposableNodes__composable_node_descriptions


def _name(node, context):
    """A composable node's name, or '' when it was loaded without one."""
    return perform_substitutions(context, node.node_name) if node.node_name else ""


def _loaded(description, context):
    """{node name: composable node description} across every load action."""
    found = {}
    for action in _load_actions(description, context):
        assert isinstance(action, LoadComposableNodes)
        for node in _composables(action):
            found[_name(node, context)] = node
    return found


@pytest.mark.parametrize(
    "package,plugin", [(p, pl) for p, pl, _ in SERVERS] + [LIFECYCLE_MANAGER]
)
def test_plugin_is_a_registered_component(package, plugin):
    """Only a registered component can be loaded into a container.

    Nav2's plugin strings do not all follow their package name — the behaviour
    server registers as `behavior_server::BehaviorServer` — so this is checked
    against the index rather than assumed.
    """
    content, _ = get_resource("rclcpp_components", package)
    registered = {line.split(";")[0] for line in content.splitlines() if line.strip()}
    assert plugin in registered, f"{plugin} is not a component of {package}"


def test_every_composable_node_is_named(description, context):
    """An unnamed composable node matches no section of the parameter file."""
    for name, node in _loaded(description, context).items():
        assert name, f"{node.package} loaded without a name"


def test_every_server_is_loaded(description, context):
    loaded = _loaded(description, context)
    for _, _, name in SERVERS:
        assert name in loaded


def test_every_params_section_reaches_a_loaded_node(params, description, context):
    """Every server section in the file names something that is really loaded.

    The costmap sections are excluded because they belong to nodes the servers
    create for themselves: those are not components, are never named in a load
    request, and reach their parameters through the container's own command
    line instead.
    """
    sections = {k for k, v in params.items() if "ros__parameters" in v}
    assert sections - UNLAUNCHED_SECTIONS <= set(_loaded(description, context))


def test_container_is_given_the_params_file(description):
    """Nav2's costmaps are not components and inherit the container's arguments.

    /local_costmap/local_costmap and /global_costmap/global_costmap are created
    by the servers themselves, so no load request ever names them; the only
    command line they can read is the container's. Drop this and they come up
    on library defaults — a slow robot, not a failed launch.
    """
    container = next(
        e
        for e in description.entities
        if isinstance(e, Node) and "rclcpp_components" in str(e._Node__package)
    )
    assert container._Node__parameters, "the nav2 container carries no params file"


def test_lifecycle_managers_manage_exactly_what_is_loaded(description, context):
    """A server that is loaded but unmanaged never leaves `unconfigured`."""
    for action in _load_actions(description, context):
        names = {_name(n, context) for n in _composables(action)}
        managers = [
            n for n in _composables(action) if "lifecycle_manager" in _name(n, context)
        ]
        assert len(managers) == 1, "each load should carry its own manager"
        manager_name = _name(managers[0], context)
        managed = evaluate_parameters(context, managers[0].parameters)[0]["node_names"]
        assert set(managed) == names - {manager_name}


def test_loads_are_rebuilt_for_every_container_start(description, context):
    """The container respawns; an already-executed action cannot run again."""
    first = _load_actions(description, context)
    second = _load_actions(description, context)
    assert len(first) == len(second)
    for a, b in zip(first, second):
        assert a is not b


def test_start_handler_registers_an_opaque_function_not_bare_loads(description):
    registered = _start_handler(description)
    assert all(isinstance(e, OpaqueFunction) for e in registered)
    assert not any(isinstance(e, LoadComposableNodes) for e in registered)


@pytest.mark.parametrize("server", ["controller_server", "behavior_server"])
def test_velocity_goes_to_the_drive_mux_not_the_controller(
    description, context, server
):
    """Nav2 is one input to the mux, and a recovery is arbitrated like a plan.

    The remap and the mux's table are separate files, and a mismatch is silent:
    Nav2 publishes happily to a topic nobody forwards and the robot simply does
    not move.
    """
    mux = yaml.safe_load(TWIST_MUX.read_text())["twist_mux"]["ros__parameters"]
    node = _loaded(description, context)[server]
    remaps = {
        perform_substitutions(context, list(src)): perform_substitutions(
            context, list(dst)
        )
        for src, dst in node.remappings
    }
    assert remaps["/cmd_vel"] == mux["topics"]["navigation"]["topic"]
