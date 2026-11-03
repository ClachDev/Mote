"""The controller spawners must survive a controller_manager respawn.

A launch action may only ever be executed once. `respawn=True` on
controller_manager makes its OnProcessStart handler fire again on every restart,
so the handler must produce *fresh* spawner actions each time. Registering fixed
Node actions raises "executed more than once", and that exception aborts the
entire launch, taking every other node down with it.
"""

import importlib
import pathlib
import sys

import pytest
import yaml
from launch.actions import OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

from mote_bringup.launch_utils import (
    CONTROLLERS,
    controller_spawn_handler,
    spawn_controllers,
)

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "launch"))


def test_each_call_returns_new_action_objects():
    first = spawn_controllers()
    second = spawn_controllers()
    assert len(first) == len(CONTROLLERS)
    # Identity matters: reusing an action object is exactly what launch rejects.
    for a, b in zip(first, second):
        assert a is not b


def test_spawns_every_controller():
    spawned = set()
    for action in spawn_controllers():
        # Node stores its arguments as normalized substitutions; each spawner
        # gets exactly one, the controller name.
        for arg in action.__dict__["_Node__arguments"]:
            spawned.add(arg[0].text if isinstance(arg, list) else str(arg))
    assert spawned == set(CONTROLLERS)


def _registered_actions(handler):
    return handler.event_handler._OnActionEventBase__actions_on_event


def test_handler_registers_an_opaque_function_not_bare_nodes():
    handler = controller_spawn_handler(
        Node(package="controller_manager", executable="ros2_control_node")
    )
    assert isinstance(handler, RegisterEventHandler)
    assert isinstance(handler.event_handler, OnProcessStart)

    registered = _registered_actions(handler)
    assert len(registered) == 1
    # An OpaqueFunction can be executed repeatedly; a Node cannot.
    assert isinstance(registered[0], OpaqueFunction)
    assert not any(isinstance(e, Node) for e in registered)


def test_opaque_function_yields_fresh_spawners_on_repeated_execution():
    handler = controller_spawn_handler(
        Node(package="controller_manager", executable="ros2_control_node")
    )
    fn = _registered_actions(handler)[0]
    # Executing twice models controller_manager starting, dying, and respawning.
    first = fn.execute(None)
    second = fn.execute(None)
    assert len(first) == len(second) == len(CONTROLLERS)
    assert all(isinstance(p, Node) for p in first + second)
    for a, b in zip(first, second):
        assert a is not b


def _arguments(action):
    return [
        arg[0].text if isinstance(arg, list) else str(arg)
        for arg in action.__dict__["_Node__arguments"]
    ]


def test_every_spawner_loads_every_param_file():
    """ros2_control 6 gives a controller only what its spawner loads.

    The controller_manager's own parameter files are not passed on, so a
    spawner without them leaves its controller on defaults: diff_drive refuses
    a zero wheel_separation, but cmd_vel_timeout or the arm's
    interpolate_from_desired_state would fall back with no error at all.
    """
    files = ("/x/controllers.yaml", "/y/mote_joint_params.yaml")
    actions = spawn_controllers(inactive=("arm_controller",), param_files=files)
    assert len(actions) == len(CONTROLLERS) + 1
    for action in actions:
        args = _arguments(action)
        loaded = [args[i + 1] for i, a in enumerate(args) if a == "--param-file"]
        assert loaded == list(files), args[0]


def _spawn_param_files(description):
    handlers = [e for e in description.entities if isinstance(e, RegisterEventHandler)]
    files = [
        fn._OpaqueFunction__kwargs.get("param_files")
        for h in handlers
        for fn in _registered_actions(h)
        if isinstance(fn, OpaqueFunction)
        and fn._OpaqueFunction__function is spawn_controllers
    ]
    assert len(files) == 1
    return files[0]


@pytest.mark.parametrize("launch_file", ["mote_launch", "arm_launch"])
def test_the_launches_hand_both_controller_files_to_the_spawners(launch_file):
    module = importlib.import_module(launch_file)
    files = _spawn_param_files(module.generate_launch_description())
    assert any(pathlib.Path(f).name == "controllers.yaml" for f in files)
    joint_params = [
        f for f in files if pathlib.Path(f).name.startswith("mote_joint_params_")
    ]
    assert len(joint_params) == 1
    with open(joint_params[0]) as f:
        assert (
            "wheel_separation"
            in yaml.safe_load(f)["diff_drive_controller"]["ros__parameters"]
        )
