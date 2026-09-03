"""The controller spawners must survive a controller_manager respawn.

A launch action may only ever be executed once. `respawn=True` on
controller_manager makes its OnProcessStart handler fire again on every restart,
so the handler must produce *fresh* spawner actions each time. Registering fixed
Node actions raises "executed more than once", and that exception aborts the
entire launch, taking every other node down with it.
"""

from launch.actions import OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

from mote_bringup.launch_utils import (
    CONTROLLERS,
    MIRROR_ARG,
    arm_mirror_node,
    controller_spawn_handler,
    declare_mirror_arg,
    spawn_controllers,
)


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


def test_the_mirror_switch_is_declared_the_same_way_wherever_it_appears():
    """One definition, because two launch files disagreeing costs a terminal.

    `mirror:=` was on arm_launch.py alone, so a bench session needing the camera
    ran `pixi run launch` and then arm_mirror in a window of its own — a third
    terminal that existed only because the two files differed.
    """
    arg = declare_mirror_arg()
    assert arg.name == MIRROR_ARG == "mirror"
    # Off by default: arm-jog, arm-pose and episode replay all command
    # arm_controller too, and none wants a second thing driving the arm.
    assert arg.default_value[0].text == "false"


def test_the_mirror_node_is_the_arm_package_s_own():
    node = arm_mirror_node()
    assert node.node_package == "mote_arm"
    assert node.node_executable == "arm_mirror"
