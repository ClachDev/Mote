"""Launch helpers that need to be unit-testable outside a launch run.

Launch files live under ``launch/`` and are loaded by path, not imported, so
logic worth testing lives here in the package instead.
"""

import tempfile

import yaml
from launch.actions import OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

CONTROLLERS = ("joint_state_broadcaster", "diff_drive_controller")

# Loaded and configured but left *inactive*. Activating a controller is what
# claims its command interfaces, and for the arm that is what enables servo
# torque (MoteHardware::perform_command_mode_switch) — so an arm nobody has
# asked to move stays limp, exactly as it did under the standalone driver.
# `pixi run arm-jog` (or the task layer) activates it on demand.
INACTIVE_CONTROLLERS = ("arm_controller",)


def arm_on_wheel_bus(cfg):
    """True when the arm is part of the wheel bus's ros2_control component.

    The same condition mote.urdf.xacro applies before emitting the arm's
    <joint> tags: one process owns one serial port, so the arm belongs to
    MoteHardware exactly when it shares that port. On a separate bus it would
    need a hardware component of its own, and there would be no arm_controller
    to spawn here either.
    """
    arm = cfg.get("arm") or {}
    return bool(arm) and arm.get("port") == cfg["servos"]["port"]


def joint_params_file(cfg):
    """Write the per-controller joint parameters out and return the path.

    These have to be a params *file* keyed by node name: a plain dict gets
    flattened to "diff_drive_controller.ros__parameters.wheel_separation" on
    the controller_manager node and never reaches the controller itself. The
    values come from robot.yaml so it stays the single source of truth for
    wheel geometry and for which joints the arm has.
    """
    params = {
        "diff_drive_controller": {
            "ros__parameters": {
                "wheel_separation": cfg["wheel_separation"],
                "wheel_radius": cfg["wheel_radius"],
            }
        }
    }
    if arm_on_wheel_bus(cfg):
        params["arm_controller"] = {
            "ros__parameters": {
                "joints": [j["name"] for j in cfg["arm"]["joints"]],
            }
        }

    handle = tempfile.NamedTemporaryFile(
        mode="w", prefix="mote_joint_params_", suffix=".yaml", delete=False
    )
    yaml.safe_dump(params, handle)
    handle.close()
    return handle.name


def spawn_controllers(_context=None, *_args, active=CONTROLLERS, inactive=()):
    """Fresh ``spawner`` actions for every controller, built per call.

    Must return NEW action objects each time: a launch action may only be
    executed once, and this runs again on every controller_manager restart.
    """
    return [
        Node(package="controller_manager", executable="spawner", arguments=[name])
        for name in active
    ] + [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--inactive"],
        )
        for name in inactive
    ]


def controller_spawn_handler(controller_manager, active=CONTROLLERS, inactive=()):
    """Spawn the controllers whenever ``controller_manager`` starts.

    Wrapping the spawners in an OpaqueFunction is what makes a
    ``respawn=True`` controller_manager survivable: registering pre-built Node
    actions instead raises "executed more than once" on the second start, and
    that exception aborts the entire launch, taking every other node with it.
    Re-spawning is also required for recovery to mean anything — a restarted
    controller_manager comes back with no controllers loaded.
    """
    return RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[
                OpaqueFunction(
                    function=spawn_controllers,
                    kwargs={"active": active, "inactive": inactive},
                )
            ],
        )
    )
