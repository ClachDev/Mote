"""Launch helpers that need to be unit-testable outside a launch run.

Launch files live under ``launch/`` and are loaded by path, not imported, so
logic worth testing lives here in the package instead.
"""

import tempfile

import yaml
from launch.actions import OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

from mote_arm import config as arm_config

CONTROLLERS = ("joint_state_broadcaster", "diff_drive_controller")

# The two odometry leaves hanging off the base, both written by
# localization_launch.py. They live here rather than in that file because
# mote_launch.py needs one of them too, and a launch file cannot import another.
#
# WHEEL_ODOM_FRAME is the inverted wheel pose kinematic_icp reads as its motion
# prior. ICP_ODOM_FRAME is kinematic_icp's own ungated track, deliberately not
# `odom`: icp_odom_gate owns odom->base and publishes it with the physically
# impossible increments taken out. slip_monitor reads the ungated leaf, because
# its `icp_fault` verdict fires on exactly what the gate removes.
#
# A disagreement between any writer and reader of these names costs the reader
# its input without failing anything loudly, which is why they are constants and
# why test_localization_composition.py checks them against each other.
WHEEL_ODOM_FRAME = "odom_wheel"
ICP_ODOM_FRAME = "odom_icp"

# Loaded and configured but left *inactive*. Activating a controller is what
# claims its command interfaces, and for the arm that is what enables servo
# torque (MoteHardware::perform_command_mode_switch) — so an arm nobody has
# asked to move stays limp, exactly as it did under the standalone driver.
# `pixi run arm-teleop` (or the task layer) activates it on demand.
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


def resolved_arm(cfg):
    """This robot's arm: packaged defaults overlaid with its own calibration.

    ``zero``/``min``/``max`` are measurements of one physical arm, so a
    calibrated robot keeps them in ``$MOTE_HOME/arm.yaml`` and robot.yaml
    carries only conservative placeholders (see `pixi run arm-setup calibrate`).
    ``mote_arm.config.load`` is the one implementation of that overlay, so it is
    used here rather than re-read — the alternative is two rules for what this
    robot's limits are, and the one that reached the hardware would be the wrong
    one.

    That matters more than it looks: calibration *moves the zero*, so an
    uncalibrated URDF does not merely clamp differently, it makes every
    commanded angle name a different physical position.

    Returns None when there is no arm on the wheel bus.
    """
    if not arm_on_wheel_bus(cfg):
        return None
    return arm_config.load()


def arm_config_file(spec):
    """Write ``spec`` out in robot.yaml's ``arm:`` shape, for xacro to load.

    xacro cannot resolve ``$MOTE_HOME`` or apply the calibration overlay, so the
    launch does it and hands over the answer.
    """
    data = {
        "port": spec.port,
        "baud_rate": spec.baud_rate,
        "moving_speed": spec.moving_speed,
        "moving_acc": spec.moving_acc,
        "joints": [
            {
                "name": joint.name,
                "id": joint.id,
                "min": joint.min_rad,
                "max": joint.max_rad,
                "zero": joint.zero_counts,
                "invert": joint.invert,
            }
            for joint in spec.joints
        ],
    }
    return _temp_yaml(data, "mote_arm_config_")


def joint_params_file(cfg, arm=None):
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
    if arm is not None:
        params["arm_controller"] = {"ros__parameters": {"joints": list(arm.names)}}

    return _temp_yaml(params, "mote_joint_params_")


def _temp_yaml(data, prefix):
    handle = tempfile.NamedTemporaryFile(
        mode="w", prefix=prefix, suffix=".yaml", delete=False
    )
    yaml.safe_dump(data, handle)
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
