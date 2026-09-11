"""Bench bring-up for the SO-101 arm: the control stack, without the mission.

The arm servos share `/dev/mote_servos` with the drive wheels, so there is no
such thing as an arm process any more — `MoteHardware` owns the bus and exports
the arm's position interfaces alongside the wheels' velocity ones. During a
mission the arm therefore comes up with `pixi run robot` / `mapping` and needs
nothing extra; this launch exists for bench work, where the lidar, camera and
Nav2 are noise.

It starts the same controller_manager against the same URDF and the same
`controllers.yaml` the mission uses — including this robot's own arm
calibration — so what you jog on the bench is what runs on the robot. No diff_drive_controller is loaded, so nothing here can drive the
wheels, and the arm controller is loaded *inactive* — the arm is limp until
`pixi run arm-teleop` (or `switch_controllers --activate arm_controller`) asks it
to hold.

Teleop is `pixi run arm-teleop` in a second terminal beside this one; it is one
process holding the keyboard, the safety rules and the arm, so this launch
starts nothing on its behalf. See `mote_arm/TELEOP.md`.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue

from mote_bringup import param_overrides
from mote_bringup.launch_utils import (
    INACTIVE_CONTROLLERS,
    arm_config_file,
    controller_spawn_handler,
    joint_params_file,
    resolved_arm,
)


def generate_launch_description():
    description_share = get_package_share_directory("mote_description")
    bringup_share = get_package_share_directory("mote_bringup")

    with open(os.path.join(description_share, "config", "robot.yaml")) as f:
        cfg = yaml.safe_load(f)

    arm = resolved_arm(cfg)
    if arm is None:
        raise RuntimeError(
            "robot.yaml puts the arm on a different port from the drive wheels, "
            "so it is not part of the MoteHardware ros2_control component and "
            "this launch has nothing to bring up (see mote_arm/README.md)"
        )

    urdf_file = os.path.join(description_share, "urdf", "mote.urdf.xacro")
    robot_description = {
        "robot_description": ParameterValue(
            Command(f"xacro {urdf_file} arm_config:={arm_config_file(arm)}"),
            value_type=str,
        )
    }
    controller_config = param_overrides.override_path(
        "controllers", os.path.join(bringup_share, "config", "controllers.yaml")
    )

    respawn = {"respawn": True, "respawn_delay": 2.0}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        **respawn,
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config, joint_params_file(cfg, arm)],
        **respawn,
    )

    return LaunchDescription(
        [
            SetParameter(name="use_sim_time", value=False),
            robot_state_publisher,
            controller_manager,
            controller_spawn_handler(
                controller_manager,
                active=("joint_state_broadcaster",),
                inactive=INACTIVE_CONTROLLERS,
            ),
        ]
    )
