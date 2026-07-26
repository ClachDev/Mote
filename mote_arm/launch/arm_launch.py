"""Launch the arm driver (joint states + safe jog target control).

Not part of the mission bringup — run alongside `pixi run mapping` / `robot`
like the perception stack:  `pixi run arm`.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            SetParameter(
                name="use_sim_time", value=LaunchConfiguration("use_sim_time")
            ),
            Node(
                package="mote_arm",
                executable="arm_driver",
                name="arm_driver",
                output="screen",
            ),
        ]
    )
