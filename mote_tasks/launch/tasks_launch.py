from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    home_zones = Path.home() / ".mote" / "zones.yaml"
    default_zones = str(home_zones) if home_zones.exists() else ""

    task_server = Node(
        package="mote_tasks",
        executable="task_server",
        name="task_server",
        parameters=[{"zones_file": LaunchConfiguration("zones_file")}],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("zones_file", default_value=default_zones),
            SetParameter(
                name="use_sim_time", value=LaunchConfiguration("use_sim_time")
            ),
            task_server,
        ]
    )
