from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

from mote_bringup import sites


def generate_launch_description():
    default_zones = sites.resolve_zones()

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
