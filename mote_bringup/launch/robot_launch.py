import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory("mote_bringup"), "launch")

    default_map = os.path.join(os.path.expanduser("~"), ".mote", "map.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")

    def include(launch_file, condition=None, **extra_args):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, launch_file)),
            launch_arguments={"use_sim_time": use_sim_time, **extra_args}.items(),
            condition=condition,
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "base",
                default_value="true",
                description="Include the hardware base (mote_launch.py). Set false "
                "when a base is provided externally, e.g. by the sim.",
            ),
            DeclareLaunchArgument(
                "map",
                default_value=default_map,
                description="Full path to the map yaml file Nav2 should load",
            ),
            include(
                "mote_launch.py", condition=IfCondition(LaunchConfiguration("base"))
            ),
            include("nav2_launch.py", map=LaunchConfiguration("map")),
        ]
    )
