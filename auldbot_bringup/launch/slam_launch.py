from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    slam_params = PathJoinSubstitution([
        FindPackageShare("auldbot_bringup"),
        "config",
        "slam_toolbox_params.yaml",
    ])

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[slam_params],
        output="screen",
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        parameters=[{
            "autostart": True,
            "node_names": ["slam_toolbox"],
            "bond_timeout": 0.0,
        }],
        output="screen",
    )

    return LaunchDescription([slam_toolbox, lifecycle_manager])
