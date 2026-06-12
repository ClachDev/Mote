from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # true when running against the simulation (sim_launch.py)
    use_sim_time = LaunchConfiguration("use_sim_time")

    slam_params = PathJoinSubstitution([
        FindPackageShare("mote_bringup"),
        "config",
        "slam_toolbox_params.yaml",
    ])

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[slam_params, {"use_sim_time": use_sim_time}],
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
            "use_sim_time": use_sim_time,
        }],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        slam_toolbox,
        lifecycle_manager,
    ])
