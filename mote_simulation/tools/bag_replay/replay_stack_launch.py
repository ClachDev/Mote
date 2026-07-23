"""Launch the SLAM/ICP stack under test for offline bag replay.

Deliberately self-contained — it references only environment packages
(slam_toolbox, nav2_lifecycle_manager, kinematic_icp), never the mote_bringup
share, so it runs against a bag without the workspace being built and can be
launched by absolute path:

    ros2 launch <this file> mode:=slam slam_params_file:=<paramset.yaml>

It mirrors the robot's real slam_launch.py / localization_launch.py node setup,
but takes ``slam_params_file`` as an argument (the real launch hardcodes it) so a
parameter set can be swapped per replay. ``use_sim_time`` defaults true because
the replayer drives ``/clock`` from bag timestamps. The bag already carries the
``base_footprint->odom_wheel`` wheel-odom edge, so ICP needs no odom relay here.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    mode = LaunchConfiguration("mode")
    slam_params_file = LaunchConfiguration("slam_params_file")

    is_slam = IfCondition(PythonExpression(["'", mode, "' == 'slam'"]))
    is_icp = IfCondition(PythonExpression(["'", mode, "' == 'icp'"]))

    slam = GroupAction(
        condition=is_slam,
        actions=[
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                parameters=[slam_params_file],
                output="screen",
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_slam",
                parameters=[
                    {
                        "autostart": True,
                        "node_names": ["slam_toolbox"],
                        "bond_timeout": 0.0,
                    }
                ],
                output="screen",
            ),
        ],
    )

    icp = Node(
        condition=is_icp,
        package="kinematic_icp",
        executable="kinematic_icp_online_node",
        name="online_node",
        namespace="kinematic_icp",
        output="screen",
        parameters=[
            {
                "lidar_topic": "/scan_filtered",
                "use_2d_lidar": True,
                "lidar_odom_frame": "odom",
                "wheel_odom_frame": "odom_wheel",
                "base_frame": "base_footprint",
                "publish_odom_tf": True,
                "invert_odom_tf": False,
                "tf_timeout": 0.05,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("mode", default_value="slam"),
            DeclareLaunchArgument("slam_params_file", default_value=""),
            SetParameter(name="use_sim_time", value=use_sim_time),
            slam,
            icp,
        ]
    )
