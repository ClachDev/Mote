from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    kinematic_icp = Node(
        package="kinematic_icp",
        executable="kinematic_icp_online_node",
        name="online_node",
        namespace="kinematic_icp",
        output="screen",
        parameters=[{
            "lidar_topic": "/scan_filtered",
            "use_2d_lidar": True,
            "lidar_odom_frame": "odom_lidar",
            "wheel_odom_frame": "odom",
            "base_frame": "base_footprint",
            "publish_odom_tf": True,
            "invert_odom_tf": True,
            "tf_timeout": 0.05,
        }],
        remappings=[
            ("lidar_odometry", "lidar_odometry"),
        ],
    )
    return LaunchDescription([kinematic_icp])
