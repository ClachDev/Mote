from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Wheel odom relayed as an inverted leaf (base_footprint -> odom_wheel) so
    # kinematic_icp can own the odom->base edge while still reading the wheel
    # odometry as its motion prior.
    odom_relay = Node(
        package="mote_bringup",
        executable="odom_tf_relay",
        parameters=[{"child_frame": "odom_wheel"}],
        remappings=[("odom_in", "/diff_drive_controller/odom")],
    )

    # Lidar odometry (wheel prior fused in) publishes odom->base, which slam and
    # nav consume instead of raw wheel odom.
    kinematic_icp = Node(
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
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            SetParameter(name="use_sim_time", value=use_sim_time),
            odom_relay,
            kinematic_icp,
        ]
    )
