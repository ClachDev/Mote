"""Lidar odometry, composed with the wheel-odometry prior it consumes.

kinematic_icp owns the odom->base edge. The prior it reads is the wheel pose,
which diff_drive publishes as the base *in* odom; `odom_tf_relay` broadcasts the
inverse of that as a leaf hanging off the base, so the wheel odometry reaches
kinematic_icp through TF without either node claiming odom->base twice.

Both ship as `rclcpp_components`, so they are loaded into one
`localization_container` rather than run as two processes. The relay is the
reason: it is woken at the controller's 50 Hz update rate to do twenty
floating-point operations, and as a standalone Python node the interpreter
wake-up, the GIL and the kernel-crossing message hop all cost more than the
arithmetic did. Its one consumer is in the same container.

The container is `component_container_isolated`, matching `nav2_launch.py`: each
component keeps its own executor in its own thread, which is what each had as a
process of its own. Intra-process communication is deliberately left off — the
relay's input comes from `ros2_control_node` and kinematic_icp's scan comes from
the laser filter, both outside this container, so there is nothing here that a
zero-copy path would shorten.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node, SetParameter
from launch_ros.descriptions import ComposableNode

CONTAINER = "localization_container"

# The leaf the relay writes and kinematic_icp reads. One name, because the two
# halves only work if they agree on it, and a disagreement costs kinematic_icp
# its motion prior without failing anything loudly.
WHEEL_ODOM_FRAME = "odom_wheel"


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    container = Node(
        package="rclcpp_components",
        executable="component_container_isolated",
        name=CONTAINER,
        output="screen",
    )

    odom_relay = ComposableNode(
        package="mote_nav",
        plugin="mote_nav::OdomTfRelay",
        name="odom_tf_relay",
        parameters=[{"child_frame": WHEEL_ODOM_FRAME}],
        remappings=[("odom_in", "/diff_drive_controller/odom")],
    )

    kinematic_icp = ComposableNode(
        package="kinematic_icp",
        plugin="kinematic_icp_ros::OnlineNode",
        name="online_node",
        namespace="kinematic_icp",
        parameters=[
            {
                "lidar_topic": "/scan_filtered",
                "use_2d_lidar": True,
                "lidar_odom_frame": "odom",
                "wheel_odom_frame": WHEEL_ODOM_FRAME,
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
            container,
            LoadComposableNodes(
                target_container=f"/{CONTAINER}",
                composable_node_descriptions=[odom_relay, kinematic_icp],
            ),
        ]
    )
