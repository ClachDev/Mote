"""Lidar odometry, composed with the wheel-odometry prior it consumes.

The prior kinematic_icp reads is the wheel pose, which diff_drive publishes as
the base *in* odom; `odom_tf_relay` broadcasts the inverse of that as a leaf
hanging off the base, so the wheel odometry reaches kinematic_icp through TF
without any node claiming odom->base twice.

kinematic_icp does not own the odom->base edge — `icp_odom_gate` does. Real
mapping bags catch the scan match emitting, in a single scan, motion the drive
cannot produce: up to 1.2 m/s against a measured 0.218 m/s limit, once moving
0.12 m while the wheels reported the robot stationary. Those frames are steps
and not spikes — the scan match carries on from the displaced pose and never
gives the displacement back — so each one is permanent error in the map frame.
A TF broadcast cannot be retracted after the fact, so kinematic_icp is left
publishing only its odometry topic, in a frame of its own (`odom_icp`), and the
gate accumulates those increments into odom->base, substituting the wheel
increment for any that the drive could not have produced. The measurements
behind the threshold are in `docs/tuning/2026-07-28-icp-velocity-gate.md`.

All three ship as `rclcpp_components`, so they are loaded into one
`localization_container` rather than run as three processes. The relay is the
original reason: it is woken at the controller's 50 Hz update rate to do twenty
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

import yaml
from ament_index_python.packages import get_package_share_directory
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

# The frame kinematic_icp accumulates in. It is deliberately not `odom`: the
# ungated track is not the odometry frame anything else may use, and naming it
# apart is what stops a consumer picking it up by accident.
ICP_ODOM_FRAME = "odom_icp"

ODOM_FRAME = "odom"
BASE_FRAME = "base_footprint"

# Slack over the drive envelope for ordinary scan-match noise. Measured across
# three mapping bags: legitimate intervals reach x1.13 of `max_wheel_speed` and
# the mildest excursion sits at x1.25, so the band this lands in is empty.
GATE_TOLERANCE = 1.15


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    with open(
        f"{get_package_share_directory('mote_description')}/config/robot.yaml"
    ) as f:
        robot = yaml.safe_load(f)

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
                "lidar_odom_frame": ICP_ODOM_FRAME,
                "wheel_odom_frame": WHEEL_ODOM_FRAME,
                "base_frame": BASE_FRAME,
                # The gate owns odom->base. kinematic_icp keeps reading its
                # prior from the wheel leaf, so muting its broadcast costs it
                # nothing: it never consumed its own output.
                "publish_odom_tf": False,
                "invert_odom_tf": False,
                "tf_timeout": 0.05,
            }
        ],
    )

    icp_gate = ComposableNode(
        package="mote_nav",
        plugin="mote_nav::IcpOdomGate",
        name="icp_odom_gate",
        parameters=[
            {
                "odom_frame": ODOM_FRAME,
                "base_frame": BASE_FRAME,
                "wheel_odom_frame": WHEEL_ODOM_FRAME,
                # The same two measurements the Nav2 wheel-speed critic bounds
                # trajectories with, so one envelope describes the hardware.
                "max_wheel_speed": float(robot["max_wheel_speed"]),
                "wheel_separation": float(robot["wheel_separation"]),
                "tolerance": GATE_TOLERANCE,
                "tf_timeout": 0.05,
            }
        ],
        remappings=[("odom_in", "/kinematic_icp/lidar_odometry")],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            SetParameter(name="use_sim_time", value=use_sim_time),
            container,
            LoadComposableNodes(
                target_container=f"/{CONTAINER}",
                composable_node_descriptions=[odom_relay, kinematic_icp, icp_gate],
            ),
        ]
    )
