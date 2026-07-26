"""The robot's off-box observability path: foxglove_bridge + the teleop relay.

Runs on the robot. An operator's Foxglove (desktop or web) connects to
`ws://<robot-id>:8765` over the tailnet -- the robot exposes nothing to the
public internet, and the bridge is what replaces joining the robot's DDS graph
from a workstation.

Bandwidth is demand-driven: the bridge advertises every topic but only
serialises the ones a connected panel has actually subscribed to, so an idle
connection costs nothing and the camera only streams while someone is looking at
it. `send_buffer_limit` bounds what a slow link can queue -- past it the bridge
drops messages rather than growing latency without limit, which is the right
failure mode for a remote view.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Teleop needs `clientPublish`; the 3D panel needs `assets` to fetch the URDF's
# package:// meshes and draw the robot. Listed in full rather than left to the
# node's default so that a change in the bridge's defaults cannot quietly remove
# the operator's ability to drive.
CAPABILITIES = [
    "clientPublish",
    "parameters",
    "parametersSubscribe",
    "services",
    "connectionGraph",
    "assets",
]


def generate_launch_description():
    bringup_share = get_package_share_directory("mote_bringup")
    use_sim_time = LaunchConfiguration("use_sim_time")

    respawn = {"respawn": True, "respawn_delay": 2.0}

    bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        parameters=[
            {
                "port": LaunchConfiguration("port"),
                "address": LaunchConfiguration("address"),
                "capabilities": CAPABILITIES,
                "use_sim_time": use_sim_time,
            }
        ],
        **respawn,
    )

    # Foxglove's Teleop panel publishes geometry_msgs/Twist and only that, while
    # DiffDriveController consumes TwistStamped -- see twist_relay's docstring.
    teleop_relay = Node(
        package="mote_bringup",
        executable="twist_relay",
        name="twist_relay",
        condition=IfCondition(LaunchConfiguration("teleop")),
        remappings=[
            ("cmd_vel_in", LaunchConfiguration("teleop_topic")),
            ("cmd_vel_out", "/diff_drive_controller/cmd_vel"),
        ],
        **respawn,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "port",
                default_value="8765",
                description="Foxglove WebSocket port on the robot.",
            ),
            DeclareLaunchArgument(
                "address",
                default_value="0.0.0.0",
                description="Bind address. All interfaces by default so the "
                "tailnet interface is covered; the tailnet, not this, is the "
                "boundary (docs/fleet/README.md).",
            ),
            DeclareLaunchArgument(
                "teleop",
                default_value="true",
                description="Run the Twist->TwistStamped relay that lets the "
                "Foxglove Teleop panel reach the drive controller.",
            ),
            DeclareLaunchArgument(
                "teleop_topic",
                default_value="/cmd_vel_teleop",
                description="Topic the Foxglove Teleop panel publishes to. It "
                "is deliberately not the controller's own topic -- see the "
                "layout in %s." % os.path.join(bringup_share, "foxglove"),
            ),
            bridge,
            teleop_relay,
        ]
    )
