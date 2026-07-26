"""Start the fleet agent (`pixi run agent`).

Deliberately *not* part of the mission bringup. The agent reports on a robot and
carries commands to it; it is not something a mission depends on, and folding it
into `mote_launch.py` would mean a robot that cannot reach the fleet server
takes its bringup with it. It runs as its own service (`mote-agent.service`)
alongside the mission, exactly like the health monitor.

    ros2 launch mote_fleet agent_launch.py
    ros2 launch mote_fleet agent_launch.py broker_host:=fleet-box   # override

With no arguments the agent reads its identity and its broker from `~/.mote`
(written by `enroll`), which is the normal path: a robot that has enrolled needs
no launch configuration at all.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    broker_host = LaunchConfiguration("broker_host")
    broker_port = LaunchConfiguration("broker_port")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "broker_host",
                default_value="",
                description="override the broker from ~/.mote/fleet.yaml",
            ),
            DeclareLaunchArgument(
                "broker_port",
                default_value="0",
                description="0 means 'whatever fleet.yaml says'",
            ),
            SetParameter(name="use_sim_time", value=use_sim_time),
            Node(
                package="mote_fleet",
                executable="agent",
                name="mote_agent",
                output="screen",
                # Per-node recovery under the whole-service systemd restart, the
                # same posture as the driver and nav2 nodes: a crashed agent
                # must not need a human to bring the robot back onto the fleet.
                respawn=True,
                respawn_delay=5.0,
                parameters=[
                    {
                        "broker_host": ParameterValue(broker_host, value_type=str),
                        "broker_port": ParameterValue(broker_port, value_type=int),
                    }
                ],
            ),
        ]
    )
