import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def _load_perception_config():
    """Perception runtime config: a per-deployment ~/.mote/perception.yaml
    overrides the packaged default (same precedence as the camera calibration)."""
    user = os.path.expanduser("~/.mote/perception.yaml")
    default = os.path.join(
        get_package_share_directory("mote_perception"), "config", "perception.yaml"
    )
    with open(user if os.path.exists(user) else default) as f:
        return yaml.safe_load(f)


def _inference_node(executable, cfg, host):
    """A perception node that reaches its inference server over TCP at `host`.

    The node runs here on the robot, in its DDS graph, next to the camera/lidar/tf
    it consumes and the Nav2 that consumes its output; only inference is off-board.
    """
    return Node(
        package="mote_perception",
        executable=executable,
        name=executable,
        remappings=[("image/compressed", "/image_raw/compressed")],
        parameters=[{"server_host": host, "server_port": cfg["server_port"]}],
        output="screen",
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    cfg = _load_perception_config()
    host = cfg["inference_host"]

    camera_monitor = Node(
        package="mote_perception",
        executable="camera_monitor",
        name="camera_monitor",
        remappings=[("image", "/image_raw")],
        output="screen",
    )

    # Depth (L1) and detection (L2) nodes attach here. Whether each runs, and where
    # its torch server lives, comes from perception.yaml — not launch args — so the
    # heavy inference can move to any machine (`pixi run inference`) without touching
    # this file. Each is torch-free and idles cheaply when its server is unreachable.
    perception_nodes = []
    if cfg["depth"]["enabled"]:
        perception_nodes.append(
            _inference_node("depth_obstacle_node", cfg["depth"], host)
        )
    if cfg["detect"]["enabled"]:
        perception_nodes.append(
            _inference_node("object_detector_node", cfg["detect"], host)
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            SetParameter(name="use_sim_time", value=use_sim_time),
            camera_monitor,
            *perception_nodes,
        ]
    )
