"""Record rosbags for perception development, split by purpose with disk caps.

Two streams are recorded in parallel under ~/.mote/bags (per-robot, outside the
repo), each in its own kind/<timestamp> run directory:

- min/         lidar + TF/odometry only, in long 10-minute segments. The
               lightweight stream, enough to replay localisation and SLAM.
- perception/  the camera stream (compressed) plus the same lidar + TF, in short
               1-minute segments so a single clip is cheap to copy off the robot.

bag_pruner gives each stream its own rolling disk budget, deleting the oldest
segments once a stream exceeds its cap so continuous recording never fills the
disk. The compressed image stream is recorded rather than raw; republish it to
/image_raw on playback with image_transport.
"""

import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

MIN_TOPICS = ["/tf", "/tf_static", "/scan_filtered"]
PERCEPTION_TOPICS = MIN_TOPICS + ["/image_raw/compressed", "/camera_info"]


def _bags_dir(kind):
    base = os.path.join(os.path.expanduser("~/.mote/bags"), kind)
    os.makedirs(base, exist_ok=True)
    return base


def _record(kind, topics, split, condition=None):
    output = os.path.join(_bags_dir(kind), datetime.now().strftime("%Y%m%d_%H%M%S"))
    return ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "--max-bag-duration",
            split,
            "-o",
            output,
            *topics,
        ],
        output="screen",
        condition=condition,
    )


def _pruner(kind, max_gb, condition=None):
    return ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "mote_bringup",
            "bag_pruner",
            "--dir",
            _bags_dir(kind),
            "--max-gb",
            max_gb,
        ],
        output="screen",
        condition=condition,
    )


def generate_launch_description():
    perception_enabled = IfCondition(LaunchConfiguration("perception"))

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "min_split",
                default_value="600",
                description="Seconds per minimal (lidar+TF) bag segment",
            ),
            DeclareLaunchArgument(
                "perception_split",
                default_value="60",
                description="Seconds per perception (camera) bag segment",
            ),
            DeclareLaunchArgument(
                "min_max_gb",
                default_value="2.0",
                description="Rolling disk cap for the minimal stream, in GB",
            ),
            DeclareLaunchArgument(
                "perception_max_gb",
                default_value="10.0",
                description="Rolling disk cap for the perception stream, in GB",
            ),
            DeclareLaunchArgument(
                "perception",
                default_value="true",
                description="Also record the camera (perception) stream",
            ),
            _record("min", MIN_TOPICS, LaunchConfiguration("min_split")),
            _pruner("min", LaunchConfiguration("min_max_gb")),
            _record(
                "perception",
                PERCEPTION_TOPICS,
                LaunchConfiguration("perception_split"),
                condition=perception_enabled,
            ),
            _pruner(
                "perception",
                LaunchConfiguration("perception_max_gb"),
                condition=perception_enabled,
            ),
        ]
    )
