"""Record a rosbag of the topics needed to develop and validate perception.

Bags are split into fixed-duration segments (default 10 minutes) and written to
~/.mote/bags (per-robot, outside the repo), one timestamped directory per run.
The compressed image stream is recorded rather than raw to keep bag size sane;
republish it to /image_raw on playback with image_transport.
"""

import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

TOPICS = [
    "/image_raw/compressed",
    "/camera_info",
    "/tf",
    "/tf_static",
    "/scan_filtered",
]


def generate_launch_description():
    bags_dir = os.path.expanduser("~/.mote/bags")
    os.makedirs(bags_dir, exist_ok=True)
    output = os.path.join(bags_dir, datetime.now().strftime("%Y%m%d_%H%M%S"))

    record = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "--max-bag-duration",
            LaunchConfiguration("split"),
            "-o",
            output,
            *TOPICS,
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "split",
                default_value="600",
                description="Seconds per bag segment (rosbag2 --max-bag-duration)",
            ),
            record,
        ]
    )
