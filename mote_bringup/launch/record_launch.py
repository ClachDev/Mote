"""Record rosbags for perception development, split by purpose with disk caps.

Streams are defined in config/record.yaml. Each enabled stream is recorded in
parallel under ~/.mote/bags/<name>/<timestamp> with its own segment length and
rolling disk cap (bag_pruner deletes the oldest segments once a stream exceeds
its cap so continuous recording never fills the disk).

The compressed image stream is recorded rather than raw; republish it to
/image_raw on playback with image_transport.
"""

import os
from datetime import datetime

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def _load_streams():
    config_path = os.path.join(
        get_package_share_directory("mote_bringup"), "config", "record.yaml"
    )
    with open(config_path) as f:
        return yaml.safe_load(f)["streams"]


def _bags_dir(kind):
    base = os.path.join(os.path.expanduser("~/.mote/bags"), kind)
    os.makedirs(base, exist_ok=True)
    return base


def _record(kind, topics, split):
    output = os.path.join(_bags_dir(kind), datetime.now().strftime("%Y%m%d_%H%M%S"))
    return ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "--max-bag-duration",
            str(split),
            "-o",
            output,
            *topics,
        ],
        output="screen",
    )


def _pruner(kind, max_gb, interval):
    return ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "mote_bringup",
            "bag_pruner",
            "--dir",
            _bags_dir(kind),
            "--max-gb",
            str(max_gb),
            "--interval",
            str(interval),
        ],
        output="screen",
    )


def generate_launch_description():
    actions = []
    for name, stream in _load_streams().items():
        split = stream["split"]
        actions.append(_record(name, stream["topics"], split))
        actions.append(_pruner(name, stream["max_gb"], split))
    return LaunchDescription(actions)
