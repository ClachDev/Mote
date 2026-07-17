"""Record rosbags, split by purpose with disk caps.

Streams are defined in config/record.yaml. Each selected stream is recorded in
parallel under ~/.mote/bags/<name>/<timestamp> (MOTE_HOME overrides ~/.mote)
with its own segment length and rolling disk cap (bag_pruner deletes the
oldest segments once a stream exceeds its cap so continuous recording never
fills the disk). A ``streams`` arg selects a comma-separated subset (default:
all).

The compressed image stream is recorded rather than raw; republish it to
/image_raw on playback with image_transport.
"""

import os
from datetime import datetime

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
)

from mote_bringup import sites


def _load_streams():
    config_path = os.path.join(
        get_package_share_directory("mote_bringup"), "config", "record.yaml"
    )
    with open(config_path) as f:
        return yaml.safe_load(f)["streams"]


def _bags_dir(kind):
    base = sites.bags_dir(kind)
    base.mkdir(parents=True, exist_ok=True)
    return str(base)


def _record(kind, topics, split, output):
    return ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "--max-bag-duration",
            str(split),
            # Action topics (e.g. /navigate_to_pose/_action/status) are hidden;
            # without this flag the recorder silently skips them even when they
            # are listed explicitly. Recording stays limited to the listed
            # topics — the flag only lets hidden ones through that filter.
            "--include-hidden-topics",
            "-o",
            output,
            "--topics",
            *topics,
        ],
        output="screen",
    )


def _pruner(kind, max_gb, interval):
    return ExecuteProcess(
        # Invoked directly rather than through `ros2 run`, whose wrapper does
        # not forward launch's shutdown signals to its child — the pruner
        # would outlive every session as an orphan.
        cmd=[
            "python3",
            "-m",
            "mote_bringup.bag_pruner",
            "--dir",
            _bags_dir(kind),
            "--max-gb",
            str(max_gb),
            "--interval",
            str(interval),
        ],
        output="screen",
        # The pruner is the only thing standing between continuous recording
        # and a full disk, so restart it if it ever dies.
        respawn=True,
        respawn_delay=5.0,
    )


def _stream_actions(context):
    streams = _load_streams()
    selected = [s for s in context.launch_configurations["streams"].split(",") if s]
    unknown = set(selected) - set(streams)
    if unknown:
        raise ValueError(
            f"unknown record stream(s) {sorted(unknown)}; "
            f"record.yaml defines {sorted(streams)}"
        )
    active = [name for name in streams if not selected or name in selected]
    actions = [
        LogInfo(
            msg=f"[record_launch] recording {len(active)} stream(s): "
            f"{', '.join(active)}"
        )
    ]
    for name in active:
        stream = streams[name]
        split = stream["split"]
        output = os.path.join(_bags_dir(name), datetime.now().strftime("%Y%m%d_%H%M%S"))
        actions.append(
            LogInfo(
                msg=f"[record_launch] stream '{name}' -> {output} "
                f"({len(stream['topics'])} topics: {', '.join(stream['topics'])})"
            )
        )
        actions.append(_record(name, stream["topics"], split, output))
        actions.append(_pruner(name, stream["max_gb"], split))
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "streams",
                default_value="",
                description="Comma-separated subset of record.yaml streams to "
                "record (empty = all)",
            ),
            OpaqueFunction(function=_stream_actions),
        ]
    )
