"""The drive path's velocity arbiter: one writer on the controller's topic.

Nav2 and the operator both want the wheels. Before this, both published
`/diff_drive_controller/cmd_vel` directly and the controller simply took
whichever message arrived last -- so driving by hand during an active goal meant
two writers competing at 20 Hz and the robot tracking neither, and the documented
remedy was "cancel the task first", which is the wrong instruction to give
someone taking over from an autonomy run that is going wrong.

`twist_mux` (adopted, like `foxglove_bridge`, rather than built) subscribes to
one topic per source and forwards the highest-priority one that has spoken
recently, so the controller has exactly one publisher. Priorities and timeouts
are in `config/twist_mux.yaml`.

Two properties this must not break, both of which come from the node being
purely event-driven -- it publishes from an input callback and only when that
input holds priority, with no timer and no stored last command:

* when every source stops, the mux stops, so the controller's `cmd_vel_timeout`
  still halts the wheels. A mux that re-published would have turned "the link
  dropped" into "the robot keeps going";
* the message is forwarded unmodified, so the stamp the controller measures
  staleness against is still the one taken by whoever produced the command.

It runs with the base rather than with a mission, because the drive path should
not depend on which mission is up: `mote_launch.py` includes it on the robot and
`sim_launch.py` includes this same file for the simulated base.

`use_stamped` is left at twist_mux 4.5.0's default of true (the parameter is not
declared by the node, so passing it as an override would be a no-op that read
like a setting). `test_twist_mux_arbitration.py` publishes TwistStamped through a
real mux, so a future release changing that default fails a test rather than a
robot.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DRIVE_TOPIC = "/diff_drive_controller/cmd_vel"


def generate_launch_description():
    bringup_share = get_package_share_directory("mote_bringup")

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        parameters=[
            os.path.join(bringup_share, "config", "twist_mux.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=[("cmd_vel_out", DRIVE_TOPIC)],
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            twist_mux,
        ]
    )
