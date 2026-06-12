"""Gazebo (gz-sim) simulation bringup — workstation only, requires the 'sim'
pixi environment: pixi run sim

Mirrors mote_launch.py's structure: same controllers, same laser filter chain,
so slam/nav launch files work against the sim unmodified (pass use_sim_time).
Runs the gz server headless; add the GUI separately with 'gz sim -g' if wanted.
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_directory("mote_description")
    bringup_share = get_package_share_directory("mote_bringup")

    with open(os.path.join(description_share, "config", "robot.yaml")) as f:
        cfg = yaml.safe_load(f)

    # gz_ros2_control runs its own controller_manager inside the gz process and
    # loads parameters from a single file referenced in the URDF, so merge
    # controllers.yaml + wheel geometry from robot.yaml + use_sim_time into one
    # temp file (same single-source-of-truth injection as mote_launch.py).
    with open(os.path.join(bringup_share, "config", "controllers.yaml")) as f:
        controller_params = yaml.safe_load(f)
    controller_params["controller_manager"]["ros__parameters"]["use_sim_time"] = True
    controller_params["diff_drive_controller"]["ros__parameters"].update({
        "wheel_separation": cfg["wheel_separation"],
        "wheel_radius": cfg["wheel_radius"],
        "use_sim_time": True,
    })
    sim_controllers_file = tempfile.NamedTemporaryFile(
        mode="w", prefix="mote_sim_controllers_", suffix=".yaml", delete=False
    )
    yaml.safe_dump(controller_params, sim_controllers_file)
    sim_controllers_file.close()

    urdf_file = os.path.join(description_share, "urdf", "mote.urdf.xacro")
    robot_description_content = Command(
        f"xacro {urdf_file} use_sim:=true "
        f"sim_controllers_file:={sim_controllers_file.name}"
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str),
        "use_sim_time": True,
    }

    world = os.path.join(bringup_share, "worlds", "mote_world.sdf")
    # gz only searches its own plugin dirs; libgz_ros2_control-system.so lives
    # in the conda env's lib dir
    gz_env = dict(os.environ)
    gz_env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = os.pathsep.join(filter(None, [
        os.path.join(os.environ.get("CONDA_PREFIX", ""), "lib"),
        os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
    ]))
    gz_server = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-s", "-v", "1", world],
        env=gz_env,
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
    )

    # gz_ros2_control reads the URDF from robot_state_publisher's
    # /robot_description topic once the model is spawned
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "mote", "-z", "0.05"],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ],
        parameters=[{"use_sim_time": True}],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "60"],
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager-timeout", "60"],
    )

    laser_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[
            os.path.join(bringup_share, "config", "laser_filters.yaml"),
            {"use_sim_time": True},
        ],
        remappings=[
            ("scan", "/scan"),
            ("scan_filtered", "/scan_filtered"),
        ],
    )

    return LaunchDescription([
        gz_server,
        robot_state_publisher,
        spawn_robot,
        bridge,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[joint_state_broadcaster_spawner, diff_drive_spawner],
            )
        ),
        laser_filter,
    ])
