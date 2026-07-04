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
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EqualsSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_directory("mote_description")
    bringup_share = get_package_share_directory("mote_bringup")
    sim_share = get_package_share_directory("mote_simulation")

    with open(os.path.join(description_share, "config", "robot.yaml")) as f:
        cfg = yaml.safe_load(f)

    # gz_ros2_control runs its own controller_manager inside the gz process and
    # loads parameters from a single file referenced in the URDF, so merge
    # controllers.yaml + wheel geometry from robot.yaml + use_sim_time into one
    # temp file (same single-source-of-truth injection as mote_launch.py).
    with open(os.path.join(bringup_share, "config", "controllers.yaml")) as f:
        controller_params = yaml.safe_load(f)
    controller_params["controller_manager"]["ros__parameters"]["use_sim_time"] = True
    controller_params["diff_drive_controller"]["ros__parameters"].update(
        {
            "wheel_separation": cfg["wheel_separation"],
            "wheel_radius": cfg["wheel_radius"],
            "use_sim_time": True,
        }
    )
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
    }

    # The world is selectable so the same launch can drive the simple
    # smoke-test room (default) or a larger stress-test layout, e.g.
    #   ros2 launch mote_simulation sim_launch.py world:=office_world.sdf
    world = PathJoinSubstitution([sim_share, "worlds", LaunchConfiguration("world")])
    # gz only searches its own plugin dirs; libgz_ros2_control-system.so lives
    # in the conda env's lib dir
    gz_env = dict(os.environ)
    gz_env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = os.pathsep.join(
        filter(
            None,
            [
                os.path.join(os.environ.get("CONDA_PREFIX", ""), "lib"),
                os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
            ],
        )
    )
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
            "/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
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
        ],
        remappings=[
            ("scan", "/scan"),
            ("scan_filtered", "/scan_filtered"),
        ],
    )

    # Lidar odometry (kinematic_icp) + wheel-odom relay — the same localization
    # stack the real robot runs, so the two can't drift out of sync.
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "localization_launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # mode:=mapping|nav runs the *real* mission launch headless on top of the sim,
    # with base:=false so it skips the hardware drivers this sim stands in for.
    # The launch files themselves define what each mode means, so the sim and the
    # robot can't drift apart — this is how the mission launches get tested.
    mode = LaunchConfiguration("mode")

    def mission(launch_file, mode_value):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, "launch", launch_file)
            ),
            launch_arguments={"base": "false", "use_sim_time": "true"}.items(),
            condition=IfCondition(EqualsSubstitution(mode, mode_value)),
        )

    # Every world ships a sibling <world>.zones.yaml giving the task layer's
    # named zones (pickup/dropoff/home) coordinates valid in that world, so a
    # fetch mission runs anywhere on the world ladder. Whenever a mission mode
    # is up (Nav2 running), include the real tasks_launch.py with the matching
    # zones file — on the robot the same launch resolves zones from the active
    # site instead.
    def task_layer(context):
        if LaunchConfiguration("mode").perform(context) == "none":
            return []
        world_file = LaunchConfiguration("world").perform(context)
        zones = os.path.join(
            sim_share, "worlds", world_file.removesuffix(".sdf") + ".zones.yaml"
        )
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("mote_tasks"),
                        "launch",
                        "tasks_launch.py",
                    )
                ),
                launch_arguments={
                    "zones_file": zones,
                    "use_sim_time": "true",
                }.items(),
            )
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="mote_world.sdf",
                description="World file in mote_simulation/worlds to load",
            ),
            DeclareLaunchArgument(
                "mode",
                default_value="none",
                description="Mission to run on top of the sim: 'mapping' (SLAM + "
                "Nav2), 'nav' (Nav2 against a saved map), or 'none' (sim only)",
            ),
            SetParameter(name="use_sim_time", value=True),
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
            localization,
            mission("mapping_launch.py", "mapping"),
            mission("robot_launch.py", "nav"),
            OpaqueFunction(function=task_layer),
        ]
    )
