import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue

from mote_bringup import mote_home, param_overrides
from mote_bringup.launch_utils import (
    ICP_ODOM_FRAME,
    INACTIVE_CONTROLLERS,
    arm_config_file,
    controller_spawn_handler,
    joint_params_file,
    resolved_arm,
)


def generate_launch_description():
    description_share = get_package_share_directory("mote_description")
    bringup_share = get_package_share_directory("mote_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")

    with open(os.path.join(description_share, "config", "robot.yaml")) as f:
        cfg = yaml.safe_load(f)

    # The arm's zero and limits are per-robot measurements living outside the
    # package, so they are resolved here and handed to xacro — see resolved_arm.
    arm = resolved_arm(cfg)

    urdf_file = os.path.join(description_share, "urdf", "mote.urdf.xacro")
    xacro_args = "" if arm is None else f" arm_config:={arm_config_file(arm)}"
    robot_description_content = Command(f"xacro {urdf_file}{xacro_args}")
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    controller_config = param_overrides.override_path(
        "controllers", os.path.join(bringup_share, "config", "controllers.yaml")
    )
    controller_joint_params = joint_params_file(cfg, arm)

    # respawn=True gives per-node recovery: if a driver process crashes, the
    # launch system relaunches it within respawn_delay. This is the inner layer;
    # systemd restarts the whole service only if `pixi run launch` itself dies.
    # The controllers are re-spawned on each controller_manager start by
    # controller_spawn_handler (see launch_utils for why that indirection).
    respawn = {"respawn": True, "respawn_delay": 2.0}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        **respawn,
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config, controller_joint_params],
        **respawn,
    )

    lidar = cfg["lidar"]
    rplidar = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="rplidar",
        parameters=[
            {
                "serial_port": lidar["port"],
                "serial_baudrate": lidar["baud_rate"],
                "frame_id": "lidar_scan_link",
                "inverted": False,
                "angle_compensate": True,
                "scan_mode": "Standard",
            }
        ],
        **respawn,
    )

    laser_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[os.path.join(bringup_share, "config", "laser_filters.yaml")],
        remappings=[
            ("scan", "/scan"),
            ("scan_filtered", "/scan_filtered"),
        ],
        **respawn,
    )

    cam = cfg["camera"]
    camera_params = {
        "video_device": cam["device"],
        "image_size": cam["image_size"],
        "camera_frame_id": "camera_optical_link",
        # bgr8 so the compressed stream carries a correct encoding label:
        # compressed_image_transport can't round-trip yuv422_yuy2 (it decodes to
        # bgr8 but keeps the yuv422 label), which breaks any consumer that trusts
        # the label, e.g. the RViz Camera display.
        "output_encoding": "bgr8",
    }
    # A per-robot calibration in MOTE_HOME overrides the packaged default fallback.
    user_calibration = mote_home.path("camera_calibration.yaml")
    if user_calibration.exists():
        camera_params["camera_info_url"] = f"file://{user_calibration}"
    elif "default_info_url" in cam:
        camera_params["camera_info_url"] = cam["default_info_url"]
    camera = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="camera",
        parameters=[camera_params],
        **respawn,
    )

    system_monitor = Node(
        package="mote_bringup",
        executable="system_monitor",
        **respawn,
    )

    # Reads the wheel-vs-lidar odometry disagreement and reports slip / stuck /
    # scan-match excursions on /diagnostics, which health_monitor folds into the
    # robot summary. It runs with the base rather than with a mission because
    # both of its inputs are the base's: the controller's odometry and
    # localization_launch.py's lidar odometry.
    #
    # It reads the *ungated* lidar track, not odom->base. Its `icp_fault`
    # verdict fires on a body speed the drive cannot produce, and icp_odom_gate
    # exists to keep exactly that out of odom->base -- so pointed at the gated
    # edge the check could never fire again, and a scan match degrading behind a
    # working gate would go unreported.
    slip_monitor = Node(
        package="mote_bringup",
        executable="slip_monitor",
        parameters=[{"odom_frame": ICP_ODOM_FRAME}],
        **respawn,
    )

    # The health monitor runs with the base by default, so *any* way of starting
    # the robot — `pixi run launch`/`robot`/`mapping` on a desk as much as the
    # systemd path — publishes /health and /diagnostics_agg. mote-bringup.service
    # passes health:=false because mote-health.service runs it separately there,
    # with a watchdog and its own lifecycle (it must outlive a bringup restart to
    # report one); two copies would both publish /health.
    health_monitor = Node(
        package="mote_bringup",
        executable="health_monitor",
        condition=IfCondition(LaunchConfiguration("health")),
        **respawn,
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "localization_launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # The controller's single publisher. It belongs to the base rather than to a
    # mission because the drive path must not depend on which mission is up —
    # `pixi run teleop` against a bare base goes through it too.
    twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "twist_mux_launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # The same arrangement as the health monitor, for the same reason: every way
    # of starting the robot gives an operator a way to watch it. Under systemd
    # mote-bringup.service passes foxglove:=false and mote-foxglove.service runs
    # the bridge instead, because it must outlive a bringup restart — a
    # crash-looping mission is exactly when someone needs to look at the robot.
    foxglove = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "foxglove_launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
        condition=IfCondition(LaunchConfiguration("foxglove")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "health",
                default_value="true",
                description="Run the health monitor alongside the base. Set "
                "false when mote-health.service already runs it.",
            ),
            DeclareLaunchArgument(
                "foxglove",
                default_value="true",
                description="Run foxglove_bridge alongside the base. Set false "
                "when mote-foxglove.service already runs it.",
            ),
            SetParameter(name="use_sim_time", value=use_sim_time),
            robot_state_publisher,
            controller_manager,
            controller_spawn_handler(
                controller_manager,
                inactive=INACTIVE_CONTROLLERS if arm is not None else (),
            ),
            rplidar,
            laser_filter,
            camera,
            system_monitor,
            slip_monitor,
            health_monitor,
            localization,
            twist_mux,
            foxglove,
        ]
    )
