import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_directory("mote_description")
    bringup_share = get_package_share_directory("mote_bringup")

    with open(os.path.join(description_share, "config", "robot.yaml")) as f:
        cfg = yaml.safe_load(f)

    urdf_file = os.path.join(description_share, "urdf", "mote.urdf.xacro")
    robot_description_content = Command(f"xacro {urdf_file}")
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    controller_config = os.path.join(bringup_share, "config", "controllers.yaml")
    # Must be a params *file* keyed by node name: a plain dict gets flattened to
    # "diff_drive_controller.ros__parameters.wheel_separation" on the
    # controller_manager node and never reaches the diff_drive_controller node.
    wheel_params_file = tempfile.NamedTemporaryFile(
        mode="w", prefix="mote_wheel_params_", suffix=".yaml", delete=False
    )
    yaml.safe_dump(
        {
            "diff_drive_controller": {
                "ros__parameters": {
                    "wheel_separation": cfg["wheel_separation"],
                    "wheel_radius": cfg["wheel_radius"],
                }
            }
        },
        wheel_params_file,
    )
    wheel_params_file.close()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config, wheel_params_file.name],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
    )

    lidar = cfg["lidar"]
    rplidar = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="rplidar",
        parameters=[{
            "serial_port": lidar["port"],
            "serial_baudrate": lidar["baud_rate"],
            "frame_id": "lidar_scan_link",
            "inverted": False,
            "angle_compensate": True,
            "scan_mode": "Standard",
        }],
    )

    laser_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[os.path.join(bringup_share, "config", "laser_filters.yaml")],
        remappings=[
            ("scan", "/scan"),
            ("scan_filtered", "/scan_filtered"),
        ],
    )

    cam = cfg["camera"]
    camera = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="camera",
        parameters=[{
            "video_device": cam["device"],
            "image_size": cam["image_size"],
            "camera_frame_id": "camera_optical_link",
        }],
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "localization_launch.py")
        )
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=controller_manager,
                on_start=[joint_state_broadcaster_spawner, diff_drive_spawner],
            )
        ),
        rplidar,
        laser_filter,
        camera,
        localization,
    ])
