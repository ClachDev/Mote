import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    hardware_config = {
        "serial_port":    "/dev/auldbot_servos",
        "baud_rate":      "1000000",
        "left_wheel_id":  "7",
        "right_wheel_id": "9",
        "velocity_scale": "674.1",
        "acceleration":   "0",
    }

    urdf_file = os.path.join(
        get_package_share_directory("auldbot_description"),
        "urdf",
        "auldbot.urdf.xacro",
    )
    xacro_args = " ".join(f"{k}:={v}" for k, v in hardware_config.items())
    robot_description_content = Command(f"xacro {urdf_file} {xacro_args}")
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    controller_config = os.path.join(
        get_package_share_directory("auldbot_bringup"),
        "config",
        "controllers.yaml",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
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

    rplidar = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="rplidar",
        parameters=[{
            "serial_port": "/dev/auldbot_lidar",
            "serial_baudrate": 460800,
            "frame_id": "lidar_scan_link",
            "inverted": False,
            "angle_compensate": True,
            "scan_mode": "Standard",
        }],
    )

    laser_filter_config = os.path.join(
        get_package_share_directory("auldbot_bringup"),
        "config",
        "laser_filters.yaml",
    )

    laser_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[laser_filter_config],
        remappings=[
            ("scan", "/scan"),
            ("scan_filtered", "/scan_filtered"),
        ],
    )

    camera = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="camera",
        parameters=[{
            "video_device": "/dev/auldbot_camera",
            "image_size": [640, 480],
            "camera_frame_id": "camera_optical_link",
        }],
    )

    launch_dir = os.path.join(
        get_package_share_directory("auldbot_bringup"), "launch"
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "localization_launch.py")
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
