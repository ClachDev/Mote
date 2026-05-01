from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("auldbot_description"),
            "urdf",
            "auldbot.urdf.xacro",
        ]),
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    controller_config = PathJoinSubstitution([
        FindPackageShare("auldbot_bringup"),
        "config",
        "controllers.yaml",
    ])

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
        package="rplidar_ros",
        executable="rplidar_composition",
        name="rplidar",
        parameters=[{
            "serial_port": "/dev/ttyUSB0",
            "serial_baudrate": 460800,
            "frame_id": "lidar_link",
            "angle_compensate": True,
        }],
    )

    camera = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="camera",
        parameters=[{
            "video_device": "/dev/video0",
            "image_size": [640, 480],
            "camera_frame_id": "camera_optical_link",
        }],
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
        camera,
    ])
