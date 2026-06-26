from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    camera_monitor = Node(
        package="mote_perception",
        executable="camera_monitor",
        name="camera_monitor",
        remappings=[("image", "/image_raw")],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            SetParameter(name="use_sim_time", value=use_sim_time),
            camera_monitor,
            # Perception extension point: rectify, depth, and detection nodes
            # (L1) attach here, downstream of the camera and its calibration.
        ]
    )
