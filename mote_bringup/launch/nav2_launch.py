from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

from mote_bringup import sites

LOCALIZATION_NODES = [
    "map_server",
    "amcl",
]

NAVIGATION_NODES = [
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
]


def generate_launch_description():
    map_arg = DeclareLaunchArgument(
        "map",
        default_value=sites.resolve_map(),
        description="Full path to the map yaml file to load; defaults to the "
        "active site's floor map (ignored when localisation:=false)",
    )

    localisation_arg = DeclareLaunchArgument(
        "localisation",
        default_value="true",
        description="Run map_server and amcl to localise against a saved map. "
        "Set false to navigate against a live slam_toolbox map, which "
        "provides the map and map->odom itself.",
    )

    sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use the /clock topic (set true when running against the sim)",
    )

    localisation = LaunchConfiguration("localisation")
    use_sim_time = LaunchConfiguration("use_sim_time")

    nav2_params = PathJoinSubstitution(
        [
            FindPackageShare("mote_bringup"),
            "config",
            "nav2_params.yaml",
        ]
    )

    cmd_vel_remap = ("/cmd_vel", "/diff_drive_controller/cmd_vel")

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        parameters=[nav2_params, {"yaml_filename": LaunchConfiguration("map")}],
        condition=IfCondition(localisation),
        output="screen",
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        parameters=[nav2_params],
        condition=IfCondition(localisation),
        output="screen",
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        parameters=[nav2_params],
        remappings=[cmd_vel_remap],
        output="screen",
    )

    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        parameters=[nav2_params],
        output="screen",
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        parameters=[nav2_params],
        output="screen",
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        parameters=[nav2_params],
        remappings=[cmd_vel_remap],
        output="screen",
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        parameters=[nav2_params],
        output="screen",
    )

    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        parameters=[nav2_params],
        output="screen",
    )

    lifecycle_manager_localization = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        parameters=[
            {
                "autostart": True,
                "node_names": LOCALIZATION_NODES,
                "bond_timeout": 10.0,
            }
        ],
        condition=IfCondition(localisation),
        output="screen",
    )

    lifecycle_manager_navigation = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        parameters=[
            {
                "autostart": True,
                "node_names": NAVIGATION_NODES,
                "bond_timeout": 10.0,
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            map_arg,
            localisation_arg,
            sim_time_arg,
            SetParameter(name="use_sim_time", value=use_sim_time),
            map_server,
            amcl,
            controller_server,
            smoother_server,
            planner_server,
            behavior_server,
            bt_navigator,
            waypoint_follower,
            lifecycle_manager_localization,
            lifecycle_manager_navigation,
        ]
    )
