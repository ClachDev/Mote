"""Nav2, composed into one process.

Every Nav2 server ships as an `rclcpp_components` component, so the stack that
used to be ten processes — nine servers plus two lifecycle managers, each a full
DDS participant with its own discovery traffic, executor threads and
kernel-crossing message hops — is loaded into a single `nav2_container`
instead. Intra-container topics stay in-process. On the four-core Pi the
saving is the point; on the workstation it also frees DDS participant slots
(the localhost cap is 33 per host — see `dds_participants`).

The container is `component_container_isolated`, not `component_container` or
`_mt`: it gives every loaded component its own executor in its own thread, which
is exactly what each server had as a standalone process. The shared-executor
containers would instead serialise the servers against each other, and Nav2
makes blocking calls from inside callbacks.

Composition costs per-server crash isolation, which is why only Nav2 is composed
— the hardware drivers in `mote_launch.py` stay separate processes, since they
are the crash-prone half. What survives is whole-stack recovery: the container
respawns, and `load_components` runs again on *every* container start, so a
respawned container is refilled rather than coming back empty. That refill is an
`OpaqueFunction` returning freshly built actions because a `LoadComposableNodes`
instance, like any action, may only be executed once.
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LoadComposableNodes, Node, SetParameter
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

from mote_bringup import param_overrides, sites

CONTAINER = "nav2_container"

# (package, component plugin, node name) for each server. The node name is
# load-bearing twice: it is the key `nav2_params.yaml` is matched on — a
# composable node loaded without a name silently receives no file parameters at
# all — and it is what the lifecycle manager is told to manage.
LOCALIZATION_SERVERS = [
    ("nav2_map_server", "nav2_map_server::MapServer", "map_server"),
    ("nav2_amcl", "nav2_amcl::AmclNode", "amcl"),
]

NAVIGATION_SERVERS = [
    ("nav2_controller", "nav2_controller::ControllerServer", "controller_server"),
    ("nav2_smoother", "nav2_smoother::SmootherServer", "smoother_server"),
    ("nav2_planner", "nav2_planner::PlannerServer", "planner_server"),
    ("nav2_behaviors", "behavior_server::BehaviorServer", "behavior_server"),
    ("nav2_bt_navigator", "nav2_bt_navigator::BtNavigator", "bt_navigator"),
    (
        "nav2_waypoint_follower",
        "nav2_waypoint_follower::WaypointFollower",
        "waypoint_follower",
    ),
]

LOCALIZATION_NODES = [name for _, _, name in LOCALIZATION_SERVERS]
NAVIGATION_NODES = [name for _, _, name in NAVIGATION_SERVERS]


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

    nav2_params = param_overrides.override_path(
        "nav2",
        PathJoinSubstitution(
            [
                FindPackageShare("mote_bringup"),
                "config",
                "nav2_params.yaml",
            ]
        ),
    )

    # The WheelSpeedLimit critic's wheel_separation and max_wheel_speed come from
    # robot.yaml, not nav2_params.yaml, so the hardware envelope has one source of
    # truth. Overlaid on controller_server only (later params files win).
    with open(
        os.path.join(
            get_package_share_directory("mote_description"), "config", "robot.yaml"
        )
    ) as f:
        robot_cfg = yaml.safe_load(f)
    critic_params_file = tempfile.NamedTemporaryFile(
        mode="w", prefix="mote_wheel_critic_", suffix=".yaml", delete=False
    )
    yaml.safe_dump(
        {
            "controller_server": {
                "ros__parameters": {
                    "FollowPath": {
                        "WheelSpeedLimit.wheel_separation": robot_cfg[
                            "wheel_separation"
                        ],
                        "WheelSpeedLimit.max_wheel_speed": robot_cfg["max_wheel_speed"],
                    }
                }
            }
        },
        critic_params_file,
    )
    critic_params_file.close()

    cmd_vel_remap = ("/cmd_vel", "/diff_drive_controller/cmd_vel")

    extra_parameters = {
        "map_server": [{"yaml_filename": LaunchConfiguration("map")}],
        "controller_server": [critic_params_file.name],
    }
    extra_remappings = {
        "controller_server": [cmd_vel_remap],
        "behavior_server": [cmd_vel_remap],
    }

    def servers(table):
        return [
            ComposableNode(
                package=package,
                plugin=plugin,
                name=name,
                parameters=[nav2_params, *extra_parameters.get(name, [])],
                remappings=extra_remappings.get(name, []),
            )
            for package, plugin, name in table
        ]

    def lifecycle_manager(name, node_names):
        return ComposableNode(
            package="nav2_lifecycle_manager",
            plugin="nav2_lifecycle_manager::LifecycleManager",
            name=name,
            parameters=[
                {
                    "autostart": True,
                    "node_names": node_names,
                    "bond_timeout": 10.0,
                }
            ],
        )

    # A crashed container is respawned by the launch system, but the components
    # it held are not: loading is a service call the launch file made once, not
    # part of the container's own startup. Re-running it on every ProcessStart
    # event refills the respawned container — and the actions must be built
    # afresh each time, since an already-executed action cannot run again.
    def load_components(context, *args, **kwargs):
        return [
            LoadComposableNodes(
                target_container=f"/{CONTAINER}",
                composable_node_descriptions=[
                    *servers(LOCALIZATION_SERVERS),
                    lifecycle_manager(
                        "lifecycle_manager_localization", LOCALIZATION_NODES
                    ),
                ],
                condition=IfCondition(localisation),
            ),
            LoadComposableNodes(
                target_container=f"/{CONTAINER}",
                composable_node_descriptions=[
                    *servers(NAVIGATION_SERVERS),
                    lifecycle_manager("lifecycle_manager_navigation", NAVIGATION_NODES),
                ],
            ),
        ]

    # The params file is given to the *container* as well as to each component,
    # and it has to be. Nav2's servers create further nodes of their own —
    # /local_costmap/local_costmap, /global_costmap/global_costmap, the
    # bt_navigator's client nodes — which are not components and so are never
    # named in a load request. As separate processes they read their sections
    # out of the process's own `--params-file`; inside a container the only
    # command line they can inherit is the container's, so that is where the
    # file has to be. Without this the costmaps come up on library defaults and
    # navigation quietly gets worse rather than failing.
    container = Node(
        package="rclcpp_components",
        executable="component_container_isolated",
        name=CONTAINER,
        parameters=[nav2_params],
        output="screen",
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription(
        [
            map_arg,
            localisation_arg,
            sim_time_arg,
            SetParameter(name="use_sim_time", value=use_sim_time),
            container,
            RegisterEventHandler(
                OnProcessStart(
                    target_action=container,
                    on_start=OpaqueFunction(function=load_components),
                )
            ),
        ]
    )
