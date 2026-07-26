"""Launch helpers that need to be unit-testable outside a launch run.

Launch files live under ``launch/`` and are loaded by path, not imported, so
logic worth testing lives here in the package instead.
"""

from launch.actions import OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

CONTROLLERS = ("joint_state_broadcaster", "diff_drive_controller")


def spawn_controllers(_context=None, *_args, **_kwargs):
    """Fresh ``spawner`` actions for every controller, built per call.

    Must return NEW action objects each time: a launch action may only be
    executed once, and this runs again on every controller_manager restart.
    """
    return [
        Node(package="controller_manager", executable="spawner", arguments=[name])
        for name in CONTROLLERS
    ]


def controller_spawn_handler(controller_manager):
    """Spawn the controllers whenever ``controller_manager`` starts.

    Wrapping the spawners in an OpaqueFunction is what makes a
    ``respawn=True`` controller_manager survivable: registering pre-built Node
    actions instead raises "executed more than once" on the second start, and
    that exception aborts the entire launch, taking every other node with it.
    Re-spawning is also required for recovery to mean anything — a restarted
    controller_manager comes back with no controllers loaded.
    """
    return RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[OpaqueFunction(function=spawn_controllers)],
        )
    )
