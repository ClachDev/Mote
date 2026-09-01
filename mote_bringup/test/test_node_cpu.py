"""node_cpu identifies the node a process *is*, and never one of its wrappers.

This is the instrument every monitor-CPU decision rests on, and both of its
failure modes are quiet: matching a wrapper reports a node that does no work as
if it were the node, and matching nothing reports "not running" on a stack that
is up. Neither raises.
"""

import sys
from pathlib import Path

# tools/ is not a package — the sampler has to run on the Pi from a checkout, so
# it is a script rather than an installed entry point.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from node_cpu import node_instance  # noqa: E402

PY = "/home/michael/Mote/.pixi/envs/default/bin/python3.12"
INSTALL = "/home/michael/Mote/install"


def test_python_entry_point_is_named_by_its_script():
    argv = [PY, f"{INSTALL}/mote_bringup/lib/mote_bringup/slip_monitor"]
    assert node_instance(argv) == "slip_monitor"


def test_compiled_executable_is_named_by_itself():
    """A C++ node runs no interpreter, so argv[1] says nothing about it."""
    argv = [f"{INSTALL}/mote_health/lib/mote_health/health_monitor"]
    assert node_instance(argv) == "health_monitor"


def test_a_rename_wins_for_either_language():
    """Two builds of one node are told apart by `__node:=`, and only by it."""
    py = [
        PY,
        f"{INSTALL}/mote_bringup/lib/mote_bringup/health_monitor",
        "--ros-args",
        "-r",
        "__node:=health_monitor_b",
    ]
    cpp = [
        f"{INSTALL}/mote_health/lib/mote_health/health_monitor",
        "--ros-args",
        "-r",
        "__node:=health_monitor_b",
    ]
    assert node_instance(py) == "health_monitor_b"
    assert node_instance(cpp) == "health_monitor_b"


def test_the_ros2_run_wrapper_is_not_a_node():
    """`ros2 run` repeats the node's whole command line in its own."""
    argv = [
        PY,
        f"{INSTALL}/../.pixi/envs/default/bin/ros2",
        "run",
        "mote_health",
        "health_monitor",
    ]
    assert node_instance(argv) is None


def test_the_pixi_wrapper_is_not_a_node():
    argv = ["/home/michael/.pixi/bin/pixi", "run", "health"]
    assert node_instance(argv) is None


def test_a_binary_outside_an_install_tree_is_not_a_node():
    """The compiled case must not turn every system process into a node."""
    assert node_instance(["/usr/bin/gz", "sim", "-s"]) is None
    assert node_instance(["/home/michael/Mote/build/mote_health/test_config"]) is None
