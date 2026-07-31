"""Command-line plumbing shared by the arm tools: arguments, and clean exits.

``ros2 run`` hands the executable everything on the line, ROS's own arguments
included, so a node with its own flags sees both. The usual answer,
``parse_known_args``, silently discards whatever it does not recognise — which
means a mistyped flag on a *safety* option (``--max-travel``, ``--speed-scale``)
does nothing at all and says nothing about it.

So instead: cut the ``--ros-args ... --`` block out first, then parse what is
left strictly. A flag we do not know is then an error, as it should be.
"""

from __future__ import annotations

import sys
import threading
from collections.abc import Sequence

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def user_args(argv: Sequence[str] | None = None) -> list[str]:
    """Everything on the command line that is not a ROS argument.

    ``--ros-args`` opens the ROS block and a bare ``--`` closes it; that bare
    separator is also what argparse would otherwise take as "no more options",
    turning ``-- --camera`` into a positional it cannot place.
    """
    argv = list(sys.argv[1:] if argv is None else argv)
    kept: list[str] = []
    in_ros_block = False
    for arg in argv:
        if arg == "--ros-args":
            in_ros_block = True
        elif arg == "--":
            in_ros_block = False
        elif not in_ros_block:
            kept.append(arg)
    return kept


def parse(parser, argv: Sequence[str] | None = None):
    """Parse this tool's own arguments, strictly, ignoring ROS's."""
    return parser.parse_args(user_args(argv))


def spin_background(node: Node) -> threading.Thread:
    """Spin a node on a worker thread while the main thread runs a CLI."""

    def _spin() -> None:
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        except Exception:  # noqa: BLE001
            # SIGINT tears the rcl context down underneath spin(), which
            # surfaces as different exception types depending on how the node
            # was launched. "The context is gone" is the ordinary stop it is;
            # anything that failed while it was still up is a real error.
            if rclpy.ok():
                raise

    thread = threading.Thread(target=_spin, daemon=True)
    thread.start()
    return thread


def shutdown(node: Node, spinner: threading.Thread) -> None:
    """Stop spinning, then destroy the node — in that order.

    Destroying a node that ``spin()`` still holds pulls it out from under the
    executor mid-callback, and the process aborts with "terminate called
    without an active exception" instead of exiting. Shutting the context down
    first makes spin return; joining it is what guarantees it has.
    """
    if rclpy.ok():
        rclpy.shutdown()
    spinner.join(timeout=2.0)
    node.destroy_node()
