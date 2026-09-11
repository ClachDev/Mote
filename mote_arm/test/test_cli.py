"""The shared CLI plumbing: strict argument parsing, and exits that are exits.

Both properties fail *silently* if they regress, which is why they are pinned
here. A mistyped safety flag that argparse drops on the floor changes nothing
and says nothing; and a node destroyed while ``spin()`` still holds it aborts
the process after the useful work is done, so the operator sees a crash on a
run that in fact succeeded.
"""

import os
import random
import subprocess
import sys
import textwrap

import pytest

from mote_arm import cli
from mote_arm.arm_pose import build_parser


def test_user_args_drops_the_ros_block():
    assert cli.user_args(["go", "home", "--ros-args", "-p", "x:=1"]) == ["go", "home"]


def test_bare_separator_closes_the_ros_block():
    """``--`` ends the ROS block; what follows is ours again.

    argparse would read that bare ``--`` as "no more options" and turn a flag
    after it into a positional it cannot place, so it must not survive.
    """
    argv = ["go", "home", "--ros-args", "-p", "x:=1", "--", "--max-lag", "0.2"]
    assert cli.user_args(argv) == ["go", "home", "--max-lag", "0.2"]


def test_no_ros_block_is_left_alone():
    assert cli.user_args(["go", "home", "--max-lag", "0.2"]) == [
        "go",
        "home",
        "--max-lag",
        "0.2",
    ]


def test_parse_accepts_a_safety_flag_past_a_ros_block():
    args = cli.parse(
        build_parser(),
        ["go", "home", "--max-lag", "1.25", "--ros-args", "-p", "x:=1"],
    )
    assert args.name == "home"
    assert args.max_lag == 1.25


@pytest.mark.parametrize("flag", ["--max_lag", "--maxlag", "--speeed"])
def test_a_mistyped_safety_flag_is_an_error(flag):
    """Not a warning, and above all not silence: the arm would move anyway."""
    with pytest.raises(SystemExit) as exc:
        cli.parse(build_parser(), ["go", "home", flag, "0.9"])
    assert exc.value.code != 0


# Destroying a node the executor still holds aborts the interpreter itself
# (SIGABRT, "terminate called without an active exception"), so no in-process
# assertion can catch it — the test has to watch a child's exit status.
CHILD = """\
import threading
import rclpy
from rclpy.node import Node
from mote_arm import cli

rclpy.init()
node = Node("cli_shutdown_probe")
node.create_timer(0.001, lambda: None)
spinner = cli.spin_background(node)
threading.Event().wait(0.4)
cli.shutdown(node, spinner)
print("clean")
"""


def test_shutdown_exits_cleanly():
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(random.randint(60, 100))
    env["PYTHONPATH"] = os.pathsep.join(sys.path)
    done = subprocess.run(
        [sys.executable, "-c", textwrap.dedent(CHILD)],
        capture_output=True,
        text=True,
        timeout=60,
        env=env,
    )
    assert done.returncode == 0, (
        f"exit {done.returncode} (-6/134 is the abort this guards against)\n"
        f"{done.stderr}"
    )
    assert "clean" in done.stdout
    assert "terminate called" not in done.stderr
