"""One command for everything that configures the servos, and one port guard.

The five tools behind `arm-setup` were five commands, which hid what they have
in common: each opens `/dev/mote_servos` directly, so the control stack must be
stopped first, and each writes servo EEPROM. Four carried a byte-identical copy
of the port guard, which is three chances for one of them to grow a different
idea of what "the base is running" means.

What is pinned here is the dispatch table — that every subcommand still reaches
the function it used to — and that the shared flags really are shared.
"""

import pytest

from mote_arm import arm_setup, cli

CASES = [
    (["check"], "arm_check", "run"),
    (["check", "--save-zero"], "arm_check", "run"),
    (["calibrate"], "arm_calibrate", "run"),
    (["calibrate", "--skip-homing"], "arm_calibrate", "run"),
    (["calibrate", "--joints", "wrist_roll"], "arm_calibrate", "run"),
    (["gains", "show"], "arm_gains", "_cmd_show"),
    (["gains", "apply"], "arm_gains", "_cmd_apply"),
    (["gains", "sweep", "--joint", "elbow_flex"], "arm_gains", "_cmd_sweep"),
    (["offsets", "show"], "arm_offsets", "_cmd_show"),
    (["offsets", "backup"], "arm_offsets", "_cmd_backup"),
    (["offsets", "restore"], "arm_offsets", "_cmd_restore"),
    (
        ["offsets", "set", "--joint", "gripper", "--value", "12"],
        "arm_offsets",
        "_cmd_set",
    ),
    (["limits", "show"], "arm_limits", "_cmd_show"),
    (["limits", "clear"], "arm_limits", "_cmd_clear"),
    (["limits", "restore"], "arm_limits", "_cmd_restore"),
]


@pytest.mark.parametrize("argv,module,func", CASES)
def test_every_subcommand_reaches_its_own_handler(argv, module, func):
    args = arm_setup.build_parser().parse_args(argv)
    assert args.func.__module__.rsplit(".", 1)[-1] == module
    assert args.func.__name__ == func


def test_a_bare_command_is_refused_rather_than_doing_something():
    with pytest.raises(SystemExit):
        arm_setup.build_parser().parse_args([])


@pytest.mark.parametrize("tool", ["gains", "offsets", "limits"])
def test_a_group_with_no_action_is_refused(tool):
    with pytest.raises(SystemExit):
        arm_setup.build_parser().parse_args([tool])


def test_the_confirmation_skip_is_one_flag_for_every_tool():
    """It was on three of the five, in two different places."""
    for argv in (["calibrate"], ["gains", "apply"], ["offsets", "restore"]):
        assert arm_setup.build_parser().parse_args(["--yes", *argv]).yes is True
    assert arm_setup.build_parser().parse_args(["check"]).yes is False


def test_ros_arguments_are_cut_out_before_parsing():
    """`ros2 run` hands the tool ROS's arguments too; none of the five coped."""
    args = cli.parse(
        arm_setup.build_parser(),
        ["limits", "clear", "--joint", "gripper", "--ros-args", "-p", "x:=1"],
    )
    assert args.joint == "gripper"


def test_a_mistyped_flag_is_an_error_rather_than_a_default():
    """Silently dropping it would run the write with a value nobody chose."""
    with pytest.raises(SystemExit) as exc:
        cli.parse(arm_setup.build_parser(), ["offsets", "set", "--jiont", "gripper"])
    assert exc.value.code != 0
