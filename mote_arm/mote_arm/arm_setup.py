"""Everything that configures the arm's servos, behind one command.

    pixi run arm-setup check                     # read-only: what is on the bus
    pixi run arm-setup calibrate                 # the once-off, needs a human
    pixi run arm-setup gains show|apply|sweep
    pixi run arm-setup offsets show|backup|restore|set
    pixi run arm-setup limits show|clear|restore

These five were five commands, and being five hid what they have in common:
each opens `/dev/mote_servos` directly, so the control stack has to be stopped
first (`pixi run kill`), and each writes servo EEPROM, which is per-robot
hardware state with no copy in the repo. Four of them carried a byte-identical
copy of the port guard.

They are also, `check` aside, **once-off**: run when the arm is built, a servo
is swapped, or something is wrong. Nothing here belongs in a session that is
trying to move the arm — for that see `arm-teleop`, `arm-pose` and `arm-record`,
which are clients of `arm_controller` and never touch the bus.

The register each one owns:

    calibrate   the position-correction offsets *and* the goal-range fence,
                written together because a fence outlives the frame it was
                measured in
    gains       the position-loop kp/kd/ki
    offsets     the position-correction offsets, on their own, for recovery
    limits      the goal-range fence, on its own, for diagnosis
"""

from __future__ import annotations

import argparse

from mote_arm import (
    arm_calibrate,
    arm_check,
    arm_gains,
    arm_limits,
    arm_offsets,
    cli,
    config,
)
from mote_arm.bus import open_bus

TOOLS = (arm_check, arm_calibrate, arm_gains, arm_offsets, arm_limits)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="arm-setup",
        description="Configure the arm's servos. Run with the base stopped.",
    )
    parser.add_argument(
        "--robot-yaml", default="", help="override the packaged robot.yaml"
    )
    parser.add_argument(
        "--yes", action="store_true", help="skip confirmations before EEPROM writes"
    )
    sub = parser.add_subparsers(dest="tool", required=True)
    for tool in TOOLS:
        tool.add_subparser(sub)
    return parser


def main() -> None:
    args = cli.parse(build_parser())
    cfg = (
        config.ArmConfig.from_yaml_file(args.robot_yaml)
        if args.robot_yaml
        else config.load()
    )
    bus = open_bus(cfg)
    try:
        args.func(cfg, bus, args)
    finally:
        bus.close()


if __name__ == "__main__":
    main()
