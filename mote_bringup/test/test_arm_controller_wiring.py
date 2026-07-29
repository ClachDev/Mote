"""The arm's place in the control stack, as the launch layer sets it up.

Three things have to stay true or the arm is unsafe rather than merely broken:
it is spawned *inactive* (activation is what enables servo torque); the joint
list handed to `arm_controller` comes from robot.yaml rather than being written
out a second time in `controllers.yaml`; and the zero/limits that reach the
URDF — and so the clamp MoteHardware enforces — are this robot's *calibrated*
ones, not the packaged placeholders.
"""

import os

import yaml
import pytest
from ament_index_python.packages import get_package_share_directory

from mote_arm import calibrate, config as arm_config
from mote_bringup.launch_utils import (
    CONTROLLERS,
    INACTIVE_CONTROLLERS,
    arm_config_file,
    arm_on_wheel_bus,
    joint_params_file,
    resolved_arm,
    spawn_controllers,
)

ROBOT_YAML = os.path.join(
    get_package_share_directory("mote_description"), "config", "robot.yaml"
)
CONTROLLERS_YAML = os.path.join(
    get_package_share_directory("mote_bringup"), "config", "controllers.yaml"
)


@pytest.fixture
def cfg():
    with open(ROBOT_YAML) as f:
        return yaml.safe_load(f)


def _arguments(action):
    return [
        arg[0].text if isinstance(arg, list) else str(arg)
        for arg in action.__dict__["_Node__arguments"]
    ]


def test_arm_is_spawned_inactive():
    # Activation claims the position command interfaces, which is what makes
    # MoteHardware enable torque. A robot that boots with the arm active would
    # be holding a pose nobody asked for.
    actions = spawn_controllers(inactive=INACTIVE_CONTROLLERS)
    inactive = [a for a in actions if "--inactive" in _arguments(a)]
    assert [_arguments(a)[0] for a in inactive] == list(INACTIVE_CONTROLLERS)


def test_active_controllers_are_not_marked_inactive():
    for action in spawn_controllers(inactive=INACTIVE_CONTROLLERS):
        args = _arguments(action)
        if args[0] in CONTROLLERS:
            assert "--inactive" not in args


def test_no_arm_spawner_when_the_arm_is_not_asked_for():
    # The sim, and any base built without the arm, go down this path.
    assert all("--inactive" not in _arguments(a) for a in spawn_controllers())


def test_arm_shares_the_wheel_bus_in_the_shipped_config(cfg):
    # The premise of the whole design: one process, one port. If this ever
    # stops being true the arm needs its own hardware component.
    assert arm_on_wheel_bus(cfg)
    assert cfg["arm"]["port"] == cfg["servos"]["port"]


def test_arm_on_a_separate_bus_is_not_part_of_this_component(cfg):
    cfg["arm"]["port"] = "/dev/some_other_bus"
    assert not arm_on_wheel_bus(cfg)


def test_joint_params_carry_the_arm_joints_from_robot_yaml(cfg):
    with open(joint_params_file(cfg, resolved_arm(cfg))) as f:
        params = yaml.safe_load(f)

    joints = params["arm_controller"]["ros__parameters"]["joints"]
    assert joints == [j["name"] for j in cfg["arm"]["joints"]]
    # ...and the wheel geometry is still injected the same way.
    wheels = params["diff_drive_controller"]["ros__parameters"]
    assert wheels["wheel_radius"] == cfg["wheel_radius"]


def test_no_arm_controller_params_when_the_arm_is_elsewhere(cfg):
    cfg["arm"]["port"] = "/dev/some_other_bus"
    assert resolved_arm(cfg) is None
    with open(joint_params_file(cfg, None)) as f:
        params = yaml.safe_load(f)
    assert "arm_controller" not in params


def test_the_urdf_gets_the_calibrated_zero_and_limits(tmp_path, monkeypatch):
    """The whole point of resolving the arm at launch.

    Calibration lives in $MOTE_HOME/arm.yaml because zero/min/max are
    measurements of one physical arm. If the launch handed xacro the packaged
    defaults instead, MoteHardware would clamp against limits this robot does
    not have — and, because calibration moves the zero, every commanded angle
    would name a different physical position.
    """
    monkeypatch.setenv("MOTE_HOME", str(tmp_path))
    # Written by the real writer, so a change to the calibration file's shape
    # fails here rather than silently ceasing to reach the URDF.
    calibrate.save_calibration(
        {
            "recorded": "a test",
            "joints": {"elbow_flex": {"zero": 2048, "min": -1.5, "max": 1.5}},
        }
    )

    with open(arm_config_file(arm_config.load())) as f:
        emitted = yaml.safe_load(f)

    elbow = next(j for j in emitted["joints"] if j["name"] == "elbow_flex")
    assert elbow["zero"] == 2048
    assert (elbow["min"], elbow["max"]) == (-1.5, 1.5)
    # Untouched joints keep the packaged defaults, and ids are never overridden.
    assert elbow["id"] == 3


def test_emitted_arm_config_is_the_shape_the_xacro_reads(cfg):
    with open(arm_config_file(resolved_arm(cfg))) as f:
        emitted = yaml.safe_load(f)

    # mote.urdf.xacro indexes exactly these keys; a rename here is a silent
    # URDF-generation failure otherwise.
    assert {"port", "moving_speed", "moving_acc", "joints"} <= set(emitted)
    for joint in emitted["joints"]:
        assert {"name", "id", "min", "max", "zero", "invert"} == set(joint)


def test_controllers_yaml_does_not_duplicate_the_joint_list():
    # robot.yaml is the single source of truth; a second copy here would drift.
    cfg = yaml.safe_load(open(CONTROLLERS_YAML))
    assert "joints" not in cfg["arm_controller"]["ros__parameters"]
    assert (
        cfg["controller_manager"]["ros__parameters"]["arm_controller"]["type"]
        == "joint_trajectory_controller/JointTrajectoryController"
    )


def test_arm_controller_commands_and_reads_position_only():
    # The hardware exports no velocity interface for the arm, so asking for one
    # here would leave the controller unable to activate.
    params = yaml.safe_load(open(CONTROLLERS_YAML))["arm_controller"]["ros__parameters"]
    assert params["command_interfaces"] == ["position"]
    assert params["state_interfaces"] == ["position"]
