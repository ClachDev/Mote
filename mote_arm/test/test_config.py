"""Unit tests for the arm config parsing, conversions, and soft-limit clamping.

Pure maths — no ROS, no serial hardware — so they run under `pixi run test`
and guard the safety-critical logic the jog tool and driver rely on.
"""

import math

import pytest

from mote_arm.config import COUNTS_PER_REV, ArmConfig, JointSpec

BASE = {
    "arm": {
        "port": "/dev/mote_arm",
        "baud_rate": 1000000,
        "moving_speed": 500,
        "moving_acc": 20,
        "joints": [
            {"name": "shoulder_pan", "id": 1, "min": -1.5, "max": 1.5},
            {
                "name": "gripper",
                "id": 6,
                "min": 0.0,
                "max": 1.0,
                "home": 1024,
                "invert": True,
            },
        ],
    }
}


def test_parse_basic():
    cfg = ArmConfig.from_dict(BASE)
    assert cfg.port == "/dev/mote_arm"
    assert cfg.baud_rate == 1000000
    assert cfg.names == ["shoulder_pan", "gripper"]
    assert cfg.ids == [1, 6]
    assert cfg.moving_speed == 500


def test_missing_arm_section():
    with pytest.raises(KeyError):
        ArmConfig.from_dict({})


def test_min_greater_than_max_rejected():
    bad = {
        "arm": {
            "port": "/dev/x",
            "baud_rate": 1,
            "joints": [{"name": "j", "id": 1, "min": 1.0, "max": -1.0}],
        }
    }
    with pytest.raises(ValueError):
        ArmConfig.from_dict(bad)


def test_duplicate_ids_rejected():
    bad = {
        "arm": {
            "port": "/dev/x",
            "baud_rate": 1,
            "joints": [
                {"name": "a", "id": 1, "min": -1, "max": 1},
                {"name": "b", "id": 1, "min": -1, "max": 1},
            ],
        }
    }
    with pytest.raises(ValueError):
        ArmConfig.from_dict(bad)


def test_duplicate_names_rejected():
    bad = {
        "arm": {
            "port": "/dev/x",
            "baud_rate": 1,
            "joints": [
                {"name": "a", "id": 1, "min": -1, "max": 1},
                {"name": "a", "id": 2, "min": -1, "max": 1},
            ],
        }
    }
    with pytest.raises(ValueError):
        ArmConfig.from_dict(bad)


def test_clamp_within_and_outside():
    j = JointSpec("j", 1, min_rad=-1.0, max_rad=1.0)
    assert j.clamp_rad(0.5) == 0.5
    assert j.clamp_rad(2.0) == 1.0
    assert j.clamp_rad(-2.0) == -1.0


def test_zero_is_home_counts():
    j = JointSpec("j", 1, min_rad=-3, max_rad=3, home_counts=2048)
    assert j.rad_to_counts(0.0) == 2048
    assert j.counts_to_rad(2048) == 0.0


def test_conversion_roundtrip():
    j = JointSpec("j", 1, min_rad=-3, max_rad=3, home_counts=2048)
    for rad in (-1.0, -0.25, 0.0, 0.5, 1.2):
        counts = j.rad_to_counts(rad)
        back = j.counts_to_rad(counts)
        assert abs(back - rad) < 0.002  # within one encoder count


def test_quarter_turn_is_ninety_degrees():
    j = JointSpec("j", 1, min_rad=-3, max_rad=3, home_counts=0)
    assert j.rad_to_counts(math.pi / 2) == COUNTS_PER_REV // 4


def test_invert_flips_direction():
    fwd = JointSpec("f", 1, min_rad=-3, max_rad=3, home_counts=2048, invert=False)
    inv = JointSpec("i", 1, min_rad=-3, max_rad=3, home_counts=2048, invert=True)
    assert fwd.rad_to_counts(0.5) > 2048
    assert inv.rad_to_counts(0.5) < 2048


def test_rad_to_counts_saturates_at_encoder_edge():
    j = JointSpec("j", 1, min_rad=-100, max_rad=100, home_counts=2048)
    # A wildly out-of-range angle never produces an invalid encoder value.
    assert 0 <= j.rad_to_counts(50.0) <= COUNTS_PER_REV - 1
    assert 0 <= j.rad_to_counts(-50.0) <= COUNTS_PER_REV - 1


def test_real_robot_yaml_parses():
    """The committed robot.yaml arm section must be valid."""
    from pathlib import Path

    here = Path(__file__).resolve()
    # mote_arm/test/ -> repo root -> mote_description/config/robot.yaml
    robot_yaml = here.parents[2] / "mote_description" / "config" / "robot.yaml"
    if not robot_yaml.exists():
        pytest.skip("robot.yaml not found in source tree")
    cfg = ArmConfig.from_yaml_file(str(robot_yaml))
    assert len(cfg.joints) >= 1
    for j in cfg.joints:
        assert j.min_rad <= j.max_rad
