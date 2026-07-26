"""Unit tests for the jog step maths (soft-limit clamped increments)."""

from mote_arm.config import JointSpec
from mote_arm.jog import next_target


def test_step_within_limits():
    j = JointSpec("j", 1, min_rad=-1.0, max_rad=1.0)
    assert abs(next_target(0.0, 0.05, j) - 0.05) < 1e-9
    assert abs(next_target(0.0, -0.05, j) + 0.05) < 1e-9


def test_step_clamped_at_upper_limit():
    j = JointSpec("j", 1, min_rad=-1.0, max_rad=1.0)
    assert next_target(0.98, 0.05, j) == 1.0
    # repeated jogs never exceed the limit
    assert next_target(1.0, 0.05, j) == 1.0


def test_step_clamped_at_lower_limit():
    j = JointSpec("j", 1, min_rad=-1.0, max_rad=1.0)
    assert next_target(-0.98, -0.05, j) == -1.0
