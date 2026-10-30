"""A taught pose the arm cannot be sent to.

Found at the bench. Posing by hand means posing a limp
arm against its mechanical stops, and the soft limits sit a margin inside those,
so a captured position is routinely a fraction past the band — stored raw, the
pose can never be reached and every `go` clamps it and says so, minutes later,
when nothing can be done about it.
"""

import pytest

from mote_arm.config import ArmConfig, JointSpec, ServoGains


def cfg(*joints):
    return ArmConfig(
        port="/dev/fake",
        baud_rate=1000000,
        gains=ServoGains(64, 32, 0),
        joints=list(joints),
    )


def joint(name, low, high, invert=False):
    return JointSpec(name=name, id=1, min_rad=low, max_rad=high, invert=invert)


# --- what `save` stores ------------------------------------------------------


def clamp_pose(arm, captured):
    """What _cmd_save writes: every joint held inside its own soft band."""
    return {n: arm.joint(n).clamp_rad(v) for n, v in captured.items()}


def test_a_pose_captured_against_a_stop_is_stored_at_the_limit():
    arm = cfg(joint("elbow_flex", -1.6458, 1.6458))
    # Measured at the bench: hand-posed to the stop, a hair past the soft limit.
    stored = clamp_pose(arm, {"elbow_flex": -1.6582})
    assert stored["elbow_flex"] == pytest.approx(-1.6458)


def test_a_stored_pose_is_reachable_by_construction():
    """The property: `go` clamps nothing, because `save` already did."""
    arm = cfg(joint("elbow_flex", -1.6458, 1.6458), joint("gripper", -1.0729, 1.0729))
    stored = clamp_pose(arm, {"elbow_flex": -1.6582, "gripper": -1.1152})
    for name, value in stored.items():
        assert arm.joint(name).clamp_rad(value) == value


def test_a_pose_inside_the_band_is_stored_untouched():
    arm = cfg(joint("elbow_flex", -1.6458, 1.6458))
    assert clamp_pose(arm, {"elbow_flex": 0.25})["elbow_flex"] == 0.25
