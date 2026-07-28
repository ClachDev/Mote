"""Unit tests for named-pose storage (ROS-free)."""

import pytest

from mote_arm import poses


def test_load_missing_file_is_empty(tmp_path):
    assert poses.load_poses(tmp_path / "nope.yaml") == {}


def test_save_then_load_roundtrip(tmp_path):
    path = tmp_path / "arm_poses.yaml"
    poses.save_pose("safe", {"shoulder_pan": 0.25, "gripper": -0.1}, path)
    loaded = poses.load_poses(path)
    assert loaded == {"safe": {"shoulder_pan": 0.25, "gripper": -0.1}}


def test_second_pose_does_not_clobber_first(tmp_path):
    path = tmp_path / "arm_poses.yaml"
    poses.save_pose("a", {"shoulder_pan": 0.1}, path)
    poses.save_pose("b", {"shoulder_pan": 0.2}, path)
    loaded = poses.load_poses(path)
    assert sorted(loaded) == ["a", "b"]
    assert loaded["a"]["shoulder_pan"] == 0.1


def test_resaving_replaces(tmp_path):
    path = tmp_path / "arm_poses.yaml"
    poses.save_pose("a", {"shoulder_pan": 0.1}, path)
    poses.save_pose("a", {"shoulder_pan": 0.9}, path)
    assert poses.load_poses(path)["a"]["shoulder_pan"] == 0.9


def test_delete(tmp_path):
    path = tmp_path / "arm_poses.yaml"
    poses.save_pose("a", {"shoulder_pan": 0.1}, path)
    assert poses.delete_pose("a", path) is True
    assert poses.load_poses(path) == {}
    assert poses.delete_pose("a", path) is False


def test_empty_name_or_joints_rejected(tmp_path):
    path = tmp_path / "arm_poses.yaml"
    with pytest.raises(ValueError):
        poses.save_pose("", {"shoulder_pan": 0.1}, path)
    with pytest.raises(ValueError):
        poses.save_pose("a", {}, path)


def test_mote_home_env_override(tmp_path, monkeypatch):
    monkeypatch.setenv("MOTE_HOME", str(tmp_path))
    assert poses.poses_path() == tmp_path / "arm_poses.yaml"


def test_creates_parent_directory(tmp_path):
    path = tmp_path / "nested" / "deeper" / "arm_poses.yaml"
    poses.save_pose("a", {"shoulder_pan": 0.1}, path)
    assert path.exists()


def test_envelope_spans_taught_poses():
    taught = {
        "a": {"elbow_flex": -3.19, "gripper": 0.0},
        "b": {"elbow_flex": -2.90, "gripper": 0.4},
    }
    band = poses.envelope(taught)
    assert band["elbow_flex"] == (-3.19, -2.90)
    assert band["gripper"] == (0.0, 0.4)


def test_envelope_margin_widens_both_ends():
    band = poses.envelope({"a": {"j": 1.0}, "b": {"j": 2.0}}, margin=0.1)
    lo, hi = band["j"]
    assert lo == pytest.approx(0.9)
    assert hi == pytest.approx(2.1)


def test_envelope_single_pose_is_a_point_plus_margin():
    band = poses.envelope({"a": {"j": 1.0}}, margin=0.25)
    assert band["j"] == pytest.approx((0.75, 1.25))


def test_envelope_omits_untaught_joints():
    band = poses.envelope({"a": {"j": 1.0}})
    assert "other" not in band


def test_envelope_rejects_negative_margin():
    with pytest.raises(ValueError):
        poses.envelope({"a": {"j": 1.0}}, margin=-0.1)


def test_interpolate_bounds_each_hop():
    steps = poses.interpolate({"j": 0.0}, {"j": 1.0}, 0.25)
    assert len(steps) == 4
    prev = 0.0
    for s in steps:
        assert abs(s["j"] - prev) <= 0.25 + 1e-9
        prev = s["j"]


def test_interpolate_ends_exactly_on_target():
    steps = poses.interpolate({"a": 0.0, "b": 1.0}, {"a": -3.19, "b": 1.2}, 0.2)
    assert steps[-1] == pytest.approx({"a": -3.19, "b": 1.2})


def test_interpolate_paces_on_largest_joint():
    """A small-moving joint gets the same hop count as the big one."""
    steps = poses.interpolate({"a": 0.0, "b": 0.0}, {"a": 1.0, "b": 0.01}, 0.25)
    assert len(steps) == 4
    assert steps[-1]["b"] == pytest.approx(0.01)


def test_interpolate_short_move_is_one_hop():
    assert poses.interpolate({"j": 0.0}, {"j": 0.05}, 0.2) == [{"j": 0.05}]


def test_interpolate_ignores_joints_not_in_start():
    steps = poses.interpolate({"a": 0.0}, {"a": 0.1, "ghost": 5.0}, 0.2)
    assert all("ghost" not in s for s in steps)


def test_interpolate_rejects_nonpositive_step():
    with pytest.raises(ValueError):
        poses.interpolate({"j": 0.0}, {"j": 1.0}, 0.0)


def test_shift_poses_preserves_the_physical_position():
    """The whole point: a migrated pose points where it always did.

    A pose is radians about the zero. Take a joint physically at encoder count
    3000 with zero 2048, re-zero it to 2500, and the stored value must change by
    exactly the amount that keeps it naming count 3000.
    """
    from mote_arm.calibrate import zero_shift
    from mote_arm.config import JointSpec

    before = JointSpec("j", 1, min_rad=-9, max_rad=9, zero_counts=2048)
    after = JointSpec("j", 1, min_rad=-9, max_rad=9, zero_counts=2500)
    at_counts = 3000

    taught = {"grip": {"j": before.counts_to_rad(at_counts)}}
    shift = zero_shift(before.zero_counts, after.zero_counts, before.invert)
    migrated = poses.shift_poses(taught, {"j": shift})

    assert migrated["grip"]["j"] == pytest.approx(after.counts_to_rad(at_counts))
    # And it really is a different number than before — the pose was invalidated.
    assert migrated["grip"]["j"] != pytest.approx(taught["grip"]["j"])


def test_shift_poses_preserves_position_for_an_inverted_joint():
    from mote_arm.calibrate import zero_shift
    from mote_arm.config import JointSpec

    before = JointSpec("j", 1, min_rad=-9, max_rad=9, zero_counts=2048, invert=True)
    after = JointSpec("j", 1, min_rad=-9, max_rad=9, zero_counts=1700, invert=True)
    at_counts = 900
    taught = {"p": {"j": before.counts_to_rad(at_counts)}}
    shift = zero_shift(before.zero_counts, after.zero_counts, before.invert)
    migrated = poses.shift_poses(taught, {"j": shift})
    assert migrated["p"]["j"] == pytest.approx(after.counts_to_rad(at_counts))


def test_shift_poses_leaves_unshifted_joints_alone():
    taught = {"p": {"a": 1.0, "b": 2.0}}
    assert poses.shift_poses(taught, {"a": 0.5}) == {"p": {"a": 1.5, "b": 2.0}}


def test_shift_poses_touches_every_pose():
    taught = {"one": {"a": 0.0}, "two": {"a": 1.0}}
    out = poses.shift_poses(taught, {"a": -0.25})
    assert out == {"one": {"a": -0.25}, "two": {"a": 0.75}}


def test_save_poses_keeps_a_backup_of_what_it_replaced(tmp_path):
    path = tmp_path / "arm_poses.yaml"
    poses.save_pose("home", {"a": 1.0}, path)
    poses.save_poses({"home": {"a": 2.0}}, path)
    assert poses.load_poses(path)["home"]["a"] == 2.0
    assert poses.load_poses(path.with_suffix(".yaml.bak"))["home"]["a"] == 1.0


def test_save_poses_without_an_existing_file_makes_no_backup(tmp_path):
    path = tmp_path / "arm_poses.yaml"
    poses.save_poses({"home": {"a": 1.0}}, path)
    assert not path.with_suffix(".yaml.bak").exists()
