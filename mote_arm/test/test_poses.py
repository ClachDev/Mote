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
