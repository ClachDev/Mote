"""bag_pruner keeps a rosbag tree under its cap without touching live data."""

import os
import time

from mote_bringup.bag_pruner import prune

MB = 1_000_000


def _segment(run_dir, name, size, age_s):
    run_dir.mkdir(parents=True, exist_ok=True)
    path = run_dir / name
    path.write_bytes(b"\0" * size)
    t = time.time() - age_s
    os.utime(path, (t, t))
    return path


def _tree_size(root):
    return sum(
        os.path.getsize(os.path.join(d, f))
        for d, _sub, files in os.walk(root)
        for f in files
        if f.endswith(".mcap")
    )


def test_prunes_oldest_first_down_to_cap(tmp_path):
    old_run = tmp_path / "20260101_000000"
    new_run = tmp_path / "20260102_000000"
    oldest = _segment(old_run, "a_0.mcap", 10 * MB, 400)
    middle = _segment(old_run, "a_1.mcap", 10 * MB, 300)
    recent = _segment(new_run, "b_0.mcap", 10 * MB, 200)
    active = _segment(new_run, "b_1.mcap", 10 * MB, 0)

    prune(str(tmp_path), 35 * MB)

    assert not oldest.exists()
    assert middle.exists() and recent.exists() and active.exists()
    assert _tree_size(tmp_path) <= 35 * MB


def test_never_deletes_the_newest_segment(tmp_path):
    run = tmp_path / "20260101_000000"
    active = _segment(run, "a_0.mcap", 50 * MB, 0)

    prune(str(tmp_path), 1 * MB)

    assert active.exists()


def test_removes_stale_run_left_without_segments(tmp_path):
    stale = tmp_path / "20260101_000000"
    stale.mkdir()
    (stale / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n")
    t = time.time() - 3600
    os.utime(stale, (t, t))
    fresh = tmp_path / "20260102_000000"
    fresh.mkdir()

    prune(str(tmp_path), 100 * MB)

    assert not stale.exists()
    assert fresh.exists()


def test_repeated_writes_stay_bounded_by_cap_plus_active_segment(tmp_path):
    run = tmp_path / "20260101_000000"
    cap = 30 * MB
    seg = 10 * MB
    for i in range(20):
        _segment(run, f"a_{i}.mcap", seg, age_s=20 - i)
        prune(str(tmp_path), cap)
        assert _tree_size(tmp_path) <= cap + seg
    assert (run / "a_19.mcap").exists()
