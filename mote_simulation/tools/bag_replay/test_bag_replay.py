#!/usr/bin/env python3
"""Tests for the bag-replay harness's ROS-free pieces.

Deliberately node-less: nothing here calls rclpy, launches a stack, or opens a
bag, so the test path can never reach a live robot (the harness's live path
additionally pins a random ROS_DOMAIN_ID — asserted below). Runs standalone
(`python test_bag_replay.py`) or under pytest. The ROS replay itself needs
slam_toolbox and a real bag and is exercised by `pixi run bag-replay`, not here.
"""

import json
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import render
import replay
import report


def test_isolated_env_pins_random_domain():
    # The live path must never run on the default domain 0 (a robot's domain).
    ids = set()
    for _ in range(50):
        env = replay.isolated_env()
        d = int(env["ROS_DOMAIN_ID"])
        assert 1 <= d <= 232
        ids.add(d)
    assert len(ids) > 1  # actually randomised, not a constant


def _fixture_run_dir(tmp):
    """A fake completed replay: series.json + map.npz for one set."""
    d = Path(tmp) / "baseline"
    d.mkdir(parents=True)
    traj = [
        [i * 0.1, np.cos(2 * np.pi * i / 120), np.sin(2 * np.pi * i / 120), 0.0]
        for i in range(120)
    ]
    (d / "series.json").write_text(
        json.dumps({"mode": "slam", "n_scans": 300, "traj": traj})
    )
    grid = np.full((80, 80), -1, dtype=np.int16)
    grid[20:60, 20:60] = 0
    grid[40, 20:60] = 100
    np.savez_compressed(
        d / "map.npz",
        grid=grid,
        resolution=np.float64(0.05),
        origin=np.array([-2.0, -2.0]),
    )
    return d


def test_score_from_recorded_outputs():
    with tempfile.TemporaryDirectory() as tmp:
        d = _fixture_run_dir(tmp)
        m, traj = replay.score(d / "series.json", d / "map.npz")
        assert m["n_scans"] == 300
        assert m["traj_samples"] == 120
        assert m["loop"]["start_end_dist_m"] < 0.2  # near-closed circle
        assert "mean_wall_thickness_m" in m["map"]
        assert m["map"]["occ_frac"] > 0.0
        assert len(traj) == 120


def test_render_writes_png():
    with tempfile.TemporaryDirectory() as tmp:
        d = _fixture_run_dir(tmp)
        traj = json.loads((d / "series.json").read_text())["traj"]
        out = Path(tmp) / "map.png"
        render.render_map(d / "map.npz", out, traj=traj)
        assert out.exists() and out.stat().st_size > 0


def test_report_compares_sets():
    results = [
        {
            "name": "baseline",
            "params_file": "a.yaml",
            "map_png": "baseline/map.png",
            "metrics": {
                "n_scans": 300,
                "traj_samples": 120,
                "loop": {
                    "start_end_dist_m": 0.05,
                    "path_length_m": 6.2,
                    "drift_ratio": 0.008,
                },
                "map": {
                    "explored_area_m2": 12.0,
                    "unknown_frac": 0.8,
                    "occ_frac": 0.01,
                    "mean_wall_thickness_m": 0.05,
                    "speckle_frac": 0.01,
                },
            },
        },
        {
            "name": "loose",
            "params_file": "b.yaml",
            "map_png": "loose/map.png",
            "metrics": {
                "n_scans": 300,
                "traj_samples": 120,
                "loop": {
                    "start_end_dist_m": 0.40,
                    "path_length_m": 6.4,
                    "drift_ratio": 0.06,
                },
                "map": {
                    "explored_area_m2": 11.0,
                    "unknown_frac": 0.82,
                    "occ_frac": 0.02,
                    "mean_wall_thickness_m": 0.14,
                    "speckle_frac": 0.05,
                },
            },
        },
    ]
    run = report.build_run(
        {
            "timestamp": "t",
            "git_commit": "abc",
            "bag": "/b",
            "mode": "slam",
            "rate": 1.0,
        },
        results,
    )
    md = report.build_markdown(run)
    assert "| baseline | loose |" in md
    assert "wall thickness (m) ↓" in md
    assert "![map for baseline](baseline/map.png)" in md
    assert "**0.050**" in md  # baseline wins wall thickness (lower is better)
    assert "Limitations" in md


def main():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print(f"ok  {fn.__name__}")
    print(f"\n{len(fns)} bag-replay tests passed")


if __name__ == "__main__":
    main()
