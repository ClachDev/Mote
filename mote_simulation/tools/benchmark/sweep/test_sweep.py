#!/usr/bin/env python3
"""Unit tests for the ROS-free sweep modules (spec / overrides / score).

Runnable standalone (`python test_sweep.py`) or under pytest — no ROS, no sim.
Pins the grid expansion, config merge, and scoring/feasibility maths so the
runner can be trusted without a full sim run.
"""

import json
import sys
import tempfile
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import overrides
import score
import spec as spec_mod


def _spec(**over):
    raw = {
        "name": "t",
        "benchmark": {"worlds": ["office_world.sdf"], "trials": 1},
        "parameters": [
            {
                "name": "particles",
                "file": "nav2",
                "path": "amcl.ros__parameters.max_particles",
                "values": [1000, 2000],
            },
            {
                "name": "inflation",
                "file": "nav2",
                "paths": [
                    [
                        "local_costmap",
                        "local_costmap",
                        "ros__parameters",
                        "inflation_layer",
                        "inflation_radius",
                    ],
                    [
                        "global_costmap",
                        "global_costmap",
                        "ros__parameters",
                        "inflation_layer",
                        "inflation_radius",
                    ],
                ],
                "range": {"start": 0.25, "stop": 0.35, "step": 0.05},
            },
        ],
    }
    raw.update(over)
    return spec_mod.Spec(raw)


def test_grid_expansion_and_baseline():
    s = _spec()
    # inflation range 0.25,0.30,0.35 -> 3 values; particles 2 -> grid 6.
    assert s.grid_size() == 6
    grid = s.grid()
    assert grid[0] == [], "baseline (defaults) must be first and empty"
    assert len(grid) == 7  # baseline + 6
    # Every non-baseline set assigns both parameters.
    for combo in grid[1:]:
        assert len(combo) == 2


def test_range_inclusive():
    s = _spec()
    infl = s.parameters[1]
    assert infl.values == [0.25, 0.30, 0.35]


def test_bad_target_rejected():
    try:
        spec_mod.Spec({"parameters": [{"file": "bogus", "path": "a.b", "values": [1]}]})
    except spec_mod.SpecError:
        return
    raise AssertionError("bad target should raise SpecError")


def test_path_xor_paths():
    try:
        spec_mod.Spec(
            {
                "parameters": [
                    {"file": "nav2", "path": "a", "paths": [["a"]], "values": [1]}
                ]
            }
        )
    except spec_mod.SpecError:
        return
    raise AssertionError("path+paths together should raise")


def test_deep_set_creates_and_sets():
    doc = {"amcl": {"ros__parameters": {"max_particles": 2000}}}
    overrides.deep_set(doc, ["amcl", "ros__parameters", "max_particles"], 4000)
    assert doc["amcl"]["ros__parameters"]["max_particles"] == 4000
    overrides.deep_set(doc, ["a", "b", "c"], 1)
    assert doc["a"]["b"]["c"] == 1


def test_deep_set_non_mapping_raises():
    doc = {"a": 5}
    try:
        overrides.deep_set(doc, ["a", "b"], 1)
    except KeyError:
        return
    raise AssertionError("traversing a non-mapping should raise KeyError")


def test_build_overrides_does_not_touch_base(tmp_path=None):
    tmp = Path(tmp_path or tempfile.mkdtemp())
    repo = tmp / "repo"
    (repo / "mote_bringup" / "config").mkdir(parents=True)
    base = repo / "mote_bringup" / "config" / "nav2_params.yaml"
    base.write_text(
        yaml.safe_dump(
            {
                "amcl": {"ros__parameters": {"max_particles": 2000}},
                "local_costmap": {
                    "local_costmap": {
                        "ros__parameters": {
                            "inflation_layer": {"inflation_radius": 0.35}
                        }
                    }
                },
            }
        )
    )
    before = base.read_text()
    s = _spec()
    # set: particles=1000, inflation=0.25
    assignments = [(s.parameters[0], 1000), (s.parameters[1], 0.25)]
    env = overrides.build_override_files(assignments, repo, tmp / "out")
    assert "MOTE_NAV2_PARAMS_FILE" in env
    merged = yaml.safe_load(Path(env["MOTE_NAV2_PARAMS_FILE"]).read_text())
    assert merged["amcl"]["ros__parameters"]["max_particles"] == 1000
    assert (
        merged["local_costmap"]["local_costmap"]["ros__parameters"]["inflation_layer"][
            "inflation_radius"
        ]
        == 0.25
    )
    assert base.read_text() == before, "committed base file must be untouched"


def test_improvement_directions():
    # higher-is-better: candidate above baseline -> positive
    assert score._improvement(1.0, 0.5, True) > 0
    # lower-is-better: candidate below baseline -> positive
    assert score._improvement(0.05, 0.10, False) > 0
    # worse localization -> negative
    assert score._improvement(0.20, 0.10, False) < 0
    # baseline zero, candidate zero -> zero
    assert score._improvement(0.0, 0.0, True) == 0.0


def _metrics(success=1.0, rmse=0.08, time=20.0, jerk=1.0):
    return {
        "office_world.sdf": {
            "success": success,
            "localization": rmse,
            "time": time,
            "smoothness": jerk,
        }
    }


def test_score_baseline_is_zero():
    base = _metrics()
    r = score.score_set(base, base)
    assert abs(r["total"]) < 1e-12


def test_faster_lower_error_beats_baseline():
    base = _metrics(rmse=0.10, time=25.0)
    better = _metrics(rmse=0.07, time=20.0)
    assert score.score_set(better, base)["total"] > 0


def test_feasibility_gate(tmp_path=None):
    tmp = Path(tmp_path or tempfile.mkdtemp())
    d = tmp / "trial_0"
    d.mkdir(parents=True)
    # straight-line 0.30 m/s -> per-wheel 0.30 > 0.218 wall
    (d / "series.json").write_text(json.dumps({"cmd": [[0.0, 0.30, 0.0]]}))
    f = score.feasibility(tmp, wheel_separation=0.22, wall_mps=0.218)
    assert f["feasible"] is False
    assert f["peak_wheel_mps"] > 0.218


def test_rank_disqualifies_infeasible_and_success_drop():
    baseline = {
        "index": 0,
        "metrics": _metrics(success=1.0),
        "feasibility": {"feasible": True, "peak_wheel_mps": 0.2},
    }
    fast_infeasible = {
        "index": 1,
        "metrics": _metrics(time=10.0),
        "feasibility": {"feasible": False, "peak_wheel_mps": 0.4},
    }
    dropped_goals = {
        "index": 2,
        "metrics": _metrics(success=0.5, time=8.0),
        "feasibility": {"feasible": True, "peak_wheel_mps": 0.2},
    }
    good = {
        "index": 3,
        "metrics": _metrics(rmse=0.05, time=18.0),
        "feasibility": {"feasible": True, "peak_wheel_mps": 0.21},
    }
    ranked = score.rank([baseline, fast_infeasible, dropped_goals, good])
    assert ranked[0]["index"] == 3, "the feasible improvement should win"
    assert not fast_infeasible["eligible"]
    assert not dropped_goals["eligible"]


def test_report_builds_with_unran_set():
    import sweep_report

    class _Spec:
        name = "t"
        worlds = ["office_world.sdf"]
        trials = 1
        goal_timeout = 120.0
        weights = None
        world_weights = None

    baseline = {
        "index": 0,
        "label": "baseline",
        "ran": True,
        "assignments": [],
        "metrics": _metrics(),
        "feasibility": {"feasible": True, "peak_wheel_mps": 0.2},
    }
    winner = {
        "index": 1,
        "label": "p=1",
        "ran": True,
        "assignments": [
            {
                "id": "nav2:a.b",
                "name": "p",
                "target": "nav2",
                "key_paths": [["a", "b"]],
                "value": 1,
            }
        ],
        "metrics": _metrics(rmse=0.05, time=18.0),
        "feasibility": {"feasible": True, "peak_wheel_mps": 0.21},
    }
    unran = {
        "index": 2,
        "label": "p=2",
        "ran": False,
        "assignments": [],
        "metrics": {},
        "feasibility": {"feasible": True, "peak_wheel_mps": None},
    }
    ranked = score.rank([baseline, winner])
    ordered = ranked + [unran]
    provenance = {"timestamp": "t", "git_commit": "abc", "spec": "s", "wall_mps": 0.218}
    md = sweep_report.build_markdown(
        ordered, baseline, _Spec(), provenance, {"nav2:a.b": 0}
    )
    assert "Winner" in md and "not run" in md


def _run():
    fns = [g for n, g in sorted(globals().items()) if n.startswith("test_")]
    for fn in fns:
        fn()
        print(f"ok  {fn.__name__}")
    print(f"\n{len(fns)} passed")


if __name__ == "__main__":
    _run()
