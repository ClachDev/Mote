#!/usr/bin/env python3
"""Tests for the bag-replay harness's ROS-free pieces.

Deliberately node-less: nothing here calls rclpy, launches a stack, or opens a
bag, so the test path can never reach a live robot (the harness's live path
additionally pins a random ROS_DOMAIN_ID — asserted below). Runs standalone
(`python test_bag_replay.py`) or under pytest. The ROS replay itself needs
slam_toolbox and a real bag and is exercised by `pixi run bag-replay`, not here.
"""

import json
import math
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import acceptance
import render
import replay
import report
import tf_lookup


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


# ---------------------------------------------------------------------------
# The acceptance chain (acceptance.py) — a transcription of slam_toolbox's own
# gates, so these tests pin the behaviours that make it a transcription rather
# than an approximation. The failure they exist to prevent is silent: a chain
# that is merely close feeds slam a different set of scans and yields a
# different map with nothing in any log to say so.

GATES = acceptance.Gates(
    throttle_scans=1,
    minimum_time_interval_ns=500_000_000,
    minimum_travel_distance=0.3,
    minimum_travel_heading=0.3,
)


def _straight(n, step=0.02, dt=0.1, y=0.0, yaw=0.0, t0=1_000_000_000):
    """A robot driving straight along +x: n scans, `step` metres apart."""
    return [(t0 + int(i * dt * 1e9), (i * step, y, yaw)) for i in range(n)]


def _spin(n, dyaw=0.02, dt=0.1, t0=1_000_000_000):
    """A robot turning in place at the origin."""
    return [(t0 + int(i * dt * 1e9), (0.0, 0.0, i * dyaw)) for i in range(n)]


def test_gates_read_the_committed_params():
    import yaml

    params = yaml.safe_load((replay.DEFAULT_SLAM_PARAMS).read_text())
    g = acceptance.Gates.from_params(params)
    assert g.throttle_scans == 1
    assert g.minimum_time_interval_ns == 500_000_000
    assert g.minimum_travel_distance == 0.3
    assert g.minimum_travel_heading == 0.3
    assert g.check_precisely is False


def test_minimum_time_interval_defaults_to_transform_timeout():
    # Upstream reuses one scratch variable for both parameters, so an unset
    # minimum_time_interval inherits whatever transform_timeout resolved to.
    # Preserved deliberately: guessing 0.5 here would mispredict any params file
    # that sets transform_timeout and not the interval.
    g = acceptance.Gates.from_params({"transform_timeout": 0.25})
    assert g.minimum_time_interval_ns == 250_000_000
    assert acceptance.Gates.from_params({}).minimum_time_interval_ns == 500_000_000


def test_first_scan_is_always_inserted():
    d = acceptance.simulate(_straight(3), GATES)
    assert d[0] == acceptance.ACCEPT


def test_warmup_rejects_the_next_three_scans():
    # shouldProcessScan's `scan_ctr < 5`: even a scan that has moved far enough
    # is refused until the fifth. Feeding without reproducing this silently loses
    # the first insertions of every leg.
    d = acceptance.simulate(_straight(6, step=1.0, dt=0.6), GATES)
    assert d == [acceptance.ACCEPT] + [acceptance.NODE_REJECT] * 3 + [
        acceptance.ACCEPT,
        acceptance.ACCEPT,
    ]


def test_node_gate_relaxes_the_distance_but_the_mapper_does_not():
    # The node passes at 0.8x the squared distance gate; karto then wants the
    # full distance. Scans landing in that band are accepted by one and rejected
    # by the other — they must still be fed, because the node's anchor moves.
    d = acceptance.simulate(_straight(120), GATES)
    assert acceptance.MAPPER_REJECT in d


def test_the_two_anchors_advance_independently():
    # The defect the first attempt at a fast feed had: chaining one anchor for
    # both quantises insertion spacing. Here the node's anchor advances on a
    # scan karto discards, so the *next* node acceptance is ~2x0.268 m from the
    # last insertion — visibly more than the 0.3 m gate, and impossible to
    # reproduce with a single chained anchor.
    scans = _straight(400)
    d = acceptance.simulate(scans, GATES)
    inserted = [scans[i][1][0] for i, x in enumerate(d) if x == acceptance.ACCEPT]
    gaps = [b - a for a, b in zip(inserted[2:], inserted[3:])]
    assert gaps, "expected several insertions"
    assert max(gaps) > 0.4, f"anchors are not diverging: {gaps[:5]}"


def test_pure_rotation_is_gated_out_at_the_node():
    # There is no heading test in shouldProcessScan, so minimum_travel_heading is
    # unreachable while the base is not translating — a real and non-obvious
    # property of the stack, not a simplification here.
    d = acceptance.simulate(_spin(200), GATES)
    assert d[0] == acceptance.ACCEPT
    assert set(d[1:]) == {acceptance.NODE_REJECT}


def test_check_precisely_lets_rotation_through():
    g = acceptance.Gates(
        minimum_travel_distance=0.3, minimum_travel_heading=0.3, check_precisely=True
    )
    d = acceptance.simulate(_spin(200), g)
    assert acceptance.ACCEPT in d[5:]


def test_time_interval_gates_a_fast_mover():
    # 1 m per scan at 10 Hz: distance is never the binding gate, the 0.5 s
    # interval is, so insertions land every fifth scan.
    d = acceptance.simulate(_straight(30, step=1.0), GATES)
    accepted = [i for i, x in enumerate(d) if x == acceptance.ACCEPT]
    assert accepted == [0, 5, 10, 15, 20, 25]


def test_throttle_skips_scans_before_any_other_gate():
    g = acceptance.Gates(
        throttle_scans=3,
        minimum_time_interval_ns=0,
        minimum_travel_distance=0.3,
        minimum_travel_heading=0.3,
    )
    d = acceptance.simulate(_straight(40, step=1.0), g)
    accepted = [i for i, x in enumerate(d) if x == acceptance.ACCEPT]
    # scan_ctr is 1-based, so the multiples of three are indices 2, 5, 8, …
    assert all(i % 3 == 2 for i in accepted[1:])


def test_scans_without_transforms_never_reach_the_counter():
    # The real node returns from laserCallback before shouldProcessScan when the
    # odom pose cannot be read, so those scans do not consume the warm-up.
    scans = [(t, None) for t, _ in _straight(4)] + _straight(
        6, step=1.0, t0=2_000_000_000
    )
    d = acceptance.simulate(scans, GATES)
    assert d[:4] == [acceptance.NO_TF] * 4
    assert d[4] == acceptance.ACCEPT  # the first scan the node actually sees
    assert d[5:8] == [acceptance.NODE_REJECT] * 3  # warm-up, counted from there


def test_decisions_survive_rebasing_the_odometry():
    # --frame pre-multiplies a rigid SE2 onto the prior; every gate is a relative
    # measure, so which scans are kept must not depend on it.
    scans = _straight(300)
    rebased = [
        (t, tuple(replay_rebase(p, (3.0, -7.0, math.radians(31.0))))) for t, p in scans
    ]
    assert acceptance.simulate(scans, GATES) == acceptance.simulate(rebased, GATES)


def replay_rebase(pose, frame):
    x, y, yaw = pose
    fx, fy, fyaw = frame
    c, s = math.cos(fyaw), math.sin(fyaw)
    return (c * x - s * y + fx, s * x + c * y + fy, yaw + fyaw)


def test_laser_offset_moves_the_mapper_gate():
    # karto measures the *sensor*, not the base, so a forward-mounted lidar makes
    # a turn count as travel. Ignoring the offset would mispredict insertions.
    scans = _straight(400)
    at_base = acceptance.simulate(scans, GATES, (0.0, 0.0, 0.0))
    offset = acceptance.simulate(scans, GATES, (0.08, 0.0, -math.pi / 2))
    assert at_base.count(acceptance.ACCEPT) == offset.count(acceptance.ACCEPT)
    # Creeping forward while turning: the base travels 0.28 m between node
    # acceptances — short of the 0.3 m mapper gate — but a lidar 1 m ahead is
    # swung far enough by the same motion to clear it.
    turning = [
        (t, (0.02 * i, 0.0, 0.02 * i)) for i, (t, _) in enumerate(_straight(200))
    ]
    at_base = acceptance.simulate(turning, GATES, (0.0, 0.0, 0.0))
    ahead = acceptance.simulate(turning, GATES, (1.0, 0.0, 0.0))
    assert acceptance.MAPPER_REJECT in at_base
    assert ahead.count(acceptance.ACCEPT) > at_base.count(acceptance.ACCEPT)


def test_pad_before_clears_the_warmup_and_the_throttle():
    assert acceptance.pad_before(0, 1, first=True) == 0
    assert acceptance.pad_before(1, 1, first=False) == 3  # ctr 2 -> 5
    assert acceptance.pad_before(5, 1, first=False) == 0  # ctr 6, already past
    assert acceptance.pad_before(1, 3, first=False) == 4  # ctr 2 -> 6
    assert acceptance.pad_before(6, 2, first=False) == 1  # ctr 7 -> 8


def test_feed_plan_lands_every_scan_on_a_counter_the_node_will_accept():
    """Replay the padded stream through the counter-dependent gates themselves.

    Real scans must land where neither the throttle nor the warm-up can fire, and
    every filler must land where one of them certainly does — the property that
    makes a filler's *content* irrelevant.
    """
    for throttle in (1, 2, 3):
        g = acceptance.Gates(
            throttle_scans=throttle,
            minimum_time_interval_ns=500_000_000,
            minimum_travel_distance=0.3,
            minimum_travel_heading=0.3,
        )
        decisions = acceptance.simulate(_straight(600), g)
        plan = acceptance.feed_plan(decisions, g)
        assert plan
        assert any(s.role == acceptance.FILLER for s in plan)
        for ctr, step in enumerate(plan, start=1):
            passes = ctr % throttle == 0 and ctr >= acceptance.WARMUP_MIN_CTR
            if step.role == acceptance.FILLER:
                assert not passes, (throttle, ctr)
            elif ctr > 1:  # the first takes the free first-measurement pass
                assert passes, (throttle, ctr)


def test_feed_plan_pads_with_the_bags_own_rejected_scans():
    # Never a re-send: slam's transform cache is 30 s deep and a lockstep leg
    # outruns that immediately, so a re-sent stamp cannot be placed and would
    # move no counter at all.
    decisions = acceptance.simulate(_straight(600), GATES)
    plan = acceptance.feed_plan(decisions, GATES)
    assert len({s.index for s in plan}) == len(plan)  # each index published once
    for s in plan:
        if s.role == acceptance.FILLER:
            assert decisions[s.index] == acceptance.NODE_REJECT


def test_feed_plan_feeds_the_scans_the_mapper_will_discard():
    decisions = acceptance.simulate(_straight(400), GATES)
    plan = acceptance.feed_plan(decisions, GATES)
    real = {s.index for s in plan if s.role != acceptance.FILLER}
    for i, d in enumerate(decisions):
        assert (d in (acceptance.ACCEPT, acceptance.MAPPER_REJECT)) == (i in real)
    assert any(s.role == acceptance.MAPPER_REJECT for s in plan)
    assert not any(s.expect_ack for s in plan if s.role != acceptance.ACCEPT)


def test_feed_plan_refuses_when_the_counter_cannot_be_realigned():
    # Back-to-back insertions leave nothing to pad the warm-up with. Inventing a
    # filler instead of refusing is the silent-divergence route.
    try:
        acceptance.feed_plan([acceptance.ACCEPT, acceptance.ACCEPT], GATES)
    except ValueError as e:
        assert "realign" in str(e)
    else:
        raise AssertionError("expected a refusal")


# ---------------------------------------------------------------------------
# The offline tf2 (tf_lookup.py). The pose the simulator predicts on has to be
# the one slam_toolbox's own buffer will produce, so interpolation and the
# refusal to extrapolate are the parts that matter.


def _quat_z(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def _tree():
    t = tf_lookup.TfTree()
    t.add_static("base_footprint", "base_link", ((0.0, 0.0, 0.0325), _quat_z(0.0)))
    t.add_static("base_link", "lidar", ((0.08, 0.0, 0.04), _quat_z(-math.pi / 2)))
    t.add_dynamic(
        "odom", "base_footprint", 1_000_000_000, ((0.0, 0.0, 0.0), _quat_z(0.0))
    )
    t.add_dynamic(
        "odom", "base_footprint", 2_000_000_000, ((2.0, 0.0, 0.0), _quat_z(1.0))
    )
    t.finalize()
    return t


def test_lookup_interpolates_between_bracketing_samples():
    t = _tree()
    x, y, yaw = tf_lookup.se2_of(t.lookup("odom", "base_link", 1_500_000_000))
    assert abs(x - 1.0) < 1e-9 and abs(y) < 1e-9
    assert abs(yaw - 0.5) < 1e-9


def test_lookup_hits_a_sample_exactly():
    t = _tree()
    assert tf_lookup.se2_of(t.lookup("odom", "base_link", 2_000_000_000))[0] == 2.0


def test_lookup_refuses_to_extrapolate():
    t = _tree()
    assert t.lookup("odom", "base_link", 999_999_999) is None
    assert t.lookup("odom", "base_link", 2_000_000_001) is None


def test_lookup_composes_the_static_chain():
    t = _tree()
    x, y, yaw = tf_lookup.se2_of(t.lookup("base_link", "lidar", 1_500_000_000))
    assert (round(x, 9), round(y, 9)) == (0.08, 0.0)
    assert abs(yaw + math.pi / 2) < 1e-9


def test_dynamic_chain_names_only_the_time_varying_edges():
    t = _tree()
    assert t.dynamic_chain("odom", "lidar") == {("odom", "base_footprint")}
    assert t.dynamic_chain("base_link", "lidar") == set()


# ---------------------------------------------------------------------------
# --validate's comparison


def _leg(n_inserted, w, h, grid, first_stamp=0):
    with tempfile.NamedTemporaryFile(suffix=".npz", delete=False) as f:
        np.savez_compressed(f.name, grid=grid, resolution=np.float64(0.05))
        return {
            "n_inserted": n_inserted,
            "inserted_stamps": [first_stamp + i for i in range(n_inserted)],
            "wall_s": 100.0,
            "map": {"width": w, "height": h, "resolution": 0.05, "origin": [0.0, 0.0]},
            "map_npz": f.name,
        }


def test_compare_legs_passes_on_a_near_identical_map():
    a = np.zeros((40, 40), dtype=np.int16)
    b = a.copy()
    b[0, :3] = 100  # 3 cells of 1600 differ: 99.8%
    r = replay.compare_legs(_leg(500, 40, 40, a), _leg(500, 40, 40, b))
    assert r["pass"] and r["cell_agreement"] > 0.99


def test_compare_legs_fails_on_a_different_node_count():
    a = np.zeros((40, 40), dtype=np.int16)
    r = replay.compare_legs(_leg(500, 40, 40, a), _leg(499, 40, 40, a))
    assert not r["pass"] and not r["checks"]["node_count"]


def test_compare_legs_fails_when_the_same_count_is_different_scans():
    a = np.zeros((40, 40), dtype=np.int16)
    r = replay.compare_legs(
        _leg(500, 40, 40, a), _leg(500, 40, 40, a, first_stamp=1_000)
    )
    assert r["checks"]["node_count"]
    assert not r["pass"] and not r["checks"]["inserted_scans"]


def test_compare_legs_fails_on_different_dimensions():
    a = np.zeros((40, 40), dtype=np.int16)
    r = replay.compare_legs(_leg(500, 40, 40, a), _leg(500, 41, 40, a))
    assert not r["pass"] and not r["checks"]["map_dimensions"]


def test_compare_legs_fails_on_a_differently_solved_graph():
    # The first attempt's actual failure: same size, 63% of cells agreeing.
    rng = np.random.default_rng(0)
    a = np.zeros((100, 100), dtype=np.int16)
    b = a.copy()
    b[rng.random((100, 100)) > 0.63] = 100
    r = replay.compare_legs(_leg(500, 100, 100, a), _leg(500, 100, 100, b))
    assert not r["pass"] and r["cell_agreement"] < 0.7


def main():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print(f"ok  {fn.__name__}")
    print(f"\n{len(fns)} bag-replay tests passed")


if __name__ == "__main__":
    main()
