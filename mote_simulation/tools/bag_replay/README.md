# Bag-replay scoring

Offline harness that replays a **real recorded mapping bag** through SLAM (or
kinematic_icp) under two or more parameter sets and scores the results with
**truth-free** proxies, then writes a side-by-side comparison report (metrics
table + rendered maps).

This is the reality check that complements the sim benchmark
(`mote_simulation/tools/benchmark/`, `pixi run bench`). The sim proves
parameters against *simulated* sensors and can report absolute error because
Gazebo publishes the robot's true pose. A real bag carries no ground truth, so
this harness scores *self-consistency* and *map appearance* instead — see
[Limitations](#limitations). The two share one metrics module: the truth-free
functions used here (`map_quality`, `loop_drift`, `path_length`) live in the
benchmark's `metrics.py`, reused not forked.

## Usage

```bash
pixi run bag-replay -- \
  --bag ~/.mote/bags/mapping/<timestamp> \
  --params mote_bringup/config/slam_toolbox_params.yaml \
           mote_simulation/tools/bag_replay/examples/sparse_no_loop.yaml
```

- `--bag` — a recorded **mapping** stream bag directory (`/tf`, `/tf_static`,
  `/scan_filtered`). These are recorded by default on every mapping session
  (`mapping_launch.py` → `record_launch.py streams:=mapping`) and stamped into
  the map revision by `save-map`; `pixi run site info` shows which bag built a
  map. Bags live under the robot's `~/.mote/bags/mapping/` — copy one over to
  score it (no code is synced *to* the Pi).
- `--params a.yaml b.yaml …` — one **complete** `slam_toolbox` params file per
  set (slam mode). Default: the committed `slam_toolbox_params.yaml` as a single
  baseline. Compare it against `examples/sparse_no_loop.yaml` (a variant with
  aggressive scan decimation + loop closure off) for a ready two-set example.
- `--mode slam|icp` — `slam` (default) scores the map + map-frame trajectory;
  `icp` scores kinematic_icp's odometry self-consistency (no map). ICP params
  are inline in the launch, so in icp mode `--params` are just repeat labels.
- `--rate` — replay speed × realtime (default 1.0). 2–3× is safe on a
  workstation; too fast and SLAM can drop scans in wall time.
- `--max-scans N` — stop after N scans (debug/quick smoke).

Output lands in `bag_replay_results/<UTC>/`: `report.md` (open it — the map
images are relative links), `run.json` (full metrics + provenance), and per set
a `stack.log`, `replay.log`, `series.json` (re-scorable trajectory), `map.npz`
(occupancy grid) and `map.png`.

## How it works

`replay.py` (orchestrator) runs each parameter set in its **own freshly-launched
stack under a random `ROS_DOMAIN_ID`** — so a stray node can never reach a live
robot, and sequential runs can't leak into each other (the sim sweep learned
that DDS-isolation lesson the hard way). Per set it:

1. launches `replay_stack_launch.py` (env packages only — `slam_toolbox` /
   `kinematic_icp` / `nav2_lifecycle_manager`, no workspace build needed),
2. runs `replayer.py`, which streams the bag's sensor + TF messages back onto
   the graph in **sim-time order** (it drives `/clock` from the bag stamps, so
   the stack runs on bag time regardless of wall speed), records the estimator's
   output trajectory, and grabs the finished `/map`,
3. scores with the shared `metrics` module and renders the map (`render.py`),
4. tears the stack down and moves on. `report.py` then builds the comparison.

**TF ownership** is the one subtlety. A mapping bag's `/tf` already contains the
edges the *original* run produced — `map→odom` (slam_toolbox) and
`odom→base_footprint` (kinematic_icp). Replaying those verbatim would fight the
fresh node publishing the same edge, so the edge the node-under-test owns is
stripped from the replayed `/tf`; everything else passes through. In slam mode
the recorded `odom→base` is kept and *fed* to slam as its odometry prior,
exactly as on the robot — so a slam-parameter comparison replays against
identical odometry.

## Metrics (all truth-free)

- **loop drift** (`loop_drift`) — start↔end distance of the estimated
  trajectory and its ratio to path length. Small on a bag where the robot
  physically returned to its start ⇒ little accumulated drift.
- **map crispness / coverage** (`map_quality`, slam mode) — from the finished
  occupancy grid: `mean_wall_thickness_m` (iterated erosion; blur/double-walls
  read thicker — lower is crisper), `speckle_frac` (isolated occupied cells —
  scan-match noise, lower is cleaner), `unknown/free/occ_frac`, and
  `explored_area_m2`.

## Limitations

These are **proxies, not error measures** — be honest about what they can and
cannot prove versus the sim's ground truth:

- **No absolute accuracy.** Without a surveyed reference the bag cannot tell you
  the map is metrically *correct*, only whether it is self-consistent and looks
  clean. For metric-accuracy claims (ATE), use the sim benchmark.
- **Loop drift needs a loop.** It cannot distinguish a legitimate open A→B
  traverse (honestly large end distance) from a drifting loop. Know the bag's
  shape before reading it.
- **Crispness ≠ correctness.** A confidently *wrong* map — e.g. a mis-closed
  loop drawn with sharp walls — can score well on wall thickness and speckle. The
  crispness proxies catch blur, noise, and incompleteness, not global error.
- **Not bit-exact.** The recorded sensor stream makes the *input* deterministic,
  but SLAM's solver is not bit-identical run to run; treat small deltas as noise
  and lean on the map images for anything marginal.

## Tests

`test_bag_replay.py` and the benchmark's `test_metrics.py` cover the ROS-free
pieces (metrics, scoring, rendering, report, domain-ID isolation) with no ROS
graph, so the test path can never touch a live robot. Run them with:

```bash
python mote_simulation/tools/bag_replay/test_bag_replay.py
python mote_simulation/tools/benchmark/test_metrics.py
```

The ROS replay itself needs `slam_toolbox` and a real bag and is exercised by
`pixi run bag-replay`, not by the unit tests.
