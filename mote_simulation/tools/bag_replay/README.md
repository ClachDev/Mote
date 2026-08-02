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
  workstation; too fast and SLAM can drop scans in wall time. Ignored under
  `--lockstep`, which is not paced against a clock at all.
- `--lockstep` — feed at the estimator's own consumption rate instead of the
  wall clock: **minutes per set instead of tens of minutes**, with no change to
  the map. See [Lockstep](#lockstep-compute-bound-replay).
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

## Lockstep: compute-bound replay

A paced replay costs what the bag cost to record — 14 minutes a leg during the
2026-07-29 flat campaign — and almost all of it is *waiting*. The SLAM compute
inside is one or two minutes: of run 5's 12,811 scans, slam_toolbox inserts 186
into its pose graph and gates out the rest. `--lockstep` pays the compute and
skips the wait:

```bash
pixi run bag-replay -- --bag <bag> --lockstep \
  --params a.yaml b.yaml
```

Two rules make it faithful rather than merely fast.

**Feed exactly what the node would keep, predicted rather than approximated.**
`acceptance.py` restates slam_toolbox's acceptance chain — `shouldProcessScan`'s
throttle, `minimum_time_interval`, 5-scan warm-up and *relaxed* travel test, then
karto's `HasMovedEnough` on the *sensor* frame at the full `minimum_travel_distance`
/ `minimum_travel_heading` — over the bag's odometry, with every threshold read
from the parameter set under test. The chain depends on nothing SLAM computes, so
it can be decided offline; `tf_lookup.py` resolves each scan's pose the way the
node's own tf2 buffer will, interpolation and all.

**Then hold the prediction to account.** Each predicted insertion must be
acknowledged on `/pose` — which slam publishes per graph node, at the scan's own
stamp — before the next scan goes out. That is what sets the pace (so queue-full
drops are impossible by construction), and a missing, duplicated or unpredicted
acknowledgement **fails the leg** instead of quietly producing a different map.

Four details are load-bearing and were not obvious. Each one was found by a
lockstep leg silently disagreeing with the reference, not by reading the code:

- **The two gates keep separate anchors, and the node's advances even when karto
  rejects.** A scan in the band between `0.8 × minimum_travel_distance²` and the
  full gate is accepted by the node, discarded by the mapper, and *still has to
  be fed* — withholding it leaves the node's anchor where the real one would not
  be. This is precisely what sank the first attempt at a fast feed (`cee548f`,
  now removed): it chained one anchor for both, so the feed spacing quantised
  slam's node spacing and the graph came out 63% cell-identical to the reference,
  with zero queue-full drops to hint at it.
- **The node counts every scan it receives, and lockstep does not send it every
  scan.** Two gates depend on that counter — `throttle_scans` and the warm-up —
  so the fed stream is padded back to where the full stream would have had it.
  Without the padding the second, third and fourth insertion of every leg vanish
  silently. The padding is drawn from the bag's *own* nearby gated-out scans, not
  from re-sending one already fed: slam's transform cache is 30 s deep and a
  lockstep leg outruns that in a moment, so a re-sent stamp is
  `earlier than all the data in the transform cache`, gets parked in the message
  filter, and moves no counter at all — measured, and the reason the first
  lockstep leg lost its second insertion. Which scans are used does not matter,
  because the padded counters always land below the warm-up threshold or on a
  non-`throttle_scans` multiple, where the chain returns before looking at
  anything else.
- **Transforms go out ahead of the scan that needs them.** In bag order a scan is
  recorded *before* the odometry derived from it; the live node parks it in a tf2
  message filter until the transform arrives, which would deadlock a feeder
  blocked on that scan's acknowledgement. tf2 interpolates between the same two
  samples either way, so the pose the node reads is unchanged.
- **The finished map is not "the newest map".** slam rebuilds the whole grid on a
  5-second *wall* timer, publishes it `transient_local`, and stamps it with the
  last scan it received — and its own `map_saver` holds a subscription, so a
  partial grid is always sitting there to be handed over. A replay that outruns
  that timer by two orders of magnitude will be given one: the first full-bag
  lockstep leg captured a map missing the last two minutes of the run, while its
  pose graph held exactly the reference's 186 scans. Both feeds now wait for a grid
  stamped at or after the last scan they fed, and say so if none arrives.

`--lockstep` composes with `--skip-secs` / `--stop-secs` / `--max-scans` (the
prediction applies the same window), with `--frame` (a rigid re-basing cannot
change a relative gate — asserted in the tests), and with posegraph
serialization, so a lockstep leg is still assemblable into a continuable site
revision. It is slam mode only.

### Proving it: `--validate`

```bash
pixi run bag-replay -- --bag <bag> --validate \
  --params mote_bringup/config/slam_toolbox_params.yaml
```

Replays one parameter set twice — paced reference first, then lockstep — and
diffs **pose-graph node count** and the **set of scans inserted** (exact: one
`/pose` per graph vertex, stamped with the scan that made it, so the same count
of *different* scans still fails), **map dimensions** (width, height,
resolution, origin) and **occupancy cell agreement** (≥99%; not 100%, because
the solver is not bit-exact run to run). Writes `validate.json` and exits
non-zero on a mismatch.

This is a committed mode, not a one-off, because what it checks is a
*transcription*: `acceptance.py` restates logic that lives in someone else's C++
(slam_toolbox 2.8.5 / karto_sdk). **Rerun it whenever slam_toolbox is upgraded,
whenever the gate logic here is touched, and before trusting lockstep with a
parameter file that reaches gates the committed one does not.**

Measured 2026-08-02, run 5's whole bag (`20260729_172428`, 12,811 scans /
21 min) under the committed `slam_toolbox_params.yaml`: 186 pose-graph nodes
each way from the same 186 scans, identical map dimensions and origin
(263×380 @ 0.05 m), **100.000% cell agreement**, **1289 s paced → 14 s lockstep
(92×)**. The lockstep leg's 14 s is 9 s of feed plus the wait for slam's next
whole-grid rebuild, which is a wall-clock 5-second timer and now the floor on
how fast a leg can finish.

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
pieces (metrics, scoring, rendering, report, domain-ID isolation, the acceptance
chain and its offline tf2) with no ROS graph, so the test path can never touch a
live robot. The acceptance-chain tests are the ones to read before changing
`acceptance.py`: each pins a specific way a merely-approximate chain diverges —
the warm-up, the two independent anchors, rotation being invisible to the node
gate, the sensor offset moving karto's, and the counter arithmetic that makes a
padding scan's content irrelevant. Run them with:

```bash
python mote_simulation/tools/bag_replay/test_bag_replay.py
python mote_simulation/tools/benchmark/test_metrics.py
```

The ROS replay itself needs `slam_toolbox` and a real bag and is exercised by
`pixi run bag-replay`, not by the unit tests.
