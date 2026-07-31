# Sim benchmark harness

Runs scripted nav missions in the headless Gazebo sim and scores them against
**Gazebo ground truth**, so Nav2 / SLAM / localization config changes can be
*proven* with numbers instead of eyeballed in RViz. This is the runner the
[parameter-sweep tool](sweep/README.md) (`pixi run bench-sweep`) and the IMU
study (`design/research/imu_fusion_study.md`) build on; the metric maths lives
in a ROS-free module (`metrics.py`) so an offline bag-replay scorer can reuse it
unchanged.

## Usage

```bash
pixi run bench                                          # default: mote_world + hospital_world, 2 trials each
pixi run bench -- --worlds mote_world.sdf --trials 3    # one world, 3 trials
pixi run bench -- --worlds mote_world.sdf,office_world.sdf,hospital_world.sdf
```

Needs a GPU workstation (headless gz-sim). Only one gz instance runs at a time,
so worlds and trials run strictly sequentially — a full default run is several
minutes. Each trial launches the *real* `sim_launch.py mode:=nav` (the same
mission `pixi run sim-nav` runs), so the benchmark exercises the production
launch files against each world's committed sim Site map.

Useful flags (`pixi run bench -- --help`):

| flag | default | meaning |
| --- | --- | --- |
| `--worlds` | `mote_world.sdf,hospital_world.sdf` | comma-separated world files |
| `--trials` | `2` | trials per world (repeat to measure variance) |
| `--order` | `pickup,dropoff,home` | zone names cycled as NavigateToPose goals |
| `--goal-timeout` | `120` | sim seconds allowed per goal |
| `--wheel-mu` | `1.0` | drive-wheel friction; `<1` induces wheel slip |
| `--slip` | off | shorthand for `--wheel-mu 0.4` (slip condition) |
| `--out` | `benchmark_results/` | output root (git-ignored) |

A caveat on `--slip`: lowering wheel friction does make the velocity-controlled
wheels slip, but kinematic_icp re-registers against the scan every frame, so the
slip is largely **absent from the resulting pose**. Do not expect it to degrade
the localization or odometry ATE much — see
`design/research/imu_fusion_study.md`. It is still useful for exercising code
that reads the *disagreement* between wheel odom and the scan-matched pose,
which slip does affect.

## What it measures

Per trial, gated on sim `/clock` (invariant to real-time factor):

- **localization error** — ATE (RMS/mean/median/max) of the estimated
  `map`→`base_footprint` pose vs. the bridged true pose, after a rigid SE(2)
  alignment (the SLAM `map` frame and the Gazebo world frame share no fixed
  transform, so alignment is required before differencing). Raw pre-alignment
  RMSE is also reported.
- **odometry error** — a second ATE of the **`odom`→`base_footprint`
  dead-reckoning** pose vs. truth, with no map correction (`odometry.*`). AMCL's
  map correction can mask an odometry change in the localization ATE above; this
  isolates odometry quality (kinematic_icp tuning, param sweeps) from the map
  correction that would otherwise hide a change in it.
- **goal success rate & time-to-goal** — over the scripted goal cycle.
- **clearance** — nearest obstacle from `/scan_filtered`: min, 5th-percentile,
  mean, and the fraction of time spent inside 0.15/0.20/0.30 m bands.
- **Nav2 recoveries / aborts** — distinct recovery goals seen on the
  behavior-server action status topics (`spin`, `backup`, `drive_on_heading`,
  `wait`), plus aborted NavigateToPose goals.
- **motion smoothness** — RMS linear/angular jerk from `cmd_vel` and the number
  of forward/backward direction reversals.

## Ground truth

The robot's true pose is bridged out of Gazebo with `ros_gz_bridge`
(`/model/mote/pose`, `gz.msgs.Pose` → `geometry_msgs/PoseStamped`), published by a
`PosePublisher` system plugin on the robot model (added to the URDF under the
`use_sim` guard, so the real-robot description is unchanged). `SceneBroadcaster`'s
`/world/<world>/pose/info` was the obvious source but loses model frame names
through the bridge; `PosePublisher` populates them. The bridge is started and torn
down by the harness, so the mission under test is unperturbed.

## Graph isolation

A benchmark is only meaningful if the graph it measures is its own, so each run
is fenced off twice:

- **Off this machine.** The `sim` pixi environment exports
  `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, so every sim/benchmark participant
  confines DDS discovery to this host. A robot or another workstation on the LAN
  can neither be discovered by nor discover a benchmark run.
- **From other runs on this machine.** `bench.py` claims a free `ROS_DOMAIN_ID`
  per invocation (probing which CycloneDDS discovery ports are already bound)
  plus a matching `GZ_PARTITION` for Gazebo's own transport, and records both in
  `run.json` / `report.md`. An inherited `ROS_DOMAIN_ID` is respected instead —
  that is how [the sweep](sweep/README.md) pins all of its sets to one domain.
  The picker is [`../sim_domain.py`](../sim_domain.py), shared with the smoke
  test and `map_world.sh` so every sim entry point isolates the same way.

Teardown is scoped to match. Each launch runs in its own session, so killing
that session reaps stragglers under any node name without reaching another run;
the last-resort `pkill` for an escaped `gz sim` is scoped to this repo's world
path. Two benchmarks — or a benchmark and a smoke test — can therefore run at
once, in the same worktree or different ones.

To watch a running sim in RViz, use `pixi run rviz-sim` — a default-range RViz
cannot see a `LOCALHOST`-only participant (the reverse direction does work).

## Outputs

```
benchmark_results/<UTC timestamp>/
  report.md                     # human-readable per-world tables (mean/std/min/max/CV)
  run.json                      # full run: provenance + every trial's metrics + aggregate
  <world>/trial_<i>/
    series.json                 # raw recorded samples — re-scorable offline via metrics.py
    metrics.json                # this trial's summary
    sim.log, bridge.log         # launch logs (for post-mortem)
```

`report.md` reports the **coefficient of variation (CV = std/mean)** for each
metric so run-to-run variance is explicit when comparing two configs. Provenance
(git commit, world, map revision, nav2_params path, timestamp) is recorded in
both `run.json` and the report so two runs are comparable.

## Observed baseline & variance

From `pixi run bench -- --worlds mote_world.sdf,hospital_world.sdf --trials 2`
(commit `f177473`, sim maps as committed). Two trials per world; the point is
that a fixed config is *reproducible* enough to compare against:

| world | goals ok | ATE rmse (mean ± std) | ATE CV | time-to-goal CV | min clearance |
| --- | --- | --- | --- | --- | --- |
| mote_world | 3/3, 3/3 | 0.068 ± 0.007 m | 10% | 0.8% | 0.95 m |
| hospital_world | 2/3, 1/3 | 0.085 ± 0.004 m | 4% | 4.9% | 0.34 m |

Localization ATE is stable across trials in **both** worlds (CV ≤ 10%), so a
config change that moves ATE by more than a few cm is a real signal, not noise.
Goal-success is the noisier axis: mote completes every goal, while hospital's
long (~30 m) legs bump the default 120 s `--goal-timeout` non-deterministically
(2/3 then 1/3) — localization is fine there, the legs just need more time. For a
fair hospital success rate, raise the cap, e.g.
`--worlds hospital_world.sdf --goal-timeout 300`.

A settle period (`--settle`, default 8 s) before the first goal is load-bearing:
driving the instant TF appears, before AMCL and the costmaps converge, makes the
robot mislocalize and fail every goal. 8 s is enough for these worlds.

## Layout

| file | role | ROS? |
| --- | --- | --- |
| `bench.py` | orchestrator: launch sim + GT bridge, gate readiness, loop trials, tear down, aggregate, write report | processes only |
| `record.py` | one-trial ROS client: drive goals, record series, write `series.json` + `metrics.json` | rclpy |
| `metrics.py` | ATE / goals / clearance / smoothness / aggregation | **no** (numpy only) |
| `report.py` | aggregate trials → markdown + run JSON | no |

Reusing the metrics offline is just `metrics.summarize(series_dict)` on a
`series.json` (or any producer of the same shapes).
