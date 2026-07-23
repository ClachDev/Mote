# Sim benchmark harness

Runs scripted nav missions in the headless Gazebo sim and scores them against
**Gazebo ground truth**, so Nav2 / SLAM / localization config changes can be
*proven* with numbers instead of eyeballed in RViz. This is the runner the
planned parameter-sweep tool and the future IMU-justification study build on;
the metric maths lives in a ROS-free module (`metrics.py`) so an offline
bag-replay scorer can reuse it unchanged.

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
| `--out` | `benchmark_results/` | output root (git-ignored) |

## What it measures

Per trial, gated on sim `/clock` (invariant to real-time factor):

- **localization error** — ATE (RMS/mean/median/max) of the estimated
  `map`→`base_footprint` pose vs. the bridged true pose, after a rigid SE(2)
  alignment (the SLAM `map` frame and the Gazebo world frame share no fixed
  transform, so alignment is required before differencing). Raw pre-alignment
  RMSE is also reported.
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

## Layout

| file | role | ROS? |
| --- | --- | --- |
| `bench.py` | orchestrator: launch sim + GT bridge, gate readiness, loop trials, tear down, aggregate, write report | processes only |
| `record.py` | one-trial ROS client: drive goals, record series, write `series.json` + `metrics.json` | rclpy |
| `metrics.py` | ATE / goals / clearance / smoothness / aggregation | **no** (numpy only) |
| `report.py` | aggregate trials → markdown + run JSON | no |

Reusing the metrics offline is just `metrics.summarize(series_dict)` on a
`series.json` (or any producer of the same shapes).
