# Jazzy → Lyrical: what the upgrade forced, 2026-11-03

**Verdict: Lyrical halves the controller rate, and no parameter fixes it.**
On office_world `/cmd_vel_nav` falls from 19.3 Hz (Jazzy, same commit) to
10.0 Hz against a 20 Hz `controller_frequency`. Nav2 1.5.2's RotationShim
looks up a transform at `now()` on every cycle and waits for it; `odom→base`
arrives at scan rate, 10 Hz. Goals still succeed: 6/6 on both distros, 85 s
mean time-to-goal on both. On mote-01 the rate falls from 19.8 to 10.1 Hz in
the same way. The planned fix is ours: publish `odom→base` at wheel rate
(task 715).

No other parameter was retuned. Every configuration change below either
restores a value the upgrade stopped reading, or moves a key to its new name
with the same value. Three defaults moved underneath Mote without a key of
ours changing; they are recorded here and left alone.

Versions: Nav2 1.3.12 → 1.5.2, ros2_control 4.x → 6.10.1, ros2_controllers
4.x → 6.9.0, Gazebo Harmonic (gz-sim 8) → Jetty (gz-sim 10.5).

## How the renames were found

- **Nav2.** `sim mode:=nav`, then every key in `nav2_params.yaml` diffed against
  `ros2 param list` of the node that owns it. A key a node did not declare is a
  key it ignores. Two passes: before the fixes the container crash-looped (no
  list at all); after them, three keys were left undeclared (below).
- **Controllers.** `ros2_control_node` on the xacro output with
  `mock_components/GenericSystem` and the arm enabled, the three controllers
  spawned as `mote_launch.py` spawns them, then the same diff against
  `controllers.yaml` and the joint-params file.
- **Migration notes.** Nav2's *Jazzy to Kilted* and *Kilted to Lyrical* guides,
  and Nav2's source at 1.3.12 and 1.5.2 for the defaults quoted.

## Forced changes: same value, new place

| what | Jazzy | Lyrical | why |
| --- | --- | --- | --- |
| controller parameters | given to `ros2_control_node` | given to each spawner (`--param-file`) | ros2_control 6 does not pass the CM's files on; `wheel_separation` fell to 0 and diff_drive refused it |
| RotationShim primary | `FollowPath.primary_controller: "dwb_core::DWBLocalPlanner"`, DWB keys beside it | `FollowPath.primary_controller.plugin`, DWB keys under `primary_controller` | refused at configure otherwise |
| WheelSpeedLimit injection | `FollowPath.WheelSpeedLimit.*` | `FollowPath.primary_controller.WheelSpeedLimit.*` | follows the DWB namespace |
| bt_navigator error codes | `error_code_names: [compute_path_error_code, follow_path_error_code]` | `error_code_name_prefixes: [compute_path, follow_path]` | `error_code_names` is a startup exception |
| arm trajectory start | `open_loop_control: true` | `interpolate_from_desired_state: true` | renamed; the description is the old one verbatim |
| sim ground truth | PosePublisher default topic `/model/mote/pose` | `<topic>/model/mote/pose</topic>` stated | gz-sim 10 defaults to `/mote/pose` |

The RViz teleop panel (`rviz_teleop_panel`, prefix.dev/mote) has no Lyrical
build. It is removed from the `dev` environment and from `mote.rviz`; the
human teleop inputs are `pixi run teleop` and the Foxglove panel.

Keys deleted because nothing reads them any more:

- DWB `transform_tolerance: 0.2`, `publish_global_plan`, `publish_transformed`.
  `publish_transformed` was not read on Jazzy either.
- waypoint_follower `action_server_result_timeout: 900.0` (Nav2 removed it from
  every action server; rcl now holds results).

## Defaults that moved without a key of ours changing

Recorded, not retuned.

1. **Plan pruning moved from DWB to the controller server.** Nav2 1.5 loads a
   default `FeasiblePathHandler` (we configure none). Jazzy's DWB pruned with
   `prune_distance: 2.0`, `forward_prune_distance: 2.0`,
   `shorten_transformed_plan: true`; the path handler prunes with
   `prune_distance: 2.0` and searches for the robot pose over the costmap's
   extent. Same distance; different code.
2. **Plan transform tolerance 0.2 → 0.3 s.** DWB's own `transform_tolerance:
   0.2` is gone; the path handler takes the local costmap's, which we leave at
   its 0.3 s default. A plan transform now accepts TF up to 0.1 s staler.
3. **Lifecycle bond heartbeat 0.1 → 0.25 s** on every Nav2 lifecycle node.
   `nav2_launch.py` pins `bond_timeout: 10.0`, so detection of a dead server is
   still bounded by the timeout.

RotationShim's `angular_disengage_threshold` (0.39 rad) and `closed_loop`
(true) already had these defaults in 1.3.12, so the shim's behaviour did not
move.

## Undeclared before and after

Two keys are ignored on both distros, so they are not upgrade findings:
`FollowPath.primary_controller.stateful` (DWB never read `stateful`) and
`GridBased.use_astar` (a NavFn key on a Smac 2D planner). Left in place.

## Benchmark against the Jazzy baseline

Raw reports: [`bench-jazzy.md`](2026-11-03-lyrical-upgrade/bench-jazzy.md)
(Jazzy, `e0ab311`, the commit this branch starts from) ·
[`bench-lyrical.md`](2026-11-03-lyrical-upgrade/bench-lyrical.md) (this branch).
`pixi run bench -- --worlds office_world.sdf --trials 2 --goal-timeout 180`.

| metric | Jazzy | Lyrical |
| --- | --- | --- |
| goal success | 6/6 | 6/6 |
| time-to-goal mean (s) | 84.9 ± 1.2 | 85.3 ± 0.3 |
| ATE rmse (m) | 0.103 ± 0.002 | 0.117 ± 0.003 |
| odom ATE rmse (m) | 0.036 ± 0.001 | 0.040 ± 0.001 |
| min clearance (m) | 0.159 ± 0.006 | 0.189 ± 0.002 |
| linear jerk rms | 34.7 ± 8.2 | 3.15 ± 0.01 |
| recoveries / aborts | 0 / 0 | 0 / 0 |

ATE is 0.014 m worse, outside both runs' spread. Its cause is not isolated:
the controller rate, Gazebo Jetty's physics and Nav2 1.5 all changed at once.
Jerk falls tenfold, which is what a command stream at half the rate looks like
to a jerk metric. Neither is retuned.

## The controller rate

Measured with [`run.sh`](2026-11-03-lyrical-upgrade/run.sh) and
[`probe.py`](2026-11-03-lyrical-upgrade/probe.py): `sim mode:=nav`, one goal,
20 s of `/cmd_vel_nav` timed in wall clock (the sim's real-time factor was
1.00). Variants from [`variants.py`](2026-11-03-lyrical-upgrade/variants.py);
raw lines in [`rate-results.txt`](2026-11-03-lyrical-upgrade/rate-results.txt).

| variant | rate | median gap | missed-rate warnings |
| --- | --- | --- | --- |
| Jazzy, committed config | 19.3 Hz | 50 ms | 23 |
| Lyrical, committed config | 10.0 Hz | 97 ms | 202 |
| `costmap_update_timeout: 0.0` | 10.0 Hz | 97 ms | 204 |
| DWB debug publishers off | 10.0 Hz | 96 ms | 204 |
| `rotate_to_heading_once: true` | 10.0 Hz | 98 ms | 203 |
| local costmap `transform_tolerance: 0.05` | 18.9 Hz | 54 ms | 213 |
| `use_path_orientations: true` | 20.0 Hz | 50 ms | 0 |

The controller_server thread used 3.5% of a core at 10 Hz: the loop waits, it
does not compute. `/clock` arrives every 4 ms, so a sim-time sleep is not the
wait. The cause is in `nav2_rotation_shim_controller.cpp` at 1.5.2:

- `computeVelocityCommands` sets `path_updated_ = true` on every call (1.3.12
  set it only when a new plan arrived, about once a second);
- so every cycle `getSampledPathPt` stamps a plan point `clock_->now()` and
  `transformPoseToBaseFrame` looks it up in `base_link`, waiting up to the
  costmap's `transform_tolerance` (0.3 s);
- `odom→base` comes from `IcpOdomGate` once per scan, 10 Hz, so the wait ends
  at the next scan.

The robot's lidar also runs at 10 Hz, and the robot shows the same cap (next
section).

Neither variant that restores the rate is a fix:

- `transform_tolerance: 0.05` makes the lookup time out instead. The shim
  catches the error at DEBUG level and skips its heading check, so rotation
  in place is silently lost on most cycles.
- `use_path_orientations: true` removes the lookup but changes the heading the
  shim aims for. Benchmarked, it failed the dropoff goal in both trials (4/6,
  4 recoveries, 2 reversals):
  [`bench-lyrical-path-orientations.md`](2026-11-03-lyrical-upgrade/bench-lyrical-path-orientations.md).

navigation2#6436 (on `main` since 2026-09-20) stamps the sample with the
robot pose's time instead of `now()`, which should remove the wait. That is a
side effect of a staleness-check refactor: the PR does not mention the rate,
carries no backport label, and was not tested here. The wait exists only
because `odom→base` is slower than the controller. Publishing it at the 50 Hz
wheel rate from `IcpOdomGate`, re-anchored on each gated ICP increment
(task 715), removes the cap on any Nav2 version, and gives every TF reader a
fresher pose. Measured in `2026-11-04-odom-rate.md`: 20 Hz in sim and on
mote-01.

## On the robot

mote-01, 2026-11-04, one run per distro from the same spot at the map origin:
the Lyrical branch, then the Pi restored to `ff59e09` (Jazzy) with its own env
and build. [`run_probe.sh`](2026-11-03-lyrical-upgrade/robot/run_probe.sh)
starts `mote-robot.target` and waits for Nav2;
[`robot_probe.py`](2026-11-03-lyrical-upgrade/robot/robot_probe.py) samples 15 s
at idle, turns in place through `/cmd_vel_teleop_stamped`, then sends a Nav2
goal 1 m ahead and one back to the start. Raw output:
[`probe-jazzy.txt`](2026-11-03-lyrical-upgrade/robot/probe-jazzy.txt),
[`probe-lyrical.txt`](2026-11-03-lyrical-upgrade/robot/probe-lyrical.txt).

| | Jazzy | Lyrical |
| --- | --- | --- |
| `/cmd_vel_nav`, goal out | 19.8 Hz, median 50 ms | 10.1 Hz, median 100 ms |
| `/cmd_vel_nav`, goal back | 16.2 Hz, median 51 ms | 10.1 Hz, median 101 ms |
| goal out / back | succeeded, 6.9 s / 12.3 s | succeeded, 7.8 s / 16.8 s |
| "Control loop missed" lines | 54 | 245 |
| controller_manager periodicity, mean | 50.01 Hz | 50.00 Hz |
| periodicity std dev / min | 0.78 / 43.1 Hz | 1.23 / 26.8 Hz |
| `/joint_states` idle, p95 / max gap | 22 / 31 ms | 22 / 28 ms |
| hardware write cycle, mean | 4.53 ms | 4.42 ms |
| teleop turn, 0.5 rad/s for 2 s | 71.8° | 71.8° |

- The controller-rate cap is the same as in sim: 10 Hz on both legs.
- controller_manager holds 50 Hz on both. Lyrical's slowest cycle is longer
  (26.8 Hz, about 37 ms), and the standard deviation is higher. With one run per distro, that may be
  noise; `/joint_states` gaps do not show it.
- The teleop turn is 2.5 s at 0.5 rad/s (71.6°) on both: the 2 s of commands
  plus the controller's 0.5 s `cmd_vel_timeout`.
- The return leg includes a turn of about 180° and is 4.5 s slower on Lyrical. This is a single
  run.
