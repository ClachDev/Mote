# Parameter sweep

Tunes Mote's nav/slam config **methodically** on top of the [sim benchmark
harness](../README.md): runs the benchmark once per parameter set, scores each
set against the committed defaults, ranks them, and emits a provenance report so
a config change ships with evidence instead of a hunch.

```bash
pixi run bench-sweep mote_simulation/tools/benchmark/sweep/examples/office_nav.yaml
pixi run bench-sweep <spec.yaml> --dry-run      # print the plan, launch nothing
pixi run bench-sweep <spec.yaml> --max-sets 4   # baseline + first 3 sets only
```

Needs a GPU workstation (headless gz-sim). Sweeps run **sequentially** — one
gz-sim instance at a time — so a full grid is `(1 + grid points) × trials × worlds`
benchmark runs and can take a while; start with `--dry-run`, then `--max-sets`
to smoke it, then the full run.

## How it works

1. **Expand** the grid: the cartesian product of every parameter's values, with
   the all-defaults **baseline** run first (index 0) as the reference.
2. **Override at launch time, never on disk.** For each set, the swept values are
   merged onto a *copy* of the committed config into `set_<i>/params/<target>_params.yaml`,
   and the matching `MOTE_*_PARAMS_FILE` variable is exported before the benchmark
   launches. The launch files read the override via `mote_bringup.param_overrides`;
   the committed `nav2_params.yaml` / `slam_toolbox_params.yaml` / `controllers.yaml`
   are never touched. Unset outside a sweep, the seam is inert.
3. **Score** each set against the baseline and **rank** (see below).
4. **Report**: `report.md` names the winner, its changed parameters (default vs
   new), and the metric deltas across the worlds; `ranking.json` has everything.

## Spec format

```yaml
name: office_nav            # optional; labels the report

benchmark:                  # forwarded to bench.py
  worlds: [office_world.sdf]
  trials: 2
  goal_timeout: 180
  order: pickup,dropoff,home
  settle: 8

parameters:
  - name: amcl_max_particles          # optional label; defaults to the leaf key
    file: nav2                        # target: nav2 | slam | controllers
    path: amcl.ros__parameters.max_particles
    values: [2000, 3000]

  - name: inflation_radius
    file: nav2
    paths:                            # one value applied to several key paths
      - [local_costmap, local_costmap, ros__parameters, inflation_layer, inflation_radius]
      - [global_costmap, global_costmap, ros__parameters, inflation_layer, inflation_radius]
    values: [0.30, 0.35]

  - name: dwb_acc_lim_x
    file: nav2
    path: controller_server.ros__parameters.FollowPath.acc_lim_x
    range: {start: 1.0, stop: 2.0, step: 0.5}   # inclusive; or `values:`

scoring:                    # optional; score.py documents the defaults
  weights: {success: 3.0, localization: 1.0, time: 1.0, smoothness: 0.5}
  world_weights: {office_world.sdf: 1.0}
```

### Adding a parameter to a sweep

Append an entry under `parameters`:

- **`file`** — which config the parameter lives in: `nav2`
  (`mote_bringup/config/nav2_params.yaml`), `slam`
  (`slam_toolbox_params.yaml`), or `controllers` (`controllers.yaml`). These are
  the files wired to the launch-time override seam
  (`mote_bringup/mote_bringup/param_overrides.py`); to make a *new* config file
  sweepable, add it to `ENV_VARS` there and to `BASE_FILES`/`ENV_VARS` in
  `overrides.py`, and have its launch consult `param_overrides.override_path`.
- **`path`** (one key path) or **`paths`** (several sharing the value) — the
  location of the leaf inside that YAML. A dotted string (`a.b.c`) is split on
  dots; use the **list form** (`[a, b, c]`) when a key itself contains a dot —
  Nav2 writes some plugin params as literal dotted keys (e.g. `WheelSpeedLimit.scale`
  is a literal key, whereas `FollowPath.max_vel_x` is nested).
- **`values`** (explicit list) or **`range`** (`{start, stop, step}`, inclusive).

The path must already exist in the committed file (a typo raises, rather than
silently adding a dead key). Slam parameters are accepted by the spec, but note
the current benchmark scores **nav** missions, so a `slam` sweep only moves the
score once a mapping-mode benchmark exists — until then sweep nav2/controllers.

## Scoring & ranking

Score is the world-weighted, weighted-metric **improvement over the baseline**,
so the baseline scores 0 and any positive score beats the committed config. Four
metrics from each world's aggregate (mean across its trials):

| metric | source | direction | default weight |
| --- | --- | --- | --- |
| success | goal success rate | higher better | 3.0 |
| localization | ATE rmse (m) | lower better | 1.0 |
| time | mean time-to-goal (s) | lower better | 1.0 |
| smoothness | RMS linear jerk | lower better | 0.5 |

Per metric the relative improvement is `(candidate − baseline)/|baseline|`
(sign flipped for lower-is-better); the per-world score is the weighted sum, the
total is the world-weighted mean. Success is weighted highest so a set can never
win by trading goal completions for speed.

**Two hard gates** decide eligibility to win, independent of score:

- **Feasibility** — the sim will command wheel speeds the real STS3215 servos
  can't reach, so for every recorded `cmd` sample the sweep computes the peak
  per-wheel speed `|v| + |ω|·wheel_separation/2` and disqualifies any set that
  exceeds the hardware wall (`robot.yaml` `max_wheel_speed` = 0.218 m/s) by more
  than a small tolerance. A config that looks fast in sim but is undriveable can
  never win.
- **Success floor** — the winner's mean success rate must be at or above the
  baseline's (minus a small tolerance).

A set is only declared the **winner** if it is eligible *and* beats the **noise
floor** by more than `score.WIN_MARGIN`. The noise floor is the best score of any
baseline-*replicate* set — a swept set whose values all happen to equal the
committed defaults, so its non-zero score is pure run-to-run variance. Requiring a
winner to clear that floor stops the sweep from "improving" the config by noise.
A cartesian grid gives a replicate for free whenever the defaults are among the
swept values (the `office_nav` example has one); if a grid has none, the floor is
0 and the margin alone guards, so include a replicate point when you can. A
replicate can never itself win. Otherwise the report says "keep the current
defaults".

Weights and per-world weights are overridable in the spec's `scoring` block; the
defaults live in `score.py`. The benchmark graph runs on a dedicated
`ROS_DOMAIN_ID` (default 42, `--ros-domain-id`) so a sim in another worktree or a
robot on the network can't pollute it — `bench.py` honours that inherited domain
instead of claiming one per set, so every set in a sweep is scored on the same
graph configuration.

## Outputs

```
sweep_results/<UTC>/
  report.md          winner: changed params (default vs new) + metric deltas
  ranking.json       every set: assignments, metrics, feasibility, score, rank
  set_<i>/params/    the merged config files applied for that set
  set_<i>/<ts>/      that set's benchmark run dir (bench.py's report.md/run.json)
```

`--dry-run` writes `plan.md` / `plan.json` and the merged params instead, so a
spec can be checked without a sim. `--resume <run_dir>` reuses the completed sets
already in a sweep dir and runs only the rest, so a long sweep survives a restart
(an interrupted set — no aggregated trials in its `run.json` — is re-run, not
trusted).

## Layout

| file | role |
| --- | --- |
| `sweep.py` | CLI/runner: expand grid, run bench per set, score, write report |
| `spec.py` | spec parse + validation + grid expansion (ROS-free) |
| `overrides.py` | merge swept values onto a config copy → temp files (ROS-free) |
| `score.py` | scoring function, feasibility gate, ranking (ROS-free) |
| `sweep_report.py` | ranking + winning-set provenance markdown (ROS-free) |
| `test_sweep.py` | unit tests for the above (`python test_sweep.py`, no sim) |
