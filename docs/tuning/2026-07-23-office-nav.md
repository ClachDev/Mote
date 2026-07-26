# Tuning run — Nav2 on office_world, 2026-07-23

**Verdict: keep the committed defaults.** No swept configuration beat the current
`nav2_params.yaml` by more than run-to-run noise, so nothing was changed. This
doc is the evidence for that decision.

Run with the [parameter-sweep tool](../../mote_simulation/tools/benchmark/sweep/README.md):

```bash
pixi run bench-sweep mote_simulation/tools/benchmark/sweep/examples/office_nav.yaml
```

- **world:** `office_world.sdf` (medium corridor)
- **trials/set:** 2 · **goal timeout:** 180 s · **goal cycle:** pickup→dropoff→home
- **grid:** 2×2×2 = 8 sets + all-defaults baseline (9 total)
- **spec:** [`sweep/examples/office_nav.yaml`](../../mote_simulation/tools/benchmark/sweep/examples/office_nav.yaml)
- raw outputs: [`sweep-report.md`](2026-07-23-office-nav/sweep-report.md) ·
  [`ranking.json`](2026-07-23-office-nav/ranking.json)

## Parameters swept

| parameter | file · key path | default | values tried |
| --- | --- | --- | --- |
| AMCL max particles | `nav2` · `amcl.ros__parameters.max_particles` | 2000 | 2000, **3000** |
| costmap inflation radius (local+global) | `nav2` · `*_costmap.*.inflation_layer.inflation_radius` | 0.35 | **0.30**, 0.35 |
| DWB forward accel limit | `nav2` · `controller_server.…FollowPath.acc_lim_x` | 1.0 | 1.0, **1.5** |

## Result

Every set was scored against the baseline (committed defaults = 0) on a weighted
blend of goal success (×3), localization ATE (×1), time-to-goal (×1), and motion
smoothness (×0.5). Full table in [`sweep-report.md`](2026-07-23-office-nav/sweep-report.md);
the essentials:

| set | score | success | ATE (m) | note |
| --- | --- | --- | --- | --- |
| amcl=3000, infl=0.30, acc=1.0 | **+0.189** | 3/3, 3/3 | 0.090 | top score, but see below |
| amcl=2000, infl=0.35, acc=1.0 | +0.129 | 3/3, 3/3 | 0.096 | **== defaults (replicate)** |
| amcl=2000, infl=0.30, acc=1.5 | +0.126 | 3/3, 3/3 | 0.086 | |
| amcl=2000, infl=0.35, acc=1.5 | +0.117 | 3/3, 3/3 | 0.087 | |
| amcl=2000, infl=0.30, acc=1.0 | +0.085 | 3/3, 3/3 | 0.101 | |
| amcl=3000, infl=0.35, acc=1.0 | +0.053 | 3/3, 3/3 | 0.102 | |
| **baseline** | 0.000 | 3/3, 3/3 | 0.097 | reference |
| amcl=3000, infl=0.35, acc=1.5 | −0.735 | 3/3, **1/3** | — | ineligible (goal failures) |
| amcl=3000, infl=0.30, acc=1.5 | −1.210 | 2/3, **1/3** | — | ineligible (goal failures) |

### Why keep the defaults

The grid happens to contain a set — `amcl=2000, infl=0.35, acc=1.0` — that *is*
the committed defaults (a **replicate**). It scored **+0.129**, not 0, purely
from run-to-run variance. That is the noise floor. The top set (+0.189) beats it
by only ~0.06, well inside the noise, and the sweep's winner gate (must clear the
floor by a 0.10 margin) correctly declines to call it a win.

The apparent leader, `amcl=3000, infl=0.30`, is also the least trustworthy
direction: raising particles to 3000 made the *other* two amcl=3000 sets
(with `acc_lim_x=1.5`) **fail goals** (1/3 and 1/3–2/3 success) — repeatably,
including on an idle machine. Localization ATE was stable everywhere (≈0.08–0.10 m);
the differences between eligible sets are noise, not signal. So there is no
defensible config change here.

Concretely useful facts this run *did* establish:

- **inflation_radius 0.30 is safe on office_world** (3/3 at amcl=2000) — an
  earlier, invalid run had suggested otherwise; that was a harness bug, not the
  config (below).
- The **WheelSpeedLimit critic holds**: peak commanded per-wheel speed was
  0.222 m/s across every set — right at the 0.218 m/s hardware wall (within the
  sweep's 5 % feasibility tolerance), never runaway.
- office_world legs are long (~85 s mean time-to-goal; the dropoff return leg
  alone is ~125 s), so `--goal-timeout` needs to stay ≥180 s here.

## Harness fixes made while running this (see the branch history)

The sweep is the first thing to run many benchmarks back-to-back, and it exposed
two defects that had to be fixed before the numbers above could be trusted:

1. **Nav2 node leak in `bench.py` teardown.** Teardown only SIGTERM'd the launch
   group and name-killed gz/bridge/controller_manager, so slow-exiting Nav2
   lifecycle nodes (controller_server, amcl, planner_server, …) piled up across
   sets until bringup timed out and *every post-baseline set failed 0/3*. Fixed
   by force-killing the whole process group (SIGTERM→SIGKILL), scoped to our own
   launches.
2. **DDS graph collision.** A sim in another worktree on the default
   `ROS_DOMAIN_ID` polluted the graph and silently failed goals. The sweep now
   runs the benchmark on a dedicated domain (default 42, `--ros-domain-id`).

The interruption-resilient `--resume` and the replicate-derived noise floor also
came out of this run.

## Reproduce

```bash
# ~1.5–2 h; one gz instance at a time, needs a GPU workstation
pixi run bench-sweep mote_simulation/tools/benchmark/sweep/examples/office_nav.yaml
# preview the plan without launching a sim:
pixi run bench-sweep mote_simulation/tools/benchmark/sweep/examples/office_nav.yaml --dry-run
```
