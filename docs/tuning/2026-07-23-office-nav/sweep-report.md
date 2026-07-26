# Parameter sweep report — office_nav

- **generated (UTC):** 20260723T153307Z
- **git commit:** `6836362`
- **spec:** `/home/michael/Projects/mote/.claude/worktrees/param-sweep/mote_simulation/tools/benchmark/sweep/examples/office_nav.yaml`
- **worlds:** office_world.sdf
- **trials/world:** 2   **goal timeout:** 180s
- **sets evaluated:** 9 of 9

## Scoring

Score is the world-weighted, weighted-metric improvement over the **baseline** (committed defaults), so baseline = 0 and any positive score beats the current config. Weights: success 3.0, localization 1.0, time 1.0, smoothness 0.5. A set must be *feasible* (peak per-wheel speed within the 0.218 m/s hardware wall) and hold goal success at or above baseline to be eligible. To be declared the winner it must beat the **noise floor** (+0.129) by more than a +0.10 margin. The noise floor is the best score of any baseline-*replicate* set (same config as the defaults) — its non-zero score is pure run-to-run variance, so a real improvement must clear it.

## Result: keep the current defaults

No set beat the noise floor (+0.129) by more than the +0.10 win margin, so every apparent improvement is within run-to-run variance. The committed config is the best of those tried (or an improvement was infeasible or cost goal success). Details below.

## Full ranking

| rank | set | score | eligible | success | ATE (m) | time (s) | peak wheel (m/s) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | amcl_max_particles=3000, inflation_radius=0.3, dwb_acc_lim_x=1.0 | +0.189 | yes | 1.000 | 0.090 | 86.1 | 0.222 |
| 2 | amcl_max_particles=2000, inflation_radius=0.35, dwb_acc_lim_x=1.0 | +0.129 | yes | 1.000 | 0.096 | 85.6 | 0.222 |
| 3 | amcl_max_particles=2000, inflation_radius=0.3, dwb_acc_lim_x=1.5 | +0.126 | yes | 1.000 | 0.086 | 85.9 | 0.222 |
| 4 | amcl_max_particles=2000, inflation_radius=0.35, dwb_acc_lim_x=1.5 | +0.117 | yes | 1.000 | 0.087 | 86.9 | 0.222 |
| 5 | amcl_max_particles=2000, inflation_radius=0.3, dwb_acc_lim_x=1.0 | +0.085 | yes | 1.000 | 0.101 | 85.8 | 0.222 |
| 6 | amcl_max_particles=3000, inflation_radius=0.35, dwb_acc_lim_x=1.0 | +0.053 | yes | 1.000 | 0.102 | 87.8 | 0.222 |
| 7 | baseline (baseline) | +0.000 | yes | 1.000 | 0.097 | 85.8 | 0.222 |
| 8 | amcl_max_particles=3000, inflation_radius=0.35, dwb_acc_lim_x=1.5 | -0.735 | **no** | 0.667 | 0.085 | 83.0 | 0.222 |
| 9 | amcl_max_particles=3000, inflation_radius=0.3, dwb_acc_lim_x=1.5 | -1.210 | **no** | 0.500 | 0.084 | 81.3 | 0.222 |

## How to read this

- **success** = goal success rate, **ATE** = localization RMS error, **time** = mean time-to-goal, **jerk** = RMS linear jerk (smoothness).
- **peak wheel** = worst commanded per-wheel speed; a set over the hardware wall is flagged infeasible and cannot win, even if it scores well in sim.
- Per-metric % in the winner table is improvement over baseline (positive = better).
