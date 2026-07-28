# Sim benchmark report

- **generated (UTC):** 20260728T232348Z
- **git commit:** `15f6f09`
- **trials per world:** 3
- **goal order:** pickup,dropoff,home
- **wheel_mu:** 1.0
- **nav2 params:** `/home/michael/.claude/jobs/cbff1358/wt-icp-gate/mote_bringup/config/nav2_params.yaml`
- **ROS_DOMAIN_ID:** 18

## mote_world.sdf

- map revision: `20260707T234935`
- map: `/home/michael/Projects/mote/mote_simulation/sim_home/sites/mote_world/floors/ground/map/map.yaml`
- successful trials: 3/3

| metric | mean | std | min | max | CV |
| --- | --- | --- | --- | --- | --- |
| goal success rate | 1.000 | 0.000 | 1.000 | 1.000 | 0.0% |
| time-to-goal mean (s) | 19.2 | 0.2 | 18.9 | 19.5 | 1.3% |
| ATE rmse (m) | 0.069 | 0.005 | 0.063 | 0.075 | 7.4% |
| ATE max (m) | 0.134 | 0.021 | 0.104 | 0.150 | 15.9% |
| odom ATE rmse (m) | 0.007 | 0.001 | 0.006 | 0.008 | 12.7% |
| odom ATE max (m) | 0.020 | 0.008 | 0.011 | 0.029 | 37.6% |
| min clearance (m) | 0.472 | 0.023 | 0.445 | 0.500 | 4.8% |
| mean clearance (m) | 1.327 | 0.025 | 1.296 | 1.357 | 1.9% |
| linear jerk rms | 24.88 | 3.00 | 21.72 | 28.92 | 12.1% |
| direction reversals | 0.0 | 0.0 | 0.0 | 0.0 | — |
| recoveries total | 0.0 | 0.0 | 0.0 | 0.0 | — |
| aborts | 0.0 | 0.0 | 0.0 | 0.0 | — |
| est path length (m) | 9.67 | 0.04 | 9.61 | 9.70 | 0.4% |

<details><summary>per-trial</summary>

| trial | goals ok | ATE rmse (m) | odom ATE (m) | min clr (m) | recoveries | aborts |
| --- | --- | --- | --- | --- | --- | --- |
| 0 | 3/3 | 0.075 | 0.006 | 0.500 | 0 | 0 |
| 1 | 3/3 | 0.069 | 0.006 | 0.472 | 0 | 0 |
| 2 | 3/3 | 0.063 | 0.008 | 0.445 | 0 | 0 |

</details>

## Notes

- Localization error is ATE (truth vs estimate) after a rigid SE(2) alignment — the SLAM `map` frame and the Gazebo world frame do not share a fixed transform, so alignment is required before differencing.
- Ground truth is the robot's true pose bridged from Gazebo's PosePublisher (`/model/mote/pose`, `gz.msgs.Pose` → `PoseStamped`).
- Recovery counts are distinct goal IDs seen on the behavior-server action status topics (`/spin`, `/backup`, `/drive_on_heading`, `/wait`) — a best-effort proxy for how often Nav2 recovered.
- **CV** = coefficient of variation (std/mean); the run-to-run variance to weigh when comparing two configs.
