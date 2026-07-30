# Sim benchmark report

- **generated (UTC):** 20260728T231142Z
- **git commit:** `15f6f09`
- **trials per world:** 3
- **goal order:** pickup,dropoff,home
- **wheel_mu:** 1.0
- **nav2 params:** `/home/michael/.claude/jobs/cbff1358/wt-baseline/mote_bringup/config/nav2_params.yaml`
- **ROS_DOMAIN_ID:** 67

## mote_world.sdf

- map revision: `20260707T234935`
- map: `/home/michael/Projects/mote/mote_simulation/sim_home/sites/mote_world/floors/ground/map/map.yaml`
- successful trials: 3/3

| metric | mean | std | min | max | CV |
| --- | --- | --- | --- | --- | --- |
| goal success rate | 1.000 | 0.000 | 1.000 | 1.000 | 0.0% |
| time-to-goal mean (s) | 22.4 | 4.1 | 19.1 | 28.2 | 18.4% |
| ATE rmse (m) | 0.436 | 0.521 | 0.064 | 1.173 | 119.5% |
| ATE max (m) | 0.869 | 1.028 | 0.138 | 2.323 | 118.4% |
| odom ATE rmse (m) | 0.009 | 0.002 | 0.007 | 0.012 | 25.7% |
| odom ATE max (m) | 0.026 | 0.005 | 0.020 | 0.033 | 20.1% |
| min clearance (m) | 0.456 | 0.010 | 0.448 | 0.470 | 2.2% |
| mean clearance (m) | 1.267 | 0.098 | 1.129 | 1.340 | 7.7% |
| linear jerk rms | 27.91 | 3.79 | 23.33 | 32.61 | 13.6% |
| direction reversals | 0.0 | 0.0 | 0.0 | 0.0 | — |
| recoveries total | 0.0 | 0.0 | 0.0 | 0.0 | — |
| aborts | 0.0 | 0.0 | 0.0 | 0.0 | — |
| est path length (m) | 12.10 | 3.57 | 9.56 | 17.14 | 29.5% |

<details><summary>per-trial</summary>

| trial | goals ok | ATE rmse (m) | odom ATE (m) | min clr (m) | recoveries | aborts |
| --- | --- | --- | --- | --- | --- | --- |
| 0 | 3/3 | 0.072 | 0.008 | 0.450 | 0 | 0 |
| 1 | 3/3 | 0.064 | 0.007 | 0.470 | 0 | 0 |
| 2 | 3/3 | 1.173 | 0.012 | 0.448 | 0 | 0 |

</details>

## Notes

- Localization error is ATE (truth vs estimate) after a rigid SE(2) alignment — the SLAM `map` frame and the Gazebo world frame do not share a fixed transform, so alignment is required before differencing.
- Ground truth is the robot's true pose bridged from Gazebo's PosePublisher (`/model/mote/pose`, `gz.msgs.Pose` → `PoseStamped`).
- Recovery counts are distinct goal IDs seen on the behavior-server action status topics (`/spin`, `/backup`, `/drive_on_heading`, `/wait`) — a best-effort proxy for how often Nav2 recovered.
- **CV** = coefficient of variation (std/mean); the run-to-run variance to weigh when comparing two configs.
