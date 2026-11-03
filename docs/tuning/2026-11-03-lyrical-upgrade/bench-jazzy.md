# Sim benchmark report

- **generated (UTC):** 20261103T181855Z
- **git commit:** `e0ab311`
- **trials per world:** 2
- **goal order:** pickup,dropoff,home
- **wheel_mu:** 1.0
- **nav2 params:** `mote_bringup/config/nav2_params.yaml`
- **ROS_DOMAIN_ID:** 34

## office_world.sdf

- map revision: `20260708T000623`
- map: `mote_simulation/sim_home/sites/office_world/floors/ground/map/map.yaml`
- successful trials: 2/2

| metric | mean | std | min | max | CV |
| --- | --- | --- | --- | --- | --- |
| goal success rate | 1.000 | 0.000 | 1.000 | 1.000 | 0.0% |
| time-to-goal mean (s) | 84.9 | 1.2 | 83.7 | 86.1 | 1.4% |
| ATE rmse (m) | 0.103 | 0.002 | 0.101 | 0.105 | 2.1% |
| ATE max (m) | 0.262 | 0.041 | 0.221 | 0.304 | 15.8% |
| odom ATE rmse (m) | 0.036 | 0.001 | 0.035 | 0.037 | 2.4% |
| odom ATE max (m) | 0.106 | 0.003 | 0.103 | 0.109 | 2.5% |
| min clearance (m) | 0.159 | 0.006 | 0.152 | 0.165 | 4.1% |
| mean clearance (m) | 0.736 | 0.008 | 0.728 | 0.744 | 1.1% |
| linear jerk rms | 34.66 | 8.21 | 26.45 | 42.87 | 23.7% |
| direction reversals | 0.0 | 0.0 | 0.0 | 0.0 | — |
| recoveries total | 0.0 | 0.0 | 0.0 | 0.0 | — |
| aborts | 0.0 | 0.0 | 0.0 | 0.0 | — |
| est path length (m) | 48.58 | 0.28 | 48.31 | 48.86 | 0.6% |

<details><summary>per-trial</summary>

| trial | goals ok | ATE rmse (m) | odom ATE (m) | min clr (m) | recoveries | aborts |
| --- | --- | --- | --- | --- | --- | --- |
| 0 | 3/3 | 0.101 | 0.035 | 0.152 | 0 | 0 |
| 1 | 3/3 | 0.105 | 0.037 | 0.165 | 0 | 0 |

</details>

## Notes

- Localization error is ATE (truth vs estimate) after a rigid SE(2) alignment — the SLAM `map` frame and the Gazebo world frame do not share a fixed transform, so alignment is required before differencing.
- Ground truth is the robot's true pose bridged from Gazebo's PosePublisher (`/model/mote/pose`, `gz.msgs.Pose` → `PoseStamped`).
- Recovery counts are distinct goal IDs seen on the behavior-server action status topics (`/spin`, `/backup`, `/drive_on_heading`, `/wait`) — a best-effort proxy for how often Nav2 recovered.
- **CV** = coefficient of variation (std/mean); the run-to-run variance to weigh when comparing two configs.
