# Sim benchmark report

- **generated (UTC):** 20261103T180315Z
- **git commit:** `07f62c5` plus the runtime fixes committed as `8a32837`
- **trials per world:** 2
- **goal order:** pickup,dropoff,home
- **wheel_mu:** 1.0
- **nav2 params:** `mote_bringup/config/nav2_params.yaml`
- **ROS_DOMAIN_ID:** 28

## office_world.sdf

- map revision: `20260708T000623`
- map: `mote_simulation/sim_home/sites/office_world/floors/ground/map/map.yaml`
- successful trials: 2/2

| metric | mean | std | min | max | CV |
| --- | --- | --- | --- | --- | --- |
| goal success rate | 1.000 | 0.000 | 1.000 | 1.000 | 0.0% |
| time-to-goal mean (s) | 85.3 | 0.3 | 85.0 | 85.6 | 0.4% |
| ATE rmse (m) | 0.117 | 0.003 | 0.113 | 0.120 | 3.0% |
| ATE max (m) | 0.335 | 0.040 | 0.295 | 0.374 | 11.8% |
| odom ATE rmse (m) | 0.040 | 0.001 | 0.039 | 0.041 | 2.3% |
| odom ATE max (m) | 0.095 | 0.001 | 0.094 | 0.096 | 0.9% |
| min clearance (m) | 0.189 | 0.002 | 0.187 | 0.192 | 1.1% |
| mean clearance (m) | 0.790 | 0.000 | 0.789 | 0.790 | 0.0% |
| linear jerk rms | 3.15 | 0.01 | 3.13 | 3.16 | 0.4% |
| direction reversals | 0.0 | 0.0 | 0.0 | 0.0 | — |
| recoveries total | 0.0 | 0.0 | 0.0 | 0.0 | — |
| aborts | 0.0 | 0.0 | 0.0 | 0.0 | — |
| est path length (m) | 48.45 | 0.35 | 48.10 | 48.80 | 0.7% |

<details><summary>per-trial</summary>

| trial | goals ok | ATE rmse (m) | odom ATE (m) | min clr (m) | recoveries | aborts |
| --- | --- | --- | --- | --- | --- | --- |
| 0 | 3/3 | 0.120 | 0.041 | 0.192 | 0 | 0 |
| 1 | 3/3 | 0.113 | 0.039 | 0.187 | 0 | 0 |

</details>

## Notes

- Localization error is ATE (truth vs estimate) after a rigid SE(2) alignment — the SLAM `map` frame and the Gazebo world frame do not share a fixed transform, so alignment is required before differencing.
- Ground truth is the robot's true pose bridged from Gazebo's PosePublisher (`/model/mote/pose`, `gz.msgs.Pose` → `PoseStamped`).
- Recovery counts are distinct goal IDs seen on the behavior-server action status topics (`/spin`, `/backup`, `/drive_on_heading`, `/wait`) — a best-effort proxy for how often Nav2 recovered.
- **CV** = coefficient of variation (std/mean); the run-to-run variance to weigh when comparing two configs.
