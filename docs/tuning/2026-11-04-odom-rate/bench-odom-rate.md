# Sim benchmark report

- **generated (UTC):** 20261104T131651Z
- **git commit:** `cc369e0` plus this branch, uncommitted at the time
- **trials per world:** 2
- **goal order:** pickup,dropoff,home
- **wheel_mu:** 1.0
- **nav2 params:** `mote_bringup/config/nav2_params.yaml`
- **ROS_DOMAIN_ID:** 32

## office_world.sdf

- map revision: `20260708T000623`
- map: `mote_simulation/sim_home/sites/office_world/floors/ground/map/map.yaml`
- successful trials: 2/2

| metric | mean | std | min | max | CV |
| --- | --- | --- | --- | --- | --- |
| goal success rate | 1.000 | 0.000 | 1.000 | 1.000 | 0.0% |
| time-to-goal mean (s) | 85.4 | 1.2 | 84.2 | 86.6 | 1.4% |
| ATE rmse (m) | 0.099 | 0.004 | 0.095 | 0.103 | 4.2% |
| ATE max (m) | 0.246 | 0.024 | 0.222 | 0.270 | 9.7% |
| odom ATE rmse (m) | 0.035 | 0.010 | 0.025 | 0.044 | 27.9% |
| odom ATE max (m) | 0.070 | 0.025 | 0.045 | 0.095 | 35.5% |
| min clearance (m) | 0.206 | 0.013 | 0.194 | 0.219 | 6.1% |
| mean clearance (m) | 0.761 | 0.007 | 0.753 | 0.768 | 0.9% |
| linear jerk rms | 9.70 | 0.26 | 9.43 | 9.96 | 2.7% |
| direction reversals | 0.0 | 0.0 | 0.0 | 0.0 | — |
| recoveries total | 0.0 | 0.0 | 0.0 | 0.0 | — |
| aborts | 0.0 | 0.0 | 0.0 | 0.0 | — |
| est path length (m) | 48.09 | 0.40 | 47.69 | 48.50 | 0.8% |

<details><summary>per-trial</summary>

| trial | goals ok | ATE rmse (m) | odom ATE (m) | min clr (m) | recoveries | aborts |
| --- | --- | --- | --- | --- | --- | --- |
| 0 | 3/3 | 0.095 | 0.025 | 0.194 | 0 | 0 |
| 1 | 3/3 | 0.103 | 0.044 | 0.219 | 0 | 0 |

</details>

## Notes

- Localization error is ATE (truth vs estimate) after a rigid SE(2) alignment — the SLAM `map` frame and the Gazebo world frame do not share a fixed transform, so alignment is required before differencing.
- Ground truth is the robot's true pose bridged from Gazebo's PosePublisher (`/model/mote/pose`, `gz.msgs.Pose` → `PoseStamped`).
- Recovery counts are distinct goal IDs seen on the behavior-server action status topics (`/spin`, `/backup`, `/drive_on_heading`, `/wait`) — a best-effort proxy for how often Nav2 recovered.
- **CV** = coefficient of variation (std/mean); the run-to-run variance to weigh when comparing two configs.
