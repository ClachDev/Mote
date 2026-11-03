# Sim benchmark report

- **generated (UTC):** 20261103T183034Z
- **git commit:** `8a32837`
- **trials per world:** 2
- **goal order:** pickup,dropoff,home
- **wheel_mu:** 1.0
- **nav2 params:** `mote_bringup/config/nav2_params.yaml` with `FollowPath.use_path_orientations: true` (`MOTE_NAV2_PARAMS_FILE`, variant F of `variants.py`)
- **ROS_DOMAIN_ID:** 28

## office_world.sdf

- map revision: `20260708T000623`
- map: `mote_simulation/sim_home/sites/office_world/floors/ground/map/map.yaml`
- successful trials: 0/2

| metric | mean | std | min | max | CV |
| --- | --- | --- | --- | --- | --- |
| goal success rate | 0.667 | 0.000 | 0.667 | 0.667 | 0.0% |
| time-to-goal mean (s) | 91.5 | 7.2 | 84.4 | 98.7 | 7.8% |
| ATE rmse (m) | 0.073 | 0.005 | 0.068 | 0.079 | 7.0% |
| ATE max (m) | 0.226 | 0.015 | 0.211 | 0.241 | 6.7% |
| odom ATE rmse (m) | 0.022 | 0.005 | 0.016 | 0.027 | 24.9% |
| odom ATE max (m) | 0.041 | 0.007 | 0.035 | 0.048 | 15.8% |
| min clearance (m) | 0.146 | 0.011 | 0.135 | 0.157 | 7.4% |
| mean clearance (m) | 1.002 | 0.018 | 0.984 | 1.019 | 1.8% |
| linear jerk rms | 9.53 | 0.10 | 9.43 | 9.63 | 1.1% |
| direction reversals | 2.0 | 0.0 | 2.0 | 2.0 | 0.0% |
| recoveries total | 4.0 | 1.0 | 3.0 | 5.0 | 25.0% |
| aborts | 1.0 | 0.0 | 1.0 | 1.0 | 0.0% |
| est path length (m) | 30.49 | 0.13 | 30.35 | 30.62 | 0.4% |

<details><summary>per-trial</summary>

| trial | goals ok | ATE rmse (m) | odom ATE (m) | min clr (m) | recoveries | aborts |
| --- | --- | --- | --- | --- | --- | --- |
| 0 | 2/3 | 0.079 | 0.027 | 0.157 | 5 | 1 |
| 1 | 2/3 | 0.068 | 0.016 | 0.135 | 3 | 1 |

</details>

## Notes

- Localization error is ATE (truth vs estimate) after a rigid SE(2) alignment — the SLAM `map` frame and the Gazebo world frame do not share a fixed transform, so alignment is required before differencing.
- Ground truth is the robot's true pose bridged from Gazebo's PosePublisher (`/model/mote/pose`, `gz.msgs.Pose` → `PoseStamped`).
- Recovery counts are distinct goal IDs seen on the behavior-server action status topics (`/spin`, `/backup`, `/drive_on_heading`, `/wait`) — a best-effort proxy for how often Nav2 recovered.
- **CV** = coefficient of variation (std/mean); the run-to-run variance to weigh when comparing two configs.
