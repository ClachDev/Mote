# 2D Lidar Odometry Research

*Researched May 2026. Context: auldbot needs scan-to-scan lidar odometry for wheel slip detection and improved odom fusion.*

---

## Problem Statement

Wheel encoders alone drift and fail under wheel slip (rugs, slippery floors). A lidar-derived odometry source, fused with wheel odom, gives the robot an independent position estimate that doesn't depend on wheel contact. This is a well-solved problem in ROS1 but the ROS2 package landscape is sparse.

---

## Algorithm Landscape

### Range Flow (rf2o) — ICRA 2016
**Paper:** *Planar Odometry from a Radial Laser Scanner. A Range Flow-based Approach.* Jaimez et al.  
**Repo:** https://github.com/MAPIRlab/rf2o_laser_odometry

Avoids explicit point correspondences by treating scan-to-scan matching like dense visual odometry — computes motion from range gradients. Very fast (~0.9ms/scan on a single CPU core). The ROS1 package was the standard answer for 2D lidar odometry for years.

**Status:** Essentially abandoned. The ROS2 port exists but has unmerged bug-fix PRs open for 9+ years. Missing `nav_msgs` dependency in `package.xml`. No covariance published in the odometry message (breaks robot_localization without patching).

**Verdict:** Not recommended. Used as an interim submodule in auldbot pending a better solution.

---

### CSM / laser_scan_matcher — scan_tools
**Paper:** *A planar scan matching algorithm for mobile robots.* Censi, ICRA 2008.  
**Repo:** https://github.com/ros-perception/scan_tools

The original ROS1 standard for 2D scan matching. Uses the C Scan Matcher (CSM) library — an ICP-like method specifically for 2D laser. Powers gmapping and early Cartographer.

**Status:** ROS2 port exists but shares the same maintenance problems as rf2o. Requires CSM as an external C library, complicating builds. Not in robostack-jazzy.

**Verdict:** Historically significant but superseded.

---

### KISS-ICP — PRBonn, RA-L 2023
**Paper:** *KISS-ICP: In Defense of Point-to-Point ICP — Simple, Accurate, and Robust Registration If Done the Right Way.*  
**Repo:** https://github.com/PRBonn/kiss-icp  
**arXiv:** https://arxiv.org/abs/2209.15397

Modern, well-maintained, from Cyrill Stachniss' group at University of Bonn. Uses point-to-point ICP with adaptive thresholding, robust kernel, and voxel downsampling. Runs faster than sensor frame rate. Very actively maintained.

**2D lidar:** Primarily 3D. Can accept 2D LaserScan via `laser_geometry` conversion (z=0 for all points) but this is not the primary use case — voxelisation and adaptive thresholding are tuned for 3D density. Performance on sparse 2D rings is degraded, especially in featureless indoor environments.

**Verdict:** Excellent for 3D lidar. Not ideal for 2D.

---

### Kinematic-ICP — PRBonn, ICRA 2025 ⭐ Recommended
**Paper:** *Kinematic-ICP: Enhancing LiDAR Odometry with Kinematic Constraints for Wheeled Mobile Robots Moving on Planar Surfaces.*  
**Repo:** https://github.com/PRBonn/kinematic-icp  
**arXiv:** https://arxiv.org/abs/2410.10277

From the same PRBonn group as KISS-ICP. Explicitly designed for wheeled robots on flat floors (warehouses, offices, homes). Integrates wheel odometry directly into the ICP optimisation as a kinematic constraint rather than fusing two separate odometry estimates after the fact.

**How it works:**
- Uses wheel odom as the initial guess for ICP
- Constrains the ICP solution to be kinematically reachable by a unicycle model (Δx, Δθ only — no lateral slip, no vertical motion)
- Computes adaptive weighting parameter βₜ: measures consistency between wheel odom and lidar at the initial guess. High disagreement → trust lidar more. High agreement → trust encoders more.
- Single joint optimisation instead of scan matching + EKF fusion pipeline

**2D lidar support:** Yes, natively. Set `use_2d_lidar:=true` — accepts `sensor_msgs/LaserScan` directly.

**Wheel slip:** Not explicitly modelled, but implicitly handled. When wheels slip, the wheel odom and lidar disagree → βₜ increases → lidar takes over. Recovery is automatic.

**Performance vs alternatives (warehouse sequences):**

| Method | RPE |
|--------|-----|
| Kinematic-ICP | **0.39%** |
| Wheel odom + 2D KISS-ICP (EKF) | 1.48% |
| Fuse framework | similar to EKF |

Runs at **100 Hz** vs robot_localization/fuse at ~10 Hz.

**ROS2 support:** Humble, Iron, Jazzy. Not in robostack-jazzy yet (submodule required). v0.1.1 released January 2025. Actively maintained.

**Inputs required:**
- `sensor_msgs/LaserScan` or `sensor_msgs/PointCloud2`
- Wheel odom TF: `odom → base_link`
- Static TF: `base_link → lidar_frame`

**Output:** TF `base_link → odom_lidar` (note: new frame name, not `odom` — slam_toolbox needs `odom_frame: odom_lidar`)

**Limitations:**
- Assumes planar surface (breaks on stairs, rough terrain — not relevant for home robot)
- Does not handle wheel slip on strongly sloped surfaces
- Requires wheel odometry (not purely lidar-only)

**Verdict:** The right choice for auldbot. Replaces both rf2o AND robot_localization with a single tighter system.

---

### MOLA — IJRR 2025
**Paper:** *A flexible framework for accurate LiDAR odometry, map manipulation, and localization.* Blanco-Claraco.  
**Repo:** https://github.com/MOLAorg/mola  
**ROS Index (Jazzy):** https://docs.ros.org/en/ros2_packages/jazzy/api/mola_lidar_odometry/__README.html

A modular C++ framework for SLAM. Has both a lidar odometry node and a full 2D SLAM system (`mola_mapper_2d` with GTSAM pose graph). Already has Jazzy binaries on the ROS index (amd64 and arm64). Impressive engineering.

**Verdict:** Full SLAM framework — overkill as a drop-in odometry node. Worth revisiting if slam_toolbox is ever replaced.

---

## Recommendation for auldbot

Replace the current rf2o submodule + robot_localization stack with **Kinematic-ICP** as a single submodule. Configuration changes needed:

1. `slam_toolbox_params.yaml`: change `odom_frame: odom` → `odom_frame: odom_lidar`
2. `controllers.yaml`: re-enable `enable_odom_tf: true` (Kinematic-ICP reads the raw wheel odom TF)
3. Remove `robot_localization` from `pixi.toml` and `localization_launch.py`
4. Add Kinematic-ICP as submodule, configure with `use_2d_lidar: true`

### Robostack contribution

Kinematic-ICP is a better robostack contribution target than rf2o:
- Active PRBonn maintainers would likely support the effort
- ICRA 2025 paper drives community interest
- Higher package quality
- More impactful for the ROS community

The conda recipe structure would be similar to the rf2o plan, targeting `linux-64` and `linux-aarch64`.

---

## Sources

- [Kinematic-ICP arXiv](https://arxiv.org/abs/2410.10277)
- [Kinematic-ICP GitHub](https://github.com/PRBonn/kinematic-icp)
- [KISS-ICP arXiv](https://arxiv.org/abs/2209.15397)
- [KISS-ICP GitHub](https://github.com/PRBonn/kiss-icp)
- [LiDAR Odometry Survey arXiv:2312.17487](https://arxiv.org/abs/2312.17487)
- [MOLA LiDAR Odometry ROS Index](https://docs.ros.org/en/ros2_packages/jazzy/api/mola_lidar_odometry/__README.html)
- [MOLA mapper 2D GitHub](https://github.com/MOLAorg/mola_mapper_2d)
- [MOLA IJRR 2025](https://journals.sagepub.com/doi/abs/10.1177/02783649251316881)
- [rf2o GitHub](https://github.com/MAPIRlab/rf2o_laser_odometry)
- [rf2o ICRA 2016 PDF](https://cvg.cit.tum.de/_media/spezial/bib/jaimez2016icra.pdf)
