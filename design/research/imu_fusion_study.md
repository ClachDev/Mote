# Does Mote need an IMU?

*Task 165, July 2026. The BNO085 9-DOF IMU is in `design/BOM.md` — bought,
uninstalled — justified there as wheel-slip detection and improved odometry via
`robot_localization`. This is the decision record for whether to fit it.*

> **Verdict: not currently justified. Leave it uninstalled.**
> Mote already carries two independent motion sources, and real mapping bags show
> the existing pair already provides the signal the IMU was bought for.

---

## The argument

kinematic_icp consumes wheel odometry as a **motion prior**, refines it against
the lidar, and publishes `odom→base`. So the robot has two independent estimates
of its own motion, and — more usefully — **the correction kinematic_icp applies
to the wheel prior is itself a direct measurement of how wrong the wheels were.**
That is the slip signal, already computed every frame on existing hardware.

An IMU would add a third source. It is worth fitting only if the existing two are
insufficient — either because they disagree too often to be trusted, or because
lidar odometry fails badly enough that its correction can't be believed.

## The evidence

Measured with `mote_bringup/tools/odom_health.py` over recorded mapping bags in
`~/.mote/bags/mapping`. No replay is needed: a mapping bag's `/tf` already
carries both `odom→base_footprint` (kinematic_icp) and the inverted
`base_footprint→odom_wheel` relay leaf (wheel odometry).

| bag | duration | ICP vs wheel path | ICP vs wheel yaw | impossible ICP frames |
| --- | --- | --- | --- | --- |
| `20260706_193037` | 180 s driving | 13.82 m vs 13.75 m (−0.4 %) | 2798° vs 2787° (−0.4 %) | 0 |
| `20260706_192836` | 98 s driving | 10.97 m vs 11.16 m (+1.7 %) | 1307° vs 1234° (−5.6 %) | 7 (0.71 %), all 1-frame |
| `20260706_133149` | 1218 s (mostly idle) | — | — | 3 (0.02 %), all 1-frame |

"Impossible" = an ICP pose implying a body speed above the drive's measured
`max_wheel_speed` (0.218 m/s). Wheel slip cannot cause this — slip makes the
*wheels* over-read, never the lidar — so these are scan-match excursions. The
wheels never exceed that ceiling in any bag (0 %).

**kinematic_icp does not systematically fail where Mote drives.** It tracks wheel
odometry to within a couple of percent over runs up to 20 minutes. Because the
quiet baseline is that tight, a genuine slip event would stand out sharply
against it — so the existing two-source residual is a *working* slip detector,
and an IMU is not needed as a tiebreaker.

Two caveats on the numbers:

1. **The yaw residual is confounded.** The tool resamples 100 Hz wheel odom onto
   10 Hz ICP stamps; at 90°/s, 10 ms of stamp skew alone manufactures ~9°/s of
   apparent disagreement. Translation residual is clean (p50 ≈ 0.003 m/s); yaw is
   not. Fix the time sync before setting any yaw threshold.
2. **One environment.** These bags have no glass, no featureless corridors, and
   the long one is mostly stationary. This shows ICP is reliable *where Mote has
   driven*, not everywhere.

## What follows from this

- **Slip/stuck detection** should be built on the existing ICP-vs-wheel residual,
  not on new hardware.
- **Gate ICP output against `max_wheel_speed`.** The rare excursions above are
  single-frame jumps to impossible speeds (up to 1.2 m/s) that land straight in
  `odom→base`. Rejecting them needs no new sensor and no new constant.
- **Revisit the IMU only if lidar odometry is shown to fail** in some environment
  Mote must work in. That is the one regime where both the fusion case and the
  detection case would become real; nothing short of it changes this verdict.

## If it is ever revisited

A sim IMU and a `robot_localization` fusion prototype were built and benchmarked
during this task, then dropped from the merge as unproven. They are preserved at
the git tag **`imu-scaffolding-archive`**.

That benchmark is *not* reported as a result, because it was not a valid test:
injected wheel slip was masked by kinematic_icp's per-frame scan-match, the sim
never degrades the lidar (the only regime where a better prior matters), the EKF
was left untuned, and the deltas were millimetres at n=2. Design notes worth
keeping:

- **Where fusion must go.** kinematic_icp takes the wheel odom only as a TF
  prior, and the fork has **no IMU input** (verified). So the sole clean
  insertion point is an EKF fusing wheel odom + IMU whose output *becomes* that
  prior. It must run `publish_tf: false` — kinematic_icp has to stay the sole
  owner of `odom→base` — with the existing `odom_tf_relay` republishing the fused
  estimate as the `odom_wheel` leaf. A downstream EKF smoothing `odom→base`
  instead was rejected: it fights for TF ownership and leaves the prior itself
  wheel-only, so registration never improves.
- **Bus.** Prefer **UART-RVC** over I2C on the Pi 5. The BNO08x is known for I2C
  clock-stretching the Pi's hardware I2C mishandles; RVC streams fixed 100 Hz
  19-byte frames (yaw/pitch/roll + accel) over a plain 3.3 V UART, and yaw is
  what the EKF would fuse. I2C (`0x4A`/`0x4B` on GPIO2/3) gives the full
  quaternion but needs clock-stretch mitigation. Power is 3.3 V + GND from the
  header, no level shifting.
- **Driver.** No `ros-jazzy-bno08x` exists on robostack/conda-forge. The BNO085's
  on-chip fusion outputs an absolute-orientation quaternion, so no Madgwick
  filter is needed. Best fit is a first-party thin node — the RVC frame format is
  ~50 lines — matching the repo's existing small ROS glue.
- **Config.** `robot.yaml` would need real `imu.link.xyz/rpy` extrinsics (REP-103
  axes; encode a rotated mount in `rpy`), an `imu_link` in the URDF outside the
  `use_sim` guard, and `ros-jazzy-robot-localization` added explicitly to the
  default environment (it is on robostack-jazzy for `linux-aarch64`, currently
  only present transitively).
