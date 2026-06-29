# mote_perception

Home for Mote's camera-derived perception nodes. This is **L0 (Foundation)** of
the vision pipeline: package scaffold, a camera health monitor, camera-calibration
plumbing, and confirmation of compressed image transport. Obstacle detection and
ML (L1) come later. This package **runs on the robot** (it will eventually feed
Nav2), so unlike `mote_simulation` it is synced to the Pi.

## Nodes

- `camera_monitor` — subscribes to `image` (remapped to `/image_raw`) and logs
  the measured frame rate, resolution, and encoding every few seconds, warning
  when no frames arrive. Dependency-light (rclpy + sensor_msgs, no OpenCV).

Run the launch:

```bash
pixi run perception
```

## Camera calibration

A committed default calibration (`config/camera_info.default.yaml`) ships as a
fallback, and a per-robot `~/.mote/camera_calibration.yaml` overrides it when
present. Most robots need no calibration of their own — see
[`config/README.md`](config/README.md) for when it's worth doing and the
`camera_calibration cameracalibrator` procedure.

## Compressed transport

`ros-jazzy-image-transport-plugins` is already a project dependency, so the
camera node publishes `/image_raw/compressed` automatically alongside the raw
stream. Off-board consumers (RViz on a workstation, remote viewers) should prefer
the `compressed` transport — raw 640x480 @ 30 FPS is roughly 28 Mbps, which
saturates Wi-Fi.

Note: a `ros2 topic pub` of a fake `sensor_msgs/msg/Image` will **not** produce a
`/image_raw/compressed` topic. Only a real `image_transport` publisher (the
camera) or an `image_transport republish` node emits the compressed variant.

## L1 — Obstacle perception (off-board monocular depth)

Turns the single RGB camera into a `PointCloud2` of obstacles (`/camera_obstacles`)
for a Nav2 voxel/obstacle layer — catching the low/thin things the 2D lidar plane
misses (cables, thresholds, table & chair legs, a robot vacuum). The lidar stays
the primary, low-latency obstacle and clearing source; this is a slower
supplementary **marker**.

Pipeline: Depth Anything V2 (metric, indoor) gives a dense depth map; its raw
metres are not accurate for our lens, so every frame is **metrically rescaled
against the known floor plane** (`depth_rescale.py`, RANSAC affine-in-disparity —
the camera's fixed height/pose is dense per-frame ground truth). The rescaled depth
is back-projected to 3D; points more than `z_obstacle` (default 0.02 m) above the
floor become the cloud, stamped at **image-capture time** so Nav2 places it via tf
at the moment it was seen (this is how the off-board latency is absorbed).

Depth inference is too heavy for the Pi CPU (~0.5 s/frame), so it runs **off-board**
as two processes:

- `tools/depth_server.py` — keeps the model resident and serves depth over a
  socket. Runs in a dedicated pixi environment (kept out of the ROS/robot env on
  purpose):
- `depth_obstacle_node` — light rclpy node (no torch); forwards each compressed
  frame to the server, rescales, and publishes the cloud. Runs anywhere (robot or
  workstation):
- Workstation all-in-one command: starts the depth server and the ROS obstacle
  node together, using the workstation's ROS graph:
  ```bash
  pixi run depth
  ```

Key params: `z_obstacle` (height deadband, default 0.02 m — below ~1.5 cm floor
noise false-positives), `range_min`/`range_max`, `server_host`/`server_port`.

Everything is developed and validated offline against recorded bags:
`tools/depth_obstacles.py` overlays the obstacle decision and compares the cloud
to lidar; other `tools/*.py` are the spike harnesses behind the design.
