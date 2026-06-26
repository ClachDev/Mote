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
pixi run -- ros2 launch mote_perception perception_launch.py
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
