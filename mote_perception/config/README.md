# Camera calibration

This directory holds the camera intrinsics (`camera_info.yaml`) for the robot's
USB webcam, produced by the calibration below and committed so it deploys to the
Pi via `pixi run sync`. `robot.yaml`'s `camera.info_url` points at it. Regenerate
it (and re-commit) if the camera is swapped.

## Print the target

Print [`checkerboard_a4_25mm_7x10.pdf`](checkerboard_a4_25mm_7x10.pdf) on A4 — it
is **7x10 squares of 25 mm** (so **6x9 inner corners**), with the calibrator
parameters printed along the bottom edge.

The page is deliberately a little smaller than A4 so it fits inside the printable
area of any A4 printer: a "shrink to printable area" setting then has nothing to
shrink, and the squares come out true-size **without changing any print
settings**. Just print it on A4 and the board is centred with a white border.

- **Mount it dead flat** on a clipboard, foamboard, or glass. A floppy sheet
  warps and wrecks the result.
- You do **not** need to measure the squares. If your printer does scale it
  slightly anyway, it still calibrates correctly: the camera **intrinsics** that
  land in `camera_info.yaml` are independent of the absolute square size — square
  size only affects metric scale, which this calibration does not rely on. So
  just print and use the `--square 0.025` printed on the board.

## Generating `camera_info.yaml`

`cameracalibrator` is an interactive GUI tool, so run it on the **workstation**
(it is a `dev` dependency, not installed on the robot) against the Pi's live
camera over the ROS 2 network. Both machines must be on the same LAN and
`ROS_DOMAIN_ID`.

1. Print and mount the target above.
2. On the **Pi**, start the camera so it publishes `/image_raw` and
   `/camera_info` (`pixi run launch`, or just the `v4l2_camera` node).
3. On the **workstation**, confirm the topics are visible across the network
   (`pixi run -e dev -- ros2 topic list` should list `/image_raw`), then run the
   calibrator:

   ```bash
   pixi run -e dev -- ros2 run camera_calibration cameracalibrator \
     --size 6x9 --square 0.025 \
     --ros-args -r image:=/image_raw -p camera:=/camera
   ```

   (`--size` is **inner corners**, not squares — the 7x10-square board is `6x9`.
   If the stream is laggy over WiFi, add `-p image_transport:=compressed`.)

4. Move the board through the frame until the X/Y/Size/Skew bars are full, then
   press **CALIBRATE**. It prints the result (`camera matrix`, `distortion`,
   `rectification`, `projection`) to the console in oST format.

   > The **SAVE**/**COMMIT** buttons crash in this package version
   > (`camera_calibration` 5.0.11 calls `numpy.ndarray.tostring()`, removed in
   > NumPy 2.0). Ignore it — copy the printed parameters straight into
   > `camera_info.yaml` instead, using the existing file as the template
   > (camera_matrix → `camera_matrix.data`, distortion → `distortion_coefficients.data`,
   > projection → `projection_matrix.data`). Then `pixi run build`.

## Wiring it in

Point the camera at the calibration by adding an `info_url` to the `camera:`
section of `mote_description/config/robot.yaml`:

```yaml
camera:
  device: /dev/mote_camera
  image_size: [640, 480]
  info_url: package://mote_perception/config/camera_info.yaml
```

When `info_url` is present, `mote_launch.py` passes it to `v4l2_camera_node` as
the `camera_info_url` parameter and the camera publishes correct
`/camera_info`. When it is absent the launch behaves exactly as before.
