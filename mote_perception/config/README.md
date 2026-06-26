# Camera calibration

This directory holds the camera intrinsics (`camera_info.yaml`) for the robot's
USB webcam. The real file can only be produced on hardware with the physical
camera and a printed checkerboard — it is **not** checked in, and `robot.yaml`'s
`camera.info_url` is left unset by default so the camera runs uncalibrated until
you generate one.

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

## Generating `camera_info.yaml` on the robot

1. Print and mount the target above.
2. Start the camera on the robot (`pixi run launch`, or just the camera node).
3. Run the interactive calibrator against the live stream:

   ```bash
   pixi run -- ros2 run camera_calibration cameracalibrator \
     --size 6x9 --square 0.025 \
     --ros-args -r image:=/image_raw -p camera:=/camera
   ```

   (`--size` is **inner corners**, not squares — the 7x10-square board is `6x9`.)

4. Move the board through the frame until the X/Y/Size/Skew bars are full, press
   **CALIBRATE**, then **SAVE**. The tool writes a tarball to `/tmp`; extract its
   `ost.yaml`, rename it to `camera_info.yaml`, and drop it into this `config/`
   directory, then `pixi run build` so it is installed to the package share.

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
