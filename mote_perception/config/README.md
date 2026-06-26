# Camera calibration

The camera intrinsics tell ROS the lens geometry so it can undistort images and
project pixels to rays. Mote resolves them at launch with a fallback:

1. **`~/.mote/camera_calibration.yaml`** — this robot's own calibration, if it
   exists. Per-robot and intentionally **outside the repo**.
2. **`camera_info.default.yaml`** (this directory) — a representative default for
   the UGREEN webcam, committed so a fresh checkout / new robot works out of the
   box.

`mote_launch.py` uses the `~/.mote` file when present, otherwise the packaged
default (`robot.yaml`'s `camera.default_info_url`).

## Do you need to calibrate?

Usually **no** — intrinsics barely vary between identical camera units, so the
committed default is fine for display and coarse perception. Calibrate your own
(and save it to `~/.mote/camera_calibration.yaml`) when:

- you fitted a **different camera** than the UGREEN model the default was made for;
- you need **metric accuracy** — e.g. the L1 obstacle/depth work that projects
  pixels onto the ground or into 3D, where small intrinsic errors bias range;
- straight edges look **curved** in the rectified image, or `/camera_info`
  clearly doesn't match your lens;
- you changed the capture **resolution** (the default is for 640x480).

## Print the target

Print [`checkerboard_a4_25mm_7x10.pdf`](checkerboard_a4_25mm_7x10.pdf) on A4 — it
is **7x10 squares of 25 mm** (so **6x9 inner corners**), with the calibrator
parameters printed along the bottom edge.

The page is deliberately a little smaller than A4 so it fits inside the printable
area of any A4 printer: most printers should print it unscaled, and the squares
come out true-size **without changing any print settings**. Just print it on A4
and the board is centred with a white border.

- **Mount it dead flat** on a clipboard, foamboard, or glass. A floppy sheet
  warps and wrecks the result.
- The squares should measure 25 mm on each side however if they do not match
  exactly it should still calibrate correctly. The **intrinsics** are
  independent of the absolute square size — square size only affects metric
  scale, which this calibration does not rely on.

## Running the calibration

`cameracalibrator` is an interactive GUI tool, so run it on the **workstation**
(it is a `dev` dependency, not installed on the robot) against the Pi's live
camera over the ROS 2 network. Both machines must be on the same LAN and
`ROS_DOMAIN_ID`.

1. Print and mount the target above.
2. On the **Pi**, start the camera so it publishes `/image_raw` (`pixi run
   launch`, or just the `v4l2_camera` node).
3. On the **workstation** run the calibrator:

   ```bash
   pixi run camera_calibration
   ```

4. Move the board through the frame until the X/Y/Size/Skew bars are full, then
   press **CALIBRATE**. It prints the result (`camera matrix`, `distortion`,
   `rectification`, `projection`) to the console in oST format.

   Copy the printed parameters into a `camera_calibration.yaml` file.

5. **Install it.** Copy calibration file to `~/.mote/camera_calibration.yaml`
  **on the Pi**.
