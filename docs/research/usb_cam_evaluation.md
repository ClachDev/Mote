# Camera driver evaluation: v4l2_camera vs usb_cam (issue #19)

**Recommendation: don't switch.** The one capability that motivated the swap —
shipping the camera's native MJPEG straight to `/image_raw/compressed` with no
per-frame conversion — is broken in every released version of `usb_cam`,
including the 0.8.1 binary on robostack-jazzy. The working `usb_cam` configs
only beat the current setup by capping the frame rate, and the measured cost of
the current setup is too small to justify the integration and re-validation
work: the whole bringup (camera included, with a compressed consumer attached)
uses ~5 % of the Pi 5's total CPU at ~61 °C.

Measured 2026-07-16 on the robot (Pi 5, UGREEN webcam, `pixi run launch`,
workstation subscriber over Wi-Fi). Raw numbers and method below.

## Research question answers

### 1. Does the UGREEN webcam expose MJPG @ 640x480?

Yes. `v4l2-ctl --list-formats-ext` shows MJPG at every resolution
(1920x1080 down to 320x240) at 30/25/20/15/10/5 fps, plus YUYV.
Native MJPEG frames at 640x480 average **14.2 KB** (90-frame capture via
`v4l2-ctl --stream-mmap`, 1,281,408 bytes total) — the smallest possible
representation of what the camera produces.

### 2. Can usb_cam deliver native MJPEG straight to `/image_raw/compressed`?

**No — the passthrough is broken upstream, in 0.8.1 and still on `main`.**

- The node only creates the compressed publisher when
  `pixel_format == "mjpeg"` (`src/ros2/usb_cam_node.cpp:174`), but the
  driver's format table names the format `raw_mjpeg`
  (`include/usb_cam/formats/mjpeg.hpp:68`). The two never meet:
  - `pixel_format:=mjpeg` → rejected at configure time, verified on the Pi:
    `Specified format 'mjpeg' is unsupported by this ROS driver` (it isn't in
    the driver format list the error prints).
  - `pixel_format:=raw_mjpeg` → starts, but the compressed publisher is never
    created; the JPEG byte stream is pushed through the *raw* `image_raw`
    path with a fabricated encoding derived from `av_device_format` (garbage
    for subscribers — upstream issue #346 reports crash/green-screen).
- Upstream: issue [#346](https://github.com/ros-drivers/usb_cam/issues/346)
  (open), fix PR [#378](https://github.com/ros-drivers/usb_cam/pull/378)
  (3-line rename, open/unmerged since Oct 2025). The feature landed in
  [#270](https://github.com/ros-drivers/usb_cam/issues/270) and has never
  worked in a release; 0.8.1 predates all of this (May 2024).
- Even with the rename fixed, `take_and_send_image_mjpeg()` publishes a
  fixed-size buffer (`get_image_size_in_bytes()` = width x height x
  bytes-per-pixel, not the frame's `bytesused`), so every CompressedImage
  would be padded to **614.4 KB** at 640x480 — ~43x the actual 14.2 KB JPEG,
  i.e. *worse* on Wi-Fi than today's re-encoded stream. Still true on `main`.

Getting the real win would mean vendoring a patched fork (two patches, one
unmerged upstream) and building it ourselves — exactly the maintenance
profile the robostack binary was supposed to avoid.

### 3. Measured Pi CPU % and SoC temperature

Method: 60–90 s windows sampling `/proc/stat`, `/proc/<pid>/stat`,
`/proc/net/dev` and `thermal_zone0` on the Pi while `pixi run launch` ran
(lidar, ros2_control, kinematic_icp all up); "sub" = one
`ros2 topic bw /image_raw/compressed` subscriber on the workstation over
Wi-Fi, which is what makes the lazy JPEG encode run — with no subscriber the
compressed plugin does no work.

| Config | Camera node CPU, no sub | Camera node CPU, with sub | fps | `/image_raw/compressed` per subscriber |
|---|---|---|---|---|
| **v4l2_camera, YUYV passthrough (current)** | **0.5 %/core** | **14.7 %/core** | 28.7 | 645 KB/s (22.4 KB/msg) |
| usb_cam `mjpeg2rgb` @ 20 fps | 4.0 %/core (ffmpeg decode is unconditional) | 13.1 %/core | 20 | 427 KB/s (24.2 KB/msg) |
| usb_cam `yuyv` @ 20 fps | 0.4 %/core | 10.2 %/core | ~18 | 576 KB/s (32.5 KB/msg) |
| usb_cam `mjpeg`/`raw_mjpeg` passthrough | broken (see Q2) | — | — | would be ~285 KB/s @ 20 fps if fixed upstream |

- Whole-system CPU: idle 0 %, launch with no camera consumer ~3 % of all four
  cores, launch + compressed subscriber ~4.5–5.6 %.
- Temperature: 51–52 °C idle, 60–62 °C under launch in every config (Pi 5
  throttles at 80+ °C). The camera driver makes no measurable difference.
- Per-frame encode cost is *slightly worse* under usb_cam (13.1 %/20 fps =
  0.66 %/fps vs 14.7 %/28.7 fps = 0.51 %/fps). All of usb_cam's advantage is
  the frame-rate cap, none of it efficiency.
- **The camera is not straining the Pi.** Note the current pipeline is already
  one conversion cheaper than when #19 was filed: #26 set
  `output_encoding: yuv422_yuy2`, so v4l2_camera publishes the captured
  format as-is (0.5 %/core) and the only per-frame cost is the compressed
  plugin's YUY2→BGR→JPEG when something actually subscribes.

### 4. Wi-Fi bandwidth

Table above; wlan TX measured on the Pi matches `ros2 topic bw` (each
additional subscriber costs another unicast copy: two subscribers → 1.3 MB/s).
Current worst case is ~0.65 MB/s per consumer, and in practice there is one
consumer (the depth node's server hop) that only subscribes while depth is
running. usb_cam@20 fps trims that to ~0.43–0.58 MB/s — real but marginal on a
link that sustains tens of MB/s. The 3x prize (285 KB/s native MJPEG, no
encode CPU at all) is locked behind the upstream breakage in Q2.

### 5. Integration cost (recorded for if this is ever revisited)

Verified on-robot from a scratch pixi env (`ros-jazzy-usb-cam` 0.8.1 solves
fine on linux-aarch64; pulls ffmpeg/libavcodec):

- Params remap in `mote_launch.py`/`robot.yaml`: `video_device` (symlink
  resolved fine), `image_size: [w,h]` → `image_width`/`image_height`,
  `camera_frame_id` → `frame_id`, add `framerate` + `pixel_format`,
  `output_encoding` has no equivalent. `camera_info_url` exists and the
  `~/.mote/camera_calibration.yaml` plumbing carries over (but usb_cam warns
  unless the yaml's `camera_name` matches).
- Topic/format compatibility: `/image_raw` + `/image_raw/compressed` appear as
  today in `yuyv`/`mjpeg2rgb` modes; depth node's format check accepts
  usb_cam's plain `"jpeg"` format string (verified in
  `depth_obstacle_node.py`). A (fixed) passthrough mode publishes **no raw
  `image_raw` at all** — `camera_monitor`, RViz raw views, and any future raw
  consumer would need the compressed stream instead.
- Timestamps come from the v4l2 buffer time shifted to epoch (fine on the
  robot; arguably better than stamping at publish). Publisher QoS is a
  hard-coded depth-100 queue for the compressed path.
- This camera's UVC controls use newer names, so usb_cam logs `unknown
  control 'white_balance_temperature_auto'` / `'exposure_auto'` warnings at
  startup (harmless, but its extra camera controls partly don't apply).
- Re-validation: camera intrinsics path, depth pipeline e2e, lidar-depth
  rescale (frame rate change alters frame pairing), record streams.

## Verdict

Stay on `v4l2_camera`. Its one real limitation — no in-driver frame-rate cap —
buys usb_cam at most ~4.5 %/core and ~0.2 MB/s in the only config that works,
against a driver swap plus re-validation of the calibration and depth
pipelines. Revisit only if (a) Pi CPU/Wi-Fi actually becomes a bottleneck, and
(b) upstream merges the passthrough fix (usb_cam PR #378) *and* sizes the
compressed message by `bytesused` — that combination would drop the camera to
near-zero CPU and ~285 KB/s, which is worth having if it ever ships in a
robostack binary.

Scratch test env left on the Pi at `~/tmp/usbcam-eval` (pixi project with
`ros-jazzy-usb-cam`; delete freely).
