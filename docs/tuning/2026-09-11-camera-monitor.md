# camera_monitor off the raw stream, 2026-09-11

**Verdict: 5.0 → 2.3% of a core and 64.5 → 59.1 MB resident, with the same
report.** `camera_monitor` subscribed to raw `/image_raw` — 640x480 rgb8 at
30 Hz, 921,600 bytes a frame, ~27 MB/s deserialized into Python — to increment
a counter and read three fields off the last frame. It now counts
`/image_raw/compressed` and reads the resolution from `camera_info`, once.

Raw sampler output is in `2026-09-11-camera-monitor/`; every figure below is a
`mean` from one of the `cm_*.json` files there.

## How it was measured

On the workstation (16 cores; load1 2.0 at the start, 1.0 at the end), by
`2026-09-11-camera-monitor/run.sh 180`:

- `camera_stream.py` stands in for `v4l2_camera`: `/image_raw` (640x480 rgb8),
  `/image_raw/compressed` (a 40 KB payload, `compressed_image_transport`'s
  format string) and `/camera_info`, stamped alike, at 30 Hz.
- Four builds run **at the same instant** against that one stream, each under a
  script name of its own, and each is sampled by its own
  `overhead.py --names none --min-procs 1 --cmdline-names <name>.py`, 5 s
  interval, 36 samples over 175 s:
  - `cm_before` — the node as it was (`f440e46`), on `/image_raw`;
  - `cm_after` — this change;
  - `cm_standing` — this change with the `camera_info` subscription left
    standing instead of read once;
  - `cm_idle` — the old node on a topic nothing publishes.
- ROS_DOMAIN_ID 87, discovery LOCALHOST, loopback-only Cyclone.

## Results

| build | what it subscribes to | CPU % of a core | RSS MB |
| --- | --- | --- | --- |
| `cm_before` | `/image_raw`, 30 Hz raw | **5.0** | **64.5** |
| `cm_after` | `/image_raw/compressed` + one `camera_info` | **2.3** | **59.1** |
| `cm_standing` | `/image_raw/compressed` + `camera_info`, 30 Hz each | 3.1 | 60.0 |
| `cm_idle` | nothing arrives | 0.2 | 57.0 |

Every one of the three live builds logged `30.0 FPS, 640x480` in all 36 reports;
`cm_idle` warned `No frames received` in all 36.

The issue measured 4.4% / 67.2 MB raw and 0.0% / 59.8 MB idle with a different
harness on a different day; this run reproduces the shape (the subscription is
the whole cost) at 5.0 / 64.5 and 0.2 / 57.0.

## What that says

**Removing the raw payload halved the cost; it did not remove it.** Above idle,
the raw build costs 4.8 points and the compressed one 2.1: what remains is 30
rclpy wake-ups a second, the cost `2026-08-11-monitor-cpu.md` found to be 87% of
a subscription's on the robot. A Python node counting a 30 Hz topic cannot get
below that, whatever it subscribes to.

**That is also why `camera_info` is read once.** A standing `camera_info`
subscription is a second 30 Hz stream of wake-ups, and it cost 0.8 points —
a third on top of the frame count — to report a resolution that only changes
when the driver restarts. The monitor subscribes, takes one message and
destroys the subscription; a report with no frames in it (the signature of that
restart) clears the resolution and re-arms the read. The one-shot path, the
re-arm after a dropout, and a report with frames but no `camera_info` are in
`mote_perception/test/test_camera_monitor.py`.

## The sim

The gz bridge publishes only raw `/image_raw`, so a `camera_monitor` on the
compressed stream would have reported no frames at all under `pixi run sim`.
`sim_launch.py` now runs an `image_transport` republisher that adds
`/image_raw/compressed`, as the real camera driver does — which also gives
`depth_obstacle_node` and `object_detector_node` a stream in the sim for the
first time, since both read only the compressed one.

`2026-09-11-camera-monitor/sim_check.sh` brings up `sim_launch.py` (no
mission) on a claimed domain, then `perception_launch.py use_sim_time:=true` —
what `pixi run perception` runs — and takes the camera's real rate from the
bridge's own raw frames' sim-time stamps. Output is in `sim/`:

- truth: 1200 raw frames over 39.6 sim s, **30.31 frames per sim second**,
  real-time factor 0.99 (`sim/truth.txt`);
- `camera_monitor`: **30.2–30.4 FPS, 640x480, format=rgb8; jpeg compressed
  bgr8** in every full report (`sim/camera_monitor.log`; the first reads 21.2
  because the node started partway through its window);
- the republisher is lazy, as claimed: before perception started, `/image_raw`
  had **0 subscribers** and `/image_raw/compressed` 0; with it up, 1 and 2
  (`camera_monitor` and `depth_obstacle_node`) (`sim/graph.txt`). A sim run
  with no camera consumer — the smoke test, the benchmark — therefore pays one
  idle process and one DDS participant for it, and no image copies or JPEG
  encoding.
