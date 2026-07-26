# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Common Commands

All tasks are run via [pixi](https://pixi.sh). Never invoke `colcon` or `ros2` directly — always use `pixi run <task>`.

```bash
pixi run build          # Build all packages with colcon + Ninja
pixi run submodules     # Fetch git submodules (sllidar_ros2, kinematic_icp)
pixi run launch         # Full robot bringup (hardware + lidar + camera + localization)
pixi run slam           # SLAM stack only (run alongside launch)
pixi run nav            # Nav2 stack (loads the active site's map; see Sites)
pixi run mapping        # bringup + SLAM together (build/extend a map)
pixi run robot          # bringup + Nav2 together (drive the active site's map)
pixi run save-map       # Save map + slam posegraph into the active site floor
pixi run save-zone <n>  # Teach a zone: capture current robot pose (+ optional --radius) into the site
pixi run site           # Site CLI: create / add-floor / use / use-map / list / info
pixi run teleop         # Keyboard teleoperation
pixi run tasks          # Task layer: behaviour-tree task_server (see mote_tasks)
pixi run arm            # SO-101 arm driver: joint states + safe jog control
pixi run arm-jog        # Interactive per-joint jog CLI (needs `pixi run arm`)
pixi run arm-check      # Standalone arm bus enumeration + health (read-only)
pixi run arm-pose       # Teach/replay named arm poses; derive soft limits
pixi run sync           # rsync project to Pi at SSH host 'mote'
pixi run setup          # One-time Pi setup: udev + wifi-powersave + systemd (needs sudo)
pixi run udev           # Install udev rules + dialout group (needs sudo)
pixi run wifi-powersave # Disable WiFi power save via NetworkManager (needs sudo)
pixi run setup-ids      # Guided servo ID assignment tool
pixi run kill           # Kill stale ROS processes and reset daemon
pixi run identity       # Fleet identity CLI: show / id / set --id --name --site
pixi run tailnet        # Join this machine to the Tailscale overlay (needs sudo)
pixi run provision      # Render cloud-init user-data for a clean Pi
pixi run dds-check      # DDS participant-slot headroom on this host

# Dev environment only (installs ros-jazzy-desktop)
pixi run rviz           # RViz2 with mote config

# Sim environment only (gz-sim Harmonic + ros_gz + gz_ros2_control; own solve,
# never affects the robot/Pi env). The sim/sim-test tasks auto-select the sim
# environment (defined only there), so no `-e sim` is needed for them.
pixi run sim            # Headless Gazebo sim: world + robot + controllers (no mission)
pixi run sim-mapping    # Sim running the real mapping mission (mapping_launch.py)
pixi run sim-nav        # Sim running the real nav mission (robot_launch.py + saved map)
#   Trailing args pass through to the launch, so pick a world with:
#     pixi run sim world:=hospital_world.sdf   (default: mote_world.sdf)
#   Worlds form an easy->hard ladder: mote_world (easy), office_world (medium),
#   hospital_world (hard).
#   sim-mapping/sim-nav are just `sim mode:=mapping` / `sim mode:=nav`.
pixi run sim-test       # ~20 s headless smoke test (local pre-PR gate, needs a GPU)
# Ad-hoc (non-task) commands still need the env named:
#   pixi run -e sim -- ros2 launch mote_bringup slam_launch.py use_sim_time:=true
pixi run test           # colcon test for mote_hardware (gtest)

# Lint environment only (pre-commit; minimal env, no ROS — auto-selected)
pixi run lint           # run all pre-commit hooks across the tree (~1 s cached)
pixi run lint-install   # wire pre-commit into .git/hooks (one time per clone)
```

Build artifacts go into `build/`, `install/`, and `log/` — all ignored by git. If you see CMakeCache.txt errors about a wrong source directory (e.g. from a path rename), delete the stale `build/` directory and rebuild.

## Robot Configuration

**All robot config lives in one place: `mote_description/config/robot.yaml`.**

This file is the single source of truth for:
- Wheel geometry (`wheel_radius`, `wheel_separation`) — used by the URDF for geometry and by `diff_drive_controller` for odometry
- Drive servo bus parameters (port, baud rate, servo IDs, `velocity_scale`, acceleration)
- Sensor device paths and baud rates (lidar, camera)

The URDF reads it via `xacro.load_yaml('$(find mote_description)/config/robot.yaml')` at xacro processing time. The launch file reads it via `yaml.safe_load` at launch time. Neither file duplicates any of these values.

`velocity_scale` converts rad/s ↔ servo speed units. It's an empirical calibration value measured on real hardware with the `velocity_cal` tool, not derivable from datasheets.

## Fleet: overlay, identity, and the per-robot/shared split

Milestone M0 of `docs/design/fleet.md`; the operator runbook is `docs/fleet/README.md` and the measurements behind it are `docs/fleet/m0-verification.md`. **`MOTE_HOME` (default `~/.mote`) is per-robot state; the package is shared config** — `mote_bringup/mote_home.py` is the one place that rule lives (`mote_dir()`, `path()`, and `override(name, packaged_default)` which prefers the per-robot file). `sites.py`, `mote_launch.py` (camera calibration), `perception_launch.py` (`perception.yaml`), `health_monitor.py` (`health.yaml`) and `self_check.py` (`self_check_status.yaml`) all resolve through it, so `MOTE_HOME` is honoured everywhere and an update can never clobber identity, site selection, calibration, maps or bags. **Identity** is `$MOTE_HOME/robot.yaml` (`mote_bringup/identity.py`, `pixi run identity show|id|set`): a `robot_id` constrained to a lowercase DNS label because it is simultaneously a MagicDNS hostname, an MQTT topic level and a directory name. It is deliberately not the hostname, and operator-set until M1's enrollment endpoint allocates it. Do not confuse `$MOTE_HOME/robot.yaml` (this robot's identity) with `mote_description/config/robot.yaml` (shared hardware description). **The overlay** is Tailscale (`pixi run tailnet`, `mote_bringup/tailscale/install.sh`), joining robots/servers as *tagged* devices and the workstation as a user device; a robot's tailnet hostname *is* its `robot_id`. **A clean Pi** is provisioned by one rendered cloud-init file (`pixi run provision`, `mote_bringup/provision.py` + `provisioning/user-data.template`): identity → tailnet (single-use tagged auth key baked into the image, shredded after use) → pixi/build → `pixi run setup`. **DDS**: the end state is `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` on the robot, which retires the `ROS_DOMAIN_ID` isolation question entirely — but that waits for **M2**, because nothing on-robot replaces an operator's RViz until `foxglove_bridge` lands, so today the robot stays LAN-discoverable and is scoped by `config/cyclonedds.xml` instead (see DDS scoping under Environment). What M0 adds is the measurement: rmw_cyclonedds caps localhost discovery at `MaxAutoParticipantIndex=32`, i.e. 33 participants (≈ processes) per host, and `pixi run dds-check` reports the headroom from `/proc/net/udp` (measured 17/33 for the sim nav mission under both localhost and stock discovery; ~24 projected with perception + the M1 agent + foxglove_bridge). Re-check it whenever a milestone adds processes — that budget is spent before M2 arrives to claim it; raise it in the robot's `cyclonedds.xml` if it runs out.

## Sites (maps & zones)

Everything that is only meaningful relative to one mapped place — the Nav2 map pair, the slam_toolbox posegraph, and named zones — lives together as a **site bundle** under `~/.mote/sites/<site>/floors/<floor>/`, managed by `mote_bringup/sites.py` (CLI: `pixi run site`, docs in the module docstring). A floor is one SLAM session (one map frame); a site groups floors sharing a location. `~/.mote/active.yaml` selects the active site/floor per robot; launch files resolve the map (`nav2_launch.py`, `robot_launch.py`) and zones (`tasks_launch.py`) from it at launch time (zones fall back to the committed default). `MOTE_HOME` overrides `~/.mote` for tests/experiments. Map artifacts are immutable **revisions** under `floors/<floor>/maps/<rev>/`, published by atomically flipping the `floors/<floor>/map` symlink once the revision is complete — a half-written save or interrupted transfer is never visible, and `site use-map <rev>` rolls back. `save-map` stores the posegraph alongside the map so mapping can be *continued* in the same frame later (extend, don't remap — remapping breaks zone coordinates). Mapping runs also record the `mapping` rosbag stream by default (`mapping_launch.py record:=true`; the sim passes false), and `save-map` stamps the session's bag into the revision's `meta.yaml` for provenance (`site info` shows it). Zones are taught by driving there and running `pixi run save-zone <name>`, not by editing YAML; a zone is a named pose (a fetch waypoint or a `goto <zone>` target) that may optionally carry an area **footprint** — a taught `--radius` circle, or a `polygon` outline that follows the actual room walls — so it reads as a room and answers "am I in it"; one concept, one `zones.yaml`, `site info` shows the zone/footprint counts. Maps are saved as PNG (map_server reads it natively; browsers can render it directly). `save-map` automatically runs an FFT structure-extraction **cleaning pass** (`mote_bringup/map_cleanup`, `sites._promote_cleaned`): it keeps the untouched map_saver output as `map_raw.png` and promotes the decluttered image to the served `map.png` (plus a `diagnostics.png`), so navigation always consumes the cleaned map while the raw is retained for provenance/audit. The `map.yaml` frame is identical for both, so zones/localization are unaffected; a cleaning failure falls back to serving the raw. The posegraph belongs to the raw map — mapping continuation extends from raw, never the cleaned image.

## Architecture

Mote is a differential-drive robot built on **ROS 2 Jazzy**, managed entirely through pixi (no system ROS install required). Four first-party packages:

### `mote_hardware` (C++)
A `ros2_control` `SystemInterface` plugin (`MoteHardware`) that drives two Feetech STS3215 servos via the SCServo SDK over a serial bus. Key implementation details:
- Servo IDs and all hardware params come from `robot.yaml` via the URDF's `<ros2_control>` tag, read by `MoteHardware::on_init` from `info_.hardware_parameters`
- Position is tracked cumulatively across the 12-bit encoder rollover using a half-range threshold
- The left wheel is mounted inverted, so its sign is negated in both `read()` and `write()`
- The serial port is opened in `on_activate` (not `on_init`), which also puts servos into wheel (continuous rotation) mode — an EEPROM write, skipped if already set
- Tools built from `mote_hardware/tools/` (`servo_debug`, `velocity_cal`, `swap_ids`, `setup_ids`) run as `pixi run -- ros2 run mote_hardware <tool>`; see `mote_hardware/tools/README.md`

### `mote_description` (CMake)
Contains `urdf/mote.urdf.xacro` and `config/robot.yaml`. The xacro loads robot.yaml at processing time and uses those values directly — no xacro args are needed or accepted. The `<ros2_control>` tag embeds the servo params so they reach `MoteHardware::on_init`.

### `mote_bringup` (Python/ament)
Launch files, config, udev rules, NetworkManager drop-ins, systemd services, and the fleet foundation: `mote_home.py` (per-robot state root), `identity.py` (`identity` console script), `provision.py` + `provisioning/user-data.template` (`provision`), `dds_participants.py` (`dds_participants`), and `tailscale/install.sh` — see Fleet above.

**On-robot reliability** (see `mote_bringup/README.md`): `pixi run robot`/`mapping` include the health monitor, so a manual run publishes `/health` too; the systemd units are installed by `pixi run setup` but **not enabled** (autostart would drain the battery on a desk — opt in with `systemctl enable --now mote-bringup mote-health`). the systemd services restart with backoff and never permanently give up (`Restart=always`, `RestartSec`/`RestartSteps`/`RestartMaxDelaySec`, `StartLimitIntervalSec=0`), order after the udev-tagged `dev-mote_*.device` units, and bound the journal. A pre-flight self-check (`self_check.py`, run as `mote-bringup`'s `ExecStartPre`; `pixi run self-check`) gates bringup on servo ping + lidar/camera/disk/clock/config and keeps the robot idle with a clear reason on failure. A health monitor (`health_monitor.py`, `mote-health.service` with a `Type=notify` watchdog; `pixi run health`) publishes per-subsystem `diagnostic_msgs/DiagnosticArray` on `/diagnostics_agg` and a single OK/DEGRADED/FAULT summary on `/health`. Driver and nav2 nodes are `respawn=True` for per-node recovery under the whole-service systemd restart. Battery voltage is **not** software-measurable (the power bank exposes no telemetry); `system_monitor` reports the Pi under-voltage flag as the only power signal.

**Launch hierarchy:** the two mission launches (`mapping_launch.py`, `robot_launch.py`) each take a `base` arg (default true) that includes the hardware base, and a `use_sim_time` arg they forward to everything they include. The sim runs these *same* files with `base:=false`, supplying a Gazebo base in place of the drivers — so the missions are defined once and the sim exercises the real launch files.
- `robot_launch.py` — nav mission: `mote_launch.py` (if `base`) + `nav2_launch.py` (drive a saved map). Forwards a `map` arg, defaulting to the active site's map (see Sites).
- `mapping_launch.py` — mapping mission: `mote_launch.py` (if `base`) + `slam_launch.py` + `nav2_launch.py` (`localisation:=false`) + `record_launch.py` (`streams:=mapping`, unless `record:=false`): build/extend a map with SLAM *and* drive to goals autonomously while doing so, recording the session for map provenance.
- `mote_launch.py` — the hardware base: robot_state_publisher, ros2_control_node, controller spawners, sllidar, laser_filter, v4l2_camera, and `localization_launch.py`. Reads `robot.yaml` for wheel geometry (injected into DiffDriveController params) and sensor config. Asserts `use_sim_time` (default false) for the whole tree via `SetParameter`.
- `localization_launch.py` — kinematic_icp LIDAR odometry (publishes `odom`→`base`; the map→odom corrector is slam_toolbox when mapping or AMCL when navigating). Despite the name, it does *not* run AMCL — AMCL lives in `nav2_launch.py`.
- `slam_launch.py` — slam_toolbox (accepts `use_sim_time:=true` for the sim)
- `nav2_launch.py` — Nav2 stack. A `localisation` arg (default true) toggles the `map_server` + `amcl` half: true localises against a saved map; false drops them so the navigation servers run against a live slam_toolbox map and `map→odom` instead (used by `mapping_launch.py`)
- `rviz_launch.py` — RViz2 (dev environment only)

**Config files** (`mote_bringup/config/`):
- `controllers.yaml` — controller_manager update rate, DiffDriveController settings (wheel geometry is injected from `robot.yaml` at launch time, not stored here — the launch file writes it to a temp params file keyed by node name, since a plain dict would never reach the controller node)
- `laser_filters.yaml` — filters lidar blind spots
- `nav2_params.yaml` — Nav2 parameters
- `slam_toolbox_params.yaml` — SLAM toolbox parameters
- `mote.rviz` — RViz2 display config
- `cyclonedds.xml` — robot DDS tuning: pins the graph to the one real interface (`MOTE_DDS_INTERFACE`, detected by `systemd/install.sh`; declared optional plus an `lo` fallback so a network-less robot still runs) and drops multicast to SPDP discovery only, so scans/maps/images go unicast to the peers that asked. Loaded via `CYCLONEDDS_URI` from the systemd units only, so an interactive `pixi run` keeps stock DDS. Discovery is deliberately *not* localhost-confined here — an operator laptop must still see the robot

### `mote_simulation` (Python/ament)
Workstation-only Gazebo simulation, kept separate from `mote_bringup` so it can be excluded from the robot sync (`pixi run sync` skips `mote_simulation/`). Built only in the `sim` pixi environment. Contains:
- `launch/sim_launch.py` — Gazebo sim: headless gz server, robot spawn, ros_gz bridge (/clock, /scan), controllers, laser_filter, and the shared `localization_launch.py`. Takes a `world:=` arg (file in `mote_simulation/worlds/`, default `mote_world.sdf` — the simple smoke-test room; `office_world.sdf` is a medium hospital-ward corridor for stress-testing localisation; `hospital_world.sdf` is the hard tier — a ~58x38 m looping hospital, generated by `worlds/gen_hospital.py`). The URDF is processed with `use_sim:=true`, which swaps `MoteHardware` for `gz_ros2_control` and adds a simulated lidar (specs from `robot.yaml` `lidar.sim`). Without that flag the xacro output is unchanged. Controller params are merged into one temp file (gz_ros2_control loads a single `<parameters>` file referenced in the URDF). It pulls `controllers.yaml`, `laser_filters.yaml`, and `localization_launch.py` from `mote_bringup`'s share so the sim and the real robot can't drift apart. It asserts `use_sim_time:=true` for the whole process tree via `SetParameter`. A `mode:=mapping|nav` arg includes the real `mapping_launch.py` / `robot_launch.py` with `base:=false` (default `none` = sim only): the sim provides the base and *delegates* the mission to the actual launch files, so it can't re-encode or drift from them, and `pixi run sim-mapping` / `sim-nav` put those mission launches under test. Mission modes also include `tasks_launch.py` with the loaded world's sibling `worlds/<world>.zones.yaml` as `zones_file`, so the fetch mission runs anywhere on the world ladder with matching zone coordinates — and since that file's room zones carry footprints, `goto <zone>` runs on the ladder too. (`pixi run mapping`/`robot` are the hardware entry points — same files, `base` defaulting true, wall-clock time.) The dependency direction stays one-way: `mote_simulation` includes from `mote_bringup` and `mote_tasks`, never the reverse.
- `worlds/` — an easy->hard ladder: `mote_world.sdf` (easy smoke-test room), `office_world.sdf` (medium hospital-ward corridor), and `hospital_world.sdf` (hard ~58x38 m looping hospital with ~50 rooms and clutter). The hard world is generated by `worlds/gen_hospital.py` (committed alongside its output — edit the script's layout and regenerate rather than hand-editing the SDF). Every world has a sibling `<world>.zones.yaml` with the same waypoint zones (`pickup`/`dropoff`/`home`) plus a few room zones (reachable by `goto`) carrying a footprint: the two smaller worlds use a `radius` circle, the hospital's rooms use `polygon` outlines of the actual ward rectangles. The hospital's is emitted by the generator, which asserts every zone clears the walls and furniture and lies inside its own footprint.
- `test/sim_smoke/` — `run_sim_smoke.sh` + `verify_sim.py`, the `pixi run sim-test` gate.
- `sim_home/` — a committed, in-repo **sim MOTE_HOME**: one real Site bundle per world (site name == world stem, floor `ground`) holding that world's SLAM map + zones. The sim pixi env points `MOTE_HOME` here (`[feature.sim.activation.env]`), so `sim-nav` loads a world's own map and never touches the robot's real `~/.mote`; `sim_launch.py` nav mode resolves the map from the `world` arg via the world's site and passes it to `robot_launch.py`. Only the bundles are committed — `active.yaml`/`bags/` are gitignored.
- `tools/map_world.sh` + `tools/explore.py` — how those sim sites are built, the same way the robot maps: `pixi run sim-map-world <world.sdf> [budget_s]` launches the real mapping mission headless, runs `explore.py` (reactive autonomous coverage — left-wall following, with a frontier-seek escape for looping layouts, driving `cmd_vel` off `/scan_filtered`), then `save-map`s into the world's site. Sim maps are ground-truth-clean, so the save uses `save_map(clean=False)` (serve the raw map_saver output): the FFT declutter pass, tuned for real-sensor noise, would strip the thin true walls. Re-running adds a new revision.

### `mote_perception` (Python/ament)
Home for camera-derived perception. Runs on the robot (feeds Nav2), so unlike `mote_simulation` it is synced to the Pi. Contains:
- `mote_perception/camera_monitor.py` — a dependency-light camera health monitor (rclpy + sensor_msgs only, no OpenCV). Subscribes to `image` and logs measured frame rate, resolution, and encoding on a timer, warning on dropouts. Registered as the `camera_monitor` console_script.
- **L1 depth-obstacle pipeline** — turns the mono camera into `/camera_obstacles` (PointCloud2) for Nav2's `camera_layer`. Split across: `depth_obstacle_node.py` (torch-free rclpy node: compressed image → server → rescale → back-project → level → z/range gates), `depth_wire.py` (the socket protocol spec + `DepthClient`, shared by node/server/tools), `lidar_rescale.py` (per-frame Theil-Sen affine-in-disparity metric rescale anchored to lidar), `ground_projection.py` (camera↔base geometry: `GroundProjector`, floor-plane fit, leveling, pixel→floor rays). Split by concern, not by machine: `depth_obstacle_node` runs on the robot (launched by `perception_launch.py`, in its DDS graph), reaching the torch server over TCP at `inference_host`. `tools/depth_server.py` runs in the `inference` pixi env (torch, no ROS) wherever the GPU is; `pixi run inference` starts it beside the detect server. `pixi run inference-rocm` is the AMD GPU variant: the same servers in the `inference-rocm` env (torch from the pytorch.org ROCm wheel index, own solve; `HSA_OVERRIDE_GFX_VERSION` set for unsupported AMD iGPUs). As a *deployed role* on a dedicated NVIDIA machine (gaming PC or cloud GPU) the same servers ship as a **container image** (`mote_perception/deploy/Dockerfile` -> `ghcr.io/clachdev/mote-inference`, built by `.github/workflows/inference-image.yml`): that host installs no repo, pixi, or scripts — one `docker run --gpus all --restart unless-stopped`. Every variant runs the same supervisor (`tools/inference_server.py` — add a tenant by adding a row to `SERVICES`). Models load on demand and release after `--idle-timeout` (default 300 s) via `tools/model_host.py`, so the machine isn't holding VRAM while idle. The full role (host decision, update story, cloud scaling + the unauthenticated-socket caveat, `pixi run inference-health`, `pixi run inference-bench`, multi-service pattern, fallback matrix) is in `docs/inference-server.md`; wire modules carry a `HEALTH_MAGIC` request so `WireClient.health()` reports each server's model/device/version (`MOTE_VERSION` baked into the image, else `git describe`). The server takes `--device auto|cpu|cuda` (auto → GPU when available, else CPU) and optional `--fp16`. The iGPU doesn't beat the CPU at idle (small ViT, bandwidth-bound) but stays flat under CPU load where the CPU-only server degrades to ~1–2 s/frame; fp16 and larger models can crash/hang on unsupported iGPUs (gfx1103), so keep fp32 + V2-Small there. Needs `/dev/kfd` access (render/video groups).
- **L2 open-vocabulary detection** — turns "fetch the red box" into a map pose for the task layer. Same node-on-robot / server-off-board split: `tools/detect_server.py` (OWLv2 in the `inference` pixi env; `pixi run detect-server`, or `pixi run inference` for both), `detect_wire.py` (protocol + `DetectClient`; the query labels ride in each request), `object_detector_node.py` (torch-free rclpy node: idles until labels arrive on `detect/labels` — String, comma-separated, transient_local, empty = idle — then grounds each bbox bottom-centre through the floor plane and publishes `detected_objects`, vision_msgs/Detection3DArray in the map frame at the capture stamp). Floor-ray grounding is metre-accurate only near the robot (camera is at ~0.10 m), gated by `range_max`.
- `tools/` — offline bag harnesses (`depth_bag_replay`, `depth_bag_eval`, `depth_obstacles`, `detect_bag`, `bag_overlay`, shared `bag_utils`) and the live `measure_camera_pitch`; see `mote_perception/README.md` for the inventory.
- `launch/perception_launch.py` — declares `use_sim_time` (applied via `SetParameter`) and starts `camera_monitor` (with `image` remapped to `/image_raw`) plus the depth/detect nodes. Which nodes run, their `server_port`s, and the shared `inference_host` come from `config/perception.yaml` (not launch args), so inference can move machines without editing launch. Not part of the mission bringup — run `pixi run perception` alongside `pixi run mapping`/`robot`.
- `config/` — camera-calibration + perception-runtime home. `camera_info.default.yaml` is a committed fallback calibration for the UGREEN webcam; a per-robot `~/.mote/camera_calibration.yaml` (outside the repo) overrides it. `mote_launch.py` prefers the `~/.mote` file when present, else `robot.yaml`'s `camera.default_info_url`, passing the result to `v4l2_camera_node` as `camera_info_url`. `perception.yaml` (`inference_host`, per-node `enabled`/`server_port`) is read by `perception_launch.py` with the same `~/.mote/perception.yaml` override. `config/README.md` documents when/how to calibrate (with the printable checkerboard).
- Compressed transport is already provided by the `image-transport-plugins` dep, so the camera publishes `/image_raw/compressed`; off-board/RViz consumers should prefer it. See `mote_perception/README.md`.

### `mote_tasks` (Python/ament)
The task layer: py_trees behaviour trees on top of Nav2 (synced to the Pi). py_trees is a pixi *PyPI* dependency (not packaged on robostack/conda-forge); the ROS glue is first-party and small — no py_trees_ros. Contains:
- `task_server.py` — node hosting the mission trees: subscribes `task/command` (String), publishes `task/status`, ticks the active tree on a timer, and dispatches on the command's first word (`fetch`/`goto`; unknown words rejected). Commands: `fetch <target> <drop_zone>` — a target matching a zone name drives there, anything else is an open-vocabulary label for the L2 detector (underscores→spaces); `goto <zone>` — drive to any named zone. Zone names → map poses come from a zones YAML resolved via Sites (active floor, then legacy `~/.mote/zones.yaml`, then the committed `config/zones.default.yaml` whose poses match `mote_world.sdf`; in the sim, `sim_launch.py` passes the loaded world's own zones file instead). `save_zone` (`pixi run save-zone <name> [--radius R]`) teaches zones from the live robot pose.
- `zones.py` — the one named-place concept: `load_zones` returns `{name: Zone(name, pose, footprint)}` from the `zones:` section. A **zone** is a pose the robot navigates to (both fetch waypoints *and* goto targets resolve against this single table); it *optionally* carries an area **footprint** — a `radius` circle or a `polygon` of explicit vertices — that's just optional metadata, not a second type. `zones.containing(zones, x, y)` is the "which zone am I in" membership query (nearest-pose first) over zones that have a footprint; `goto` itself only needs the pose. A polygon may be concave (ray-cast membership, so an L-shaped ward or a corridor stretch works where a circle can only under- or over-cover: the hospital wards are 4.7x5.6 m, of which a `radius: 1.5` circle claimed 7.1 m² of 26.5 m²), wins over a `radius` if a zone carries both, and — since polygons come from post-processing a map rather than from driving — may omit `x`/`y`, in which case the loader derives a pose guaranteed to lie inside the outline. `save-zone` therefore preserves an existing footprint when it re-teaches a pose; `--radius` is the explicit way to replace one. Auto-segmenting a saved map into room polygons is the tracked follow-up.
- `behaviours/` — `DriveTo` (Nav2 NavigateToPose action client as a behaviour; cancels in-flight goals on preemption), `AcquireObject` (label missions: publishes the label to `detect/labels`, waits for a matching `detected_objects` detection, writes a standoff goal — 0.4 m short of the object, facing it — to `object_pose`; zone missions pass through), and `TimedStub` (placeholder pick/place until the SO-101 arm is actuated).
- `trees/` — `common.py` (shared `WaitForTask` + the `task` blackboard key), `fetch.py` (wait → acquire object → drive to object → pick stub → drive to drop → place stub; blackboard keys `task`/`object_pose`/`object_label`/`drop_pose` are the seam between the command grammar and perception), and `goto.py` (wait → drive to the zone's pose; success == Nav2 success).
- `test/` — mock-`navigate_to_pose` tree ticks (`test_fetch_tree.py`, `test_fetch_object.py` against a mock detector, `test_goto_tree.py`) plus pure parser/loader tests (`test_parse_command.py`, `test_goto_command.py`, `test_zones.py` — which covers zone footprints and `containing`), no Gazebo/Nav2 needed, run by `pixi run test`.

### `mote_arm` (Python/ament)
SO-101 **follower** arm bring-up (synced to the Pi). There is no leader arm.
Uses **direct Feetech control** (not LeRobot): the arm servos are the same
STS-class Feetech bus as the drive wheels, so it reuses the servo stack rather
than pulling `torch` onto the lean Pi env — the sole new dep is the pure-Python
`feetech-servo-sdk` (`scservo_sdk`). All arm config (port, baud, servo IDs,
per-joint soft limits, home offsets, direction) lives in `robot.yaml`'s `arm:`
section. Contains:
- `config.py` — parses `arm:`; encoder<->radian conversion + soft-limit clamping
  (ROS-free, unit-tested in `test/`).
- `bus.py` — `FeetechBus`, a thin `scservo_sdk` wrapper (lazy import so
  build/lint/test stay hardware-free); register map matches `mote_hardware`.
- `arm_driver` (node, `pixi run arm`) — the **single bus owner**: publishes
  `/joint_states` for the arm, accepts absolute goals on `arm/goal`
  (soft-clamped), exposes `arm/set_torque` (`std_srvs/SetBool`). Starts **limp**
  and goes limp on shutdown — nothing moves without an explicit command.
- `jog` (CLI, `pixi run arm-jog`) — interactive per-joint jog; a *client* of the
  driver (publishes clamped `arm/goal`, torque-off on exit). No bus contention.
- `arm_check` (`pixi run arm-check`) — standalone read-only enumeration/health
  + `--save-home` calibration snapshot. Run with the driver stopped (same port).
- `poses.py` + `arm_pose` (`pixi run arm-pose`) — teach/replay named poses
  (`~/.mote/arm_poses.yaml`, `MOTE_HOME`-overridable), the arm's analogue of
  `save-zone`. **The committed soft limits are the envelope of physically vetted
  poses** (`arm-pose limits`), not guesses; `go` refuses moves over
  `--max-travel`. Changing `home` invalidates stored poses.
- The arm links/joints are added to `mote.urdf.xacro` behind an `arm:=true`
  default (the sim passes `arm:=false`); joint names match `robot.yaml` and
  `/joint_states` so robot_state_publisher animates the arm in TF.
- **The arm shares the drive-wheel bus** (verified: arm IDs 1-6, wheels 7/9, all
  on `/dev/mote_servos`), so it needs no udev rule. Two guards enforce this
  rather than merely documenting it: `config.py` rejects an arm ID colliding
  with a wheel ID on a shared port, and `bus.py` refuses to open a port another
  process already holds (naming the PID). Consequence: the arm driver cannot run
  concurrently with the robot base — stop it first (`pixi run kill`). Lifting
  that means folding arm control into `mote_hardware`'s ros2_control
  `SystemInterface` so one process owns the bus.
- Torque policy, control interfaces, and calibration in `mote_arm/README.md`;
  the human bench runbook in `mote_arm/BENCH.md`.
- `arm_gains` (`pixi run arm-gains show|apply`) — the servos' position-loop
  gains live in EEPROM, i.e. invisible config a servo swap would silently
  revert, so `robot.yaml`'s `arm.gains` is the source of truth and this tool
  reconciles hardware with it. The arm shipped `Kp=16`, which left permanent
  droop under load (the servo settles where `Kp x error` balances the holding
  torque; `Ki=0` never integrates it away). Measured on elbow at -0.200 rad:
  Kp=16 -> error 0.071 at load 196/1000; Kp=32 -> error 0.033 at load 176 —
  error halves as Kp doubles at ~constant load, so it was droop, NOT torque
  saturation, and the 5 V supply was never the binding constraint. **Kp=32
  (wheel/STS3215 default) is applied**; the arm now completes the full 3.19 rad
  home<->reachy move both ways with 0.02-0.06 rad residual. Gotcha: an EEPROM
  read-back races the relock — wait ~150 ms and read twice, or a single read can
  return a garbled 250 and make a successful write look failed.
- **Physical note (GitHub #2):** the camera doesn't fit with the arm attached —
  an unresolved mechanical clash, tracked separately, not addressed here.
  `mote_arm` is not part of the mission bringup; run it explicitly.

### Third-party submodules (`third_party/`)
- `sllidar_ros2` — SLAMTEC RPLIDAR C1 ROS 2 driver
- `kinematic_icp` — kinematic-ICP LIDAR odometry (reads raw wheel odom TF)

## Device Naming

Hardware is addressed by stable symlinks created by `mote_bringup/udev/99-mote.rules` (matched by USB vendor/product ID):
- `/dev/mote_servos` — Waveshare Serial Bus Servo Driver Board
- `/dev/mote_lidar` — SLAMTEC RPLIDAR C1
- `/dev/mote_camera` — USB webcam

With multiple identical USB-serial adapters, pin by serial number — see comments in the rules file.

## Environment

pixi activates `install/setup.sh` and sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` automatically. Dependencies come from the `mote` prefix.dev channel, robostack-jazzy, and conda-forge. The default environment is what runs on the robot (a Raspberry Pi, deployed with `pixi run sync`); the `dev` feature adds `ros-jazzy-desktop` and the `rviz` task.

**DDS scoping.** The `sim` environment additionally sets `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, so sims and benchmarks are invisible to the LAN and to each other's machines; `bench.py` claims a free `ROS_DOMAIN_ID` + `GZ_PARTITION` per invocation so two runs on one machine stay separate. Discovery visibility is one-way: a `LOCALHOST` participant still finds same-host default-range ones, but not vice-versa — hence `pixi run rviz-sim` (RViz joined to the sim's host-local graph) alongside `pixi run rviz` (default range, for the robot). The robot itself keeps LAN discovery and is tuned instead by `config/cyclonedds.xml` (systemd only).

Non-code directories: `design/` holds the BOM (`design/BOM.md`) and CAD files (step/stl/3mf); `docs/images/` holds README photos and the logo (webp).

## Verification note

xacro generation (and therefore all URDF/config changes) can be verified on a workstation with `pixi run -- xacro install/mote_description/share/mote_description/urdf/mote.urdf.xacro`. Controller param injection can also be verified on a workstation by running ros2_control_node against the xacro output with the plugin swapped to `mock_components/GenericSystem` (plus robot_state_publisher to publish the description topic), then `ros2 param get /diff_drive_controller wheel_separation`. Actual motion requires the Pi with hardware connected.
