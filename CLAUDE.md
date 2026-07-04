# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Common Commands

All tasks are run via [pixi](https://pixi.sh). Never invoke `colcon` or `ros2` directly — always use `pixi run <task>`.

```bash
pixi run build          # Build all packages with colcon + Ninja
pixi run submodules     # Fetch git submodules (sllidar_ros2, kinematic_icp)
pixi run launch         # Full robot bringup (hardware + lidar + camera + localization)
pixi run slam           # SLAM stack only (run alongside launch)
pixi run nav            # Nav2 stack (requires a saved map at ~/.mote/map.yaml)
pixi run mapping        # bringup + SLAM together (build/extend a map)
pixi run robot          # bringup + Nav2 together (drive a saved map; needs ~/.mote/map.yaml)
pixi run save-map       # Save current map to ~/.mote/map
pixi run teleop         # Keyboard teleoperation
pixi run sync           # rsync project to Pi at SSH host 'mote'
pixi run setup          # One-time Pi setup: udev + wifi-powersave + systemd (needs sudo)
pixi run udev           # Install udev rules + dialout group (needs sudo)
pixi run wifi-powersave # Disable WiFi power save via NetworkManager (needs sudo)
pixi run setup-ids      # Guided servo ID assignment tool
pixi run kill           # Kill stale ROS processes and reset daemon

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
Launch files, config, udev rules, NetworkManager drop-ins, and systemd services.

**Launch hierarchy:** the two mission launches (`mapping_launch.py`, `robot_launch.py`) each take a `base` arg (default true) that includes the hardware base, and a `use_sim_time` arg they forward to everything they include. The sim runs these *same* files with `base:=false`, supplying a Gazebo base in place of the drivers — so the missions are defined once and the sim exercises the real launch files.
- `robot_launch.py` — nav mission: `mote_launch.py` (if `base`) + `nav2_launch.py` (drive a saved map). Forwards a `map` arg, defaulting to `~/.mote/map.yaml`.
- `mapping_launch.py` — mapping mission: `mote_launch.py` (if `base`) + `slam_launch.py` + `nav2_launch.py` (`localisation:=false`): build/extend a map with SLAM *and* drive to goals autonomously while doing so.
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

### `mote_simulation` (Python/ament)
Workstation-only Gazebo simulation, kept separate from `mote_bringup` so it can be excluded from the robot sync (`pixi run sync` skips `mote_simulation/`). Built only in the `sim` pixi environment. Contains:
- `launch/sim_launch.py` — Gazebo sim: headless gz server, robot spawn, ros_gz bridge (/clock, /scan), controllers, laser_filter, and the shared `localization_launch.py`. Takes a `world:=` arg (file in `mote_simulation/worlds/`, default `mote_world.sdf` — the simple smoke-test room; `office_world.sdf` is a medium hospital-ward corridor for stress-testing localisation; `hospital_world.sdf` is the hard tier — a ~58x38 m looping hospital, generated by `worlds/gen_hospital.py`). The URDF is processed with `use_sim:=true`, which swaps `MoteHardware` for `gz_ros2_control` and adds a simulated lidar (specs from `robot.yaml` `lidar.sim`). Without that flag the xacro output is unchanged. Controller params are merged into one temp file (gz_ros2_control loads a single `<parameters>` file referenced in the URDF). It pulls `controllers.yaml`, `laser_filters.yaml`, and `localization_launch.py` from `mote_bringup`'s share so the sim and the real robot can't drift apart. It asserts `use_sim_time:=true` for the whole process tree via `SetParameter`. A `mode:=mapping|nav` arg includes the real `mapping_launch.py` / `robot_launch.py` with `base:=false` (default `none` = sim only): the sim provides the base and *delegates* the mission to the actual launch files, so it can't re-encode or drift from them, and `pixi run sim-mapping` / `sim-nav` put those mission launches under test. (`pixi run mapping`/`robot` are the hardware entry points — same files, `base` defaulting true, wall-clock time.) The dependency direction stays one-way: `mote_simulation` includes from `mote_bringup`, never the reverse.
- `worlds/` — an easy->hard ladder: `mote_world.sdf` (easy smoke-test room), `office_world.sdf` (medium hospital-ward corridor), and `hospital_world.sdf` (hard ~58x38 m looping hospital with ~50 rooms and clutter). The hard world is generated by `worlds/gen_hospital.py` (committed alongside its output — edit the script's layout and regenerate rather than hand-editing the SDF).
- `test/sim_smoke/` — `run_sim_smoke.sh` + `verify_sim.py`, the `pixi run sim-test` gate.

### `mote_perception` (Python/ament)
Home for camera-derived perception nodes — **L0 (Foundation)** of the vision pipeline. Runs on the robot (it will eventually feed Nav2), so unlike `mote_simulation` it is synced to the Pi. Contains:
- `mote_perception/camera_monitor.py` — a dependency-light camera health monitor (rclpy + sensor_msgs only, no OpenCV). Subscribes to `image` and logs measured frame rate, resolution, and encoding on a timer, warning on dropouts. Registered as the `camera_monitor` console_script; it is the template later perception nodes follow.
- `launch/perception_launch.py` — declares `use_sim_time` (applied via `SetParameter`) and starts `camera_monitor` with `image` remapped to `/image_raw`. Marked as the extension point where L1 nodes (rectify, depth, detection) attach.
- `config/` — camera-calibration home. `camera_info.default.yaml` is a committed fallback calibration for the UGREEN webcam; a per-robot `~/.mote/camera_calibration.yaml` (outside the repo) overrides it. `mote_launch.py` prefers the `~/.mote` file when present, else `robot.yaml`'s `camera.default_info_url`, passing the result to `v4l2_camera_node` as `camera_info_url`. `config/README.md` documents when/how to calibrate (with the printable checkerboard).
- Compressed transport is already provided by the `image-transport-plugins` dep, so the camera publishes `/image_raw/compressed`; off-board/RViz consumers should prefer it. See `mote_perception/README.md`.

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

Non-code directories: `design/` holds the BOM (`design/BOM.md`) and CAD files (step/stl/3mf); `docs/images/` holds README photos and the logo (webp).

## Verification note

xacro generation (and therefore all URDF/config changes) can be verified on a workstation with `pixi run -- xacro install/mote_description/share/mote_description/urdf/mote.urdf.xacro`. Controller param injection can also be verified on a workstation by running ros2_control_node against the xacro output with the plugin swapped to `mock_components/GenericSystem` (plus robot_state_publisher to publish the description topic), then `ros2 param get /diff_drive_controller wheel_separation`. Actual motion requires the Pi with hardware connected.
