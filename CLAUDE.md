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
pixi run robot          # mote_launch + slam together
pixi run save-map       # Save current map to ~/.mote/map
pixi run teleop         # Keyboard teleoperation
pixi run sync           # rsync project to Pi at SSH host 'mote'
pixi run udev           # Install udev rules (needs sudo)
pixi run setup-ids      # Guided servo ID assignment tool
pixi run clean          # Kill stale ROS processes and reset daemon

# Dev environment only (installs ros-jazzy-desktop)
pixi run rviz           # RViz2 with mote config
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

Mote is a differential-drive robot built on **ROS 2 Jazzy**, managed entirely through pixi (no system ROS install required). Three first-party packages:

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
Launch files, config, udev rules, and systemd services.

**Launch hierarchy:**
- `robot_launch.py` — combines `mote_launch.py` + `slam_launch.py`
- `mote_launch.py` — main bringup: robot_state_publisher, ros2_control_node, controller spawners, sllidar, laser_filter, v4l2_camera, and `localization_launch.py`. Reads `robot.yaml` for wheel geometry (injected into DiffDriveController params) and sensor config.
- `localization_launch.py` — AMCL-based localization
- `slam_launch.py` — slam_toolbox
- `nav2_launch.py` — Nav2 stack
- `rviz_launch.py` — RViz2 (dev environment only)

**Config files** (`mote_bringup/config/`):
- `controllers.yaml` — controller_manager update rate, DiffDriveController settings (wheel geometry is injected from `robot.yaml` at launch time, not stored here — the launch file writes it to a temp params file keyed by node name, since a plain dict would never reach the controller node)
- `laser_filters.yaml` — filters lidar blind spots
- `nav2_params.yaml` — Nav2 parameters
- `slam_toolbox_params.yaml` — SLAM toolbox parameters
- `mote.rviz` — RViz2 display config

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
