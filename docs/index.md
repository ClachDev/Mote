# Mote

![Mote with camera](images/mote_camera.webp)

Mote is a differential-drive robot built to be as cheap as it can be while
still running a real ROS 2 stack: SLAM, Nav2, a behaviour-tree task layer, a
monocular perception pipeline, an [SO-101](https://github.com/TheRobotStudio/SO-ARM100)
arm, and a small fleet control plane. The [project README](https://github.com/ClachDev/Mote#readme)
says why it exists; this site is the depth behind it.

Everything is managed with [pixi](https://pixi.sh) — no system ROS install on
the Pi or on your workstation.

## Start here

<div class="grid cards" markdown>

- **[Getting started](getting-started.md)** — prerequisites, build, one-time Pi
  setup, and your first map and drive.
- **[Hardware](hardware/index.md)** — design decisions, the bill of materials,
  printing and assembly, wiring and power.
- **[Simulation](simulation.md)** — the same launch files against Gazebo, on a
  world ladder, with no hardware at all.
- **[Fleet](fleet/README.md)** — the operator runbook: overlay, enrollment,
  dashboard, dispatch, and the map registry.

</div>

## The stack

| Package | What it does |
| --- | --- |
| [`mote_bringup`](robot/bringup.md) | Launch files, site management, health monitoring, provisioning |
| [`mote_description`](https://github.com/ClachDev/Mote/tree/main/mote_description) | URDF robot model and `robot.yaml`, the single source of hardware config |
| [`mote_hardware`](hardware/servo-tools.md) | The `ros2_control` interface, and sole owner of the servo bus |
| [`mote_nav`](https://github.com/ClachDev/Mote/tree/main/mote_nav) | Nav2 plugins: the wheel-speed critic and the ICP odometry gate |
| [`mote_perception`](perception/index.md) | Depth and open-vocabulary detection from one cheap webcam |
| [`mote_tasks`](robot/missions.md) | The behaviour-tree task layer — `fetch` and `goto` |
| [`mote_arm`](arm/index.md) | The SO-101 follower arm: calibration, jogging, taught poses |
| [`mote_fleet`](fleet/package.md) | Fleet control plane: server, operator dashboard, map registry |
| [`mote_simulation`](simulation.md) | Gazebo sim, a world ladder, benchmarks, and the smoke test |

## How the docs are organised

Reference material that belongs to a package lives *with* that package in the
repo, and is copied onto this site at build time — so a page here and the file
a contributor edits are never two different documents. The pencil icon at the
top of a page opens the file it came from.

Design notes, verification ledgers and tuning runs record *why* something is
the way it is, and what was measured. They are kept rather than rewritten: the
[fleet architecture](design/fleet.md) note and the
[verification ledgers](fleet/m0-verification.md) beside it are the honest
record of what was and was not proven.
