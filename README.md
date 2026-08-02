# Mote

[![build](https://github.com/ClachDev/Mote/actions/workflows/build.yml/badge.svg)](https://github.com/ClachDev/Mote/actions/workflows/build.yml)
[![docs](https://github.com/ClachDev/Mote/actions/workflows/docs.yml/badge.svg)](https://clachdev.github.io/Mote/)

**📖 [Documentation](https://clachdev.github.io/Mote/)** — this README is the
tour; the site is the manual.

![Mote with camera](docs/images/mote_camera.webp)

## *Mote*vation (I'm sorry I had to!)

While working on some libraries I really needed a simple robot platform to test
them out on. There are some existing platforms but they are either too expensive
(turtlebot 3), or much too expensive (turtlebot 4). Some are cheap but lack
sensors (LeKiwi).

When I started out in robotics there was a [$50
robot](https://www.societyofrobots.com/step_by_step_robot.shtml) project I
followed. That was made for a different age but I figured why not see how
cheaply I can make a fully functioning robotics platform for today's enthusiasts.

The main factors I've engineered for are:

1. it must be as **cheap** as operationally possible - if it isn't affordable the
   project loses purpose.
2. it must be able to run **ROS** - you should be able to use this platform to run a
   normal ROS stack to map and navigate around a home or office.
3. it must be compatible with cutting edge PhysicalAI platforms like **LeRobot** -
   ROS is good but the future requires experimentation
4. it should follow the [Open Robotic
   Platform](https://openroboticplatform.com/designrules) standard - robots are
   more fun if you tinker about and add arms.

I've taken a lot of inspiration from projects like
[LeKiwi](https://github.com/SIGRobotics-UIUC/LeKiwi) which aim to be accessible
and [ORP](https://openroboticplatform.com/) which wants interoperability.

Apart from allowing me to test algorithms, I also want Mote to be a comparison
platform between classical ROS/Nav2 navigation and learned policies via
[LeRobot](https://github.com/huggingface/lerobot).

Finally, I also want to take this project further than just getting a robot running.
I want to use it to explore and act as a reference for what a full robotics
stack might look like: versioned maps, zones, remote operation, and maintaining
a multi-robot fleet.

See [`design/`](design/) for hardware design decisions, requirements, and bill
of materials.

## Hardware

The main hardware components are below with the Raspberry Pi and the battery
being the biggest cost factors (see [the BOM](design/BOM.md) for the full
hardware list).

- Raspberry Pi 5 (4GB) - Linux so we can run ROS, 4GB because memory is crazy expensive these days
- 5V USB-C power bank (slim form factor, ≥85W dual output) - Easy, cheap, and simple to integrate power supply
- 2× Feetech STS3215 servo - this simplifies our logic and standardises on components used by the [SO-101 arm](https://github.com/TheRobotStudio/SO-ARM100)
- Waveshare Serial Bus Servo Driver Board - Needed to connect servos to the pi.
  If using the SO-101 arm you can share a single board.
- SLAMTEC RPLIDAR C1 - The cheapest LIDAR I could find.
- USB webcam - Need some vision for LeRobot to function. Also helps with teleoperation.

I've tried to keep as many of the components 3D printed as possible to keep it
accessible. In theory some parts of the chassis can be CNC'd but I don't have
the ability to test and iterate on that right now.

## Software

Built with ROS2 Jazzy, managed via [pixi](https://pixi.sh). This gives us a nice
way to package everything up without worrying about ecosystem concerns — no
system ROS install needed, on the Pi or your workstation.

| Package                                 | Purpose                                                   |
| --------------------------------------- | --------------------------------------------------------- |
| [`mote_bringup`](mote_bringup/)         | Launch files, site management, health monitoring, provisioning, etc |
| [`mote_description`](mote_description/) | URDF robot model and `robot.yaml` |
| [`mote_hardware`](mote_hardware/)       | ros2_control hardware interface |
| [`mote_nav`](mote_nav/)                 | Nav2 plugins |
| [`mote_perception`](mote_perception/)   | Image processing pipeline (needs a separate "inference" machine) |
| [`mote_tasks`](mote_tasks/)             | Behaviour-tree task layer |
| [`mote_arm`](mote_arm/)                 | SO-101 arm driver |
| [`mote_fleet`](mote_fleet/)             | Fleet control plane: fleet server, operator dashboard, map registry |
| [`mote_simulation`](mote_simulation/)   | Gazebo sim, a world ladder, and the smoke test (workstation only) |

I'm trying to keep all dependencies from
[Robostack](https://robostack.github.io/index.html) or `conda-forge`. Anything
else belongs as a git submodule moving to either `conda-forge` or the
`prefix.dev/mote` channel once condafied/pixified.

## Build & Setup

### Prerequisites

- A Raspberry Pi 5 running 64-bit Linux (Ubuntu 24.04 or Raspberry Pi OS
  Bookworm both work) - set up with [Raspberry Pi
  Imager](https://www.raspberrypi.com/software/) -> Raspberry Pi OS Other ->
  Raspberry Pi OS Lite.
- [pixi](https://pixi.prefix.dev/) installed. Everything else (ROS 2 Jazzy,
  Nav2, drivers) is pulled in by pixi, so you don't need a system ROS install.
- The robot assembled with the servos and sensors wired to the Pi.

**Note:** The build commands (1 and 2) can be run on your developer machine for
testing, but 3 onwards requires the real hardware connected so must be run on
the Pi.

### 1. Clone

```bash
git clone --recurse-submodules https://github.com/ClachDev/Mote
cd Mote
```

(Forgot `--recurse-submodules`? `pixi run submodules` fetches them afterwards.)

### 2. Build

```bash
pixi run build
```

If you run these on your developer machine, sync the code to the Pi with
`pixi run sync`.

### 3. Setup Pi

There are a few setup tasks that must be run on the Pi before first use. These
set up udev rules, systemd services, and other configuration. It should only
need to be run once.

```bash
pixi run setup
```

### 4. Configure the servos

The drive wheels are expected at servo IDs **7** (left) and **9** (right) at
1 Mbaud. Fresh Feetech STS3215 servos ship as ID 1, so you'll need to assign
IDs before first use. Run the guided setup and connect one servo at a time when
prompted:

```bash
pixi run setup-ids
```

### 5. Map, then drive

The stack is split into two phases: an initial SLAM mapping phase and then a
non-SLAM drive phase.

To use SLAM to generate a map:

```bash
pixi run mapping   # bringup + SLAM: drive around (teleop or nav goals) to build a map
pixi run save-map  # save the map + posegraph into the active site
```

With a map saved you can just run the robot stack (no slam):

```bash
pixi run robot     # bringup + Nav2: drive the saved map autonomously
pixi run teleop    # Manual keyboard teleoperation, any time
```

You can also run a Foxglove desktop to connect to the robot over websocket at
`ws://<robot-id>:8765`. The layout is defined in
`mote_bringup/foxglove/mote.json`.

Maps live in **site bundles** under `~/.mote/sites/`.

**Congratulations! At this point you should have a working robot stack: maps,
SLAM, Nav2, and teleop.**

The longer version of all of the above — provisioning a clean Pi, what a site
bundle holds, the systemd services, teaching zones and running missions — is
[Getting started](https://clachdev.github.io/Mote/getting-started/) on the docs
site.

## Simulation (no hardware required)

A Gazebo simulation of Mote runs the same controllers, same scan pipeline, and
crucially the *same launch files* as the real robot, so nothing can drift
between sim and hardware. The sim dependencies live in a separate pixi
environment so the robot install stays lean:

```bash
pixi run sim            # headless gz + robot + controllers
pixi run sim-test       # ~20 s headless smoke test (drive + odom + scan + map)
pixi run -e sim -- gz sim -g   # optional: attach the Gazebo GUI
```

The worlds in `mote_simulation/worlds/` form an easy-to-hard ladder, selectable
with `world:=` (e.g. `pixi run sim world:=hospital_world.sdf`):

- **`mote_world.sdf`** (easy) — a simple walled room; the default and the
  `sim-test` world.
- **`office_world.sdf`** (medium) — a corridor of identical rooms that
  stresses localisation.
- **`hospital_world.sdf`** (hard) — a ~58×38 m hospital with a looping
  corridor grid, ~50 rooms, and clutter.

## Perception

Rather than add a £150 RGB-D camera to the build, I've been trying to make use of a
single cheap webcam plus some smart vision models. The perception stack is
mostly offloaded to a separate "inference server" which runs somewhere with
access to a (cheap) GPU. This allows us to extract depth, tag objects, and
generally navigate as if we had a full 3D camera on board.

- **Depth Anything V2** — monocular depth, rescaled against the lidar per frame,
  feeds Nav2 a point cloud of the low obstacles the lidar plane can't see.
- **Open-vocabulary** — "red mug" becomes a map pose for the fetch
  mission, with no training.

I'm still experimenting with this to see how far we can push it. How the
inference machine is chosen, deployed and scaled — and what the robot does when
it isn't there — is
[The inference server](https://clachdev.github.io/Mote/inference-server/).

## SO-101 Follower Arm

![Mote with SO-101 arm](docs/images/mote_SO_101.webp)

The chassis is compatible with the [SO-101 follower
arm](https://github.com/TheRobotStudio/SO-ARM100) via the ORP mounting grid and
a custom base (see the SO-ARM100 project for the arm's BOM and assembly). The
arm shares the drive wheels' servo bus, so it needs no extra electronics.

I've started integrating it into ROS in [`mote_arm`](mote_arm/), covering the
driver, per-joint jogging, guided calibration that measures each joint's real
travel, and taught named poses.

## Fleet

I'm currently working on the beginnings of a fleet layer based on Tailscale and
MQTT. This is heavily work in progress but here's a little preview:

![Fleet dashboard](docs/images/fleet-ui.webp)

The operator's side of it — overlay, enrollment, the dashboard, dispatch and
the map registry — is the
[fleet runbook](https://clachdev.github.io/Mote/fleet/).


## Contributions

This project is still in its early stages and I'm happy to accept contributions
of any kind. AI _aided_ contributions are also welcome but only if you can explain
and vouch for every change!

A [pre-commit](https://pre-commit.com/) config handles quick hygiene checks,
shell linting (shellcheck) and Python error checking (ruff). Enable it once per
clone, and it runs automatically on commit:

```bash
pixi run lint-install   # wire it into .git/hooks (one time)
pixi run lint           # or run across the whole tree manually (~1 s)
```

The [docs site](https://clachdev.github.io/Mote/) is built from this repo with
mkdocs, and mounts each package's own README rather than copying it — so
documentation is edited beside the code it describes:

```bash
pixi run docs           # live-reloading site on http://127.0.0.1:8000
pixi run docs-build     # what CI runs, with --strict
```

## Sponsorship

If you want to help me test new sensors or components to lower the cost even
further please consider sponsoring the project and I'll recognise you or your
company here!
