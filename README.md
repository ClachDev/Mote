# Mote

[![build](https://github.com/ClachDev/Mote/actions/workflows/build.yml/badge.svg)](https://github.com/ClachDev/Mote/actions/workflows/build.yml)

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
and [ORP](https://openroboticplatform.com/) which wants interoperability. Apart
from allowing me to test algorithms, I also want Mote to be a comparison
platform between classical ROS/Nav2 navigation and learned policies via
[LeRobot](https://github.com/huggingface/lerobot).

Getting a cheap robot to drive is only the start, though — most of this repo is
about everything that comes after: versioned maps, taught zones, fetch missions,
remote operation, and running more than one robot without SSH-ing into each.

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
| [`mote_bringup`](mote_bringup/)         | Launch files, site/map/zone management, health monitoring, self-check, provisioning |
| [`mote_description`](mote_description/) | URDF robot model and `robot.yaml`, the single source of truth for hardware config |
| [`mote_hardware`](mote_hardware/)       | ros2_control hardware interface for the Feetech servo bus |
| [`mote_nav`](mote_nav/)                 | Nav2 plugins (wheel-speed critic) and the odometry TF relay |
| [`mote_perception`](mote_perception/)   | Camera depth obstacles for Nav2 + open-vocabulary object detection |
| [`mote_tasks`](mote_tasks/)             | Behaviour-tree task layer: `fetch` and `goto` missions on top of Nav2 |
| [`mote_arm`](mote_arm/)                 | SO-101 arm driver, jog, guided calibration, and taught poses |
| [`mote_fleet`](mote_fleet/)             | Fleet control plane: robot agent, fleet server, operator dashboard, map registry |
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
`pixi run sync` (see [Deploying to the Pi](#deploying-to-the-pi)).

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

The IDs, baud rate and `velocity_scale` (the rad/s → raw servo-unit conversion)
live in [`mote_description/config/robot.yaml`](mote_description/config/robot.yaml),
which is the single source of truth for all robot configuration. Both the URDF
and the launch file read from it. The helper tools in
[`mote_hardware/tools/`](mote_hardware/tools/) round this out (`servo_debug`,
`velocity_cal`, `swap_ids`).

### 5. Map, then drive

```bash
pixi run mapping   # bringup + SLAM: drive around (teleop or nav goals) to build a map
pixi run save-map  # save the map + posegraph into the active site
pixi run robot     # bringup + Nav2: drive the saved map autonomously
pixi run teleop    # keyboard teleoperation, any time
```

Maps live in **site bundles** under `~/.mote/sites/` — one per floor, holding
map revisions, the SLAM posegraph (so a map can be *extended* later instead of
remapped), and named zones. Teach a zone by driving there and naming it
(`pixi run save-zone kitchen`), or let `segment-map` propose one per room
straight off the map. [`mote_bringup`](mote_bringup/) has the details.

### 6. Missions

The task layer ([`mote_tasks`](mote_tasks/)) runs behaviour-tree missions on
top of Nav2. Start it with `pixi run tasks` alongside the robot, then send
commands (from the fleet dashboard, `fleetctl dispatch`, or the `task/command`
topic):

```text
goto kitchen              # drive to any named zone
fetch red_mug dropoff     # find "red mug" by open-vocabulary detection,
                          # drive to it, and carry it to the dropoff zone
```

Object fetching uses the camera and an open-vocabulary detector — no training,
just a label ([`mote_perception`](mote_perception/), below).

## Fleet: run more than one (optional)

![Fleet dashboard](docs/images/fleet-ui.webp)

A robot works standalone with nothing below, but the interesting part starts
when you stop SSH-ing into robots. Every machine joins a
[Tailscale](https://tailscale.com/) overlay, so "same LAN" becomes "same
tailnet" with nothing exposed to the internet, and a fleet server hands out
identities and carries telemetry and tasks over MQTT. A robot runs
`pixi run enroll` once to be allocated an id (`mote-01`, ...), then
`pixi run agent` to bridge it to the fleet.

From there the dashboard (above) shows every robot live on its floor's map —
presence, health, pose, current task — and dispatches missions; a
[Foxglove](https://foxglove.dev/) remote console gives camera, lidar, TF, and
teleop from anywhere on the tailnet; and a map registry reviews robots' saved
maps and distributes promoted revisions to every robot on the floor. A blank
SD card can even be provisioned unattended into an enrolled, navigating robot.

The runbook — broker and server setup, and every command — is
[`docs/fleet/README.md`](docs/fleet/README.md); the design and milestones are
[`docs/design/fleet.md`](docs/design/fleet.md).

## Simulation (no hardware required)

A Gazebo (gz-sim) simulation of Mote runs entirely on a workstation — same
controllers, same scan pipeline, and crucially the *same mission launch files*
as the real robot, so nothing can drift between sim and hardware. The sim
dependencies live in a separate pixi environment so the robot install stays
lean:

```bash
pixi run sim            # headless gz + robot + controllers
pixi run sim-mapping    # the real mapping mission, in sim
pixi run sim-nav        # the real nav mission against the world's saved map
pixi run sim-test       # ~20 s headless smoke test (drive + odom + scan + map)
pixi run -e sim -- gz sim -g   # optional: attach the Gazebo GUI
```

The worlds in `mote_simulation/worlds/` form an easy-to-hard ladder, selectable
with `world:=` (e.g. `pixi run sim-nav world:=hospital_world.sdf`):

- **`mote_world.sdf`** (easy) — a simple walled room; the default and the
  `sim-test` world.
- **`office_world.sdf`** (medium) — a corridor of identical rooms that
  stresses localisation.
- **`hospital_world.sdf`** (hard) — a ~58×38 m hospital with a looping
  corridor grid, ~50 rooms, and clutter.

Each world ships with a committed site bundle (its own SLAM-built map and
zones), so `sim-nav` and `goto <zone>` work out of the box on all three.
`sim-test` needs a working render backend (a GPU or fast software GL), so it's
a local pre-PR gate rather than a hosted-CI job.

## Perception

The single cheap webcam earns its keep twice
([`mote_perception`](mote_perception/)):

- **Depth obstacles** — monocular depth, rescaled against the lidar per frame,
  feeds Nav2 a point cloud of the low obstacles the lidar plane can't see.
- **Open-vocabulary detection** — "red mug" becomes a map pose for the fetch
  mission, with no training.

Both run torch-free on the Pi and call a GPU inference server elsewhere on the
network — workstation, gaming PC, or cloud
([`docs/inference-server.md`](docs/inference-server.md)). If the server is
unreachable the robot warns and navigates on lidar alone.

![Detections grounded to the floor](docs/images/perception_detection_vs_floor.webp)

## Deploying to the Pi

Day to day I develop on a workstation and push to the Pi with rsync: the
`sync` task targets an SSH host named `mote` — change the host in
[`pixi.toml`](pixi.toml) to match your Pi, then `pixi run sync` (or
`sync-watch` to push on every save).

For pushing finished builds to one or more robots, the direction is versioned
packages on the `prefix.dev/mote` channel (built with
[`pixi-build-ros`](https://pixi.prefix.dev/latest/build/ros/)) so a robot just
needs `pixi install` — no source checkout or compile on the bot. That work is
in progress.

## SO-101 Follower Arm

![Mote with SO-101 arm](docs/images/mote_SO_101.webp)

The chassis is compatible with the [SO-101 follower
arm](https://github.com/TheRobotStudio/SO-ARM100) via the ORP mounting grid and
a custom base (see the SO-ARM100 project for the arm's BOM and assembly). The
arm shares the drive wheels' servo bus, so it needs no extra electronics — and
it's driven, not just mounted: [`mote_arm`](mote_arm/) covers the driver,
per-joint jogging, guided calibration that measures each joint's real travel,
and taught named poses, the arm's analogue of zones.

My long term goal is to eventually have Mote able to explore a space and tidy
things up off the floor [obligatory xkcd](https://xkcd.com/1425/).

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

## Sponsorship

If you want to help me test new sensors or components to lower the cost even
further please consider sponsoring the project and I'll recognise you or your
company here!
