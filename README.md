# Mote

![Mote with camera](docs/images/mote_camera.webp)

## *Mote*vation (I'm sorry I had to!)

While working on some libraries I really needed a simple robot platform to test
them out on. There are some existing platforms but they are either too expensive
(turtlebot 3), or much too expensive (turtlebot 4). Some are cheap but lack
sensors (LeKiwi).

When I started out in roboticss there was a [$50
robot](https://www.societyofrobots.com/step_by_step_robot.shtml) project I
followed. That was made for a different age but I figured why not see how
cheaply I can make a fully functioning robotics platform for todays enthusiasts.

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

See [`design/`](design/) for hardware design decisions, requirements, and bill
of materials.

## Hardware

The main hardware components are below with the Raspberry Pi and the battery
being the biggest cost factors (see [the BOM](design/BOM.md) for the full
hardware list).

- Raspberry Pi 5 (4GB) - Linux so we can run ROS, 4GB because memory is crazy expensive these days
- 5V USB-C power bank (slim form factor, ≥85W dual output) - Easy, cheap, and simple to integrate power supply
- 2× Feetech STS3215 servo - this simplifies our logic and standardises on components used by the [S0-101 arm](https://github.com/TheRobotStudio/SO-ARM100)
- Waveshare Serial Bus Servo Driver Board - Needed to connect servos to the pi.
  If using the S0-101 arm you can share a single board.
- SLAMTEC RPLIDAR C1 - The cheapest LIDAR I could find.
- USB webcam - Need some vision for LeRobot to function. Also helps with teleoperation.

I've tried to keep as many of the components 3D printed as possible to keep it
accessible. In theory some parts of the chassis can be CNC'd but I don't have
the ability to test and iterate on that right now.

## Software

Built with ROS2 Jazzy, managed via [pixi](https://pixi.sh). This gives us a nice
way to package everything up without worrying about ecosystem concerns.

| Package                                 | Purpose                                                   |
| --------------------------------------- | --------------------------------------------------------- |
| [`mote_bringup`](mote_bringup/)         | Launch files for bringing up the robot                    |
| [`mote_description`](mote_description/) | URDF robot model and TF tree                              |
| [`mote_hardware`](mote_hardware/)       | ros2_control hardware interface for the Feetech servo bus |

I'm trying to keep all dependencies from
[Robostack](https://robostack.github.io/index.html) or `conda-forge`. Anything
else belongs as a git submodule moving to either `conda-forge` or the
`prefix.dev/mote` channel once condafied/pixified.

## Getting Started

Clone the repo with submodules:

```bash
git clone --recurse-submodules https://github.com/ClachDev/Mote
```

Install [pixi](https://pixi.prefix.dev/) if you don't have it, then build:

```bash
pixi run build
```

Launch:

```bash
pixi run launch # Launches the base
pixi run rviz   # Runs rviz to view the map and navigate
# and
pixi run slam   # Runs the SLAM stack to create a map
# or
pixi run nav    # Runs the nav stack
```

I still need to work out the "deploy" pipeline. I'm currently using git and
rsync to move the code to the Pi and build it there. I want to try using [pixi
pack](https://pixi.prefix.dev/latest/deployment/pixi_pack/) but I haven't had a
chance yet.

## SO-101 Follower Arm

![Mote with SO-101 arm](docs/images/mote_S0_101.webp)

The chassis is compatible with the [SO-101 follower
arm](https://github.com/TheRobotStudio/SO-ARM100) via the ORP mounting grid and
a custom base. See the SO-ARM100 project for the arm's BOM and assembly
instructions.

My long term goal is to eventually have Mote able to explore a space and tidy
things up off the floor [obligatory xkcd](https://xkcd.com/1425/).

## Contributions

This project is still in its early stages and I'm happy to accept contributions
of any kind. AI _aided_ contributions are also welcome but only if you can explain
and vouch for every change!

## Sponsorship

If you want to help me test new sensors or components to lower the cost even
further please consider sponsoring the project and I'll recognise you or your
company here!
