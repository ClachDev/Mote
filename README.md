# AuldBot

A differential drive robot platform combining the accessibility of
[LeKiwi](https://github.com/TheRobotStudio/SO-ARM100) with standard ROS2/Nav2
infrastructure and [ORP](https://openroboticplatform.com/) interoperability.
Intended as a comparison platform between classical ROS/Nav2 navigation and
learned policies via [LeRobot](https://github.com/huggingface/lerobot).

See [`design/`](design/) for hardware design decisions, requirements, and bill
of materials.

## Hardware

- Raspberry Pi 5 (4GB)
- 2× Feetech STS3215 servo
- Waveshare Serial Bus Servo Driver Board
- SLAMTEC RPLIDAR C1
- USB webcam
- 5V USB-C power bank (slim form factor, ≥85W dual output)

## Software

Built with ROS2 Jazzy, managed via [pixi](https://pixi.sh).

| Package | Purpose |
|---|---|
| [`auldbot_bringup`](auldbot_bringup/) | Launch files for bringing up the robot |
| [`auldbot_description`](auldbot_description/) | URDF robot model and TF tree |
| [`auldbot_hardware`](auldbot_hardware/) | ros2_control hardware interface for the Feetech servo bus (SDK: **`scservo-linux`** from [`auldbot-forge`](https://prefix.dev/auldbot-forge) via Pixi) |

## Getting Started

Clone the repo with submodules:

```bash
git clone --recurse-submodules <url>
```

Install [pixi](https://pixi.prefix.dev/) if you don't have it, then build:

```bash
pixi run build
```

Launch:

```bash
pixi run launch
```
