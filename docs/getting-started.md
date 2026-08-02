# Getting started

This is the path from a box of parts to a robot that maps a room and drives it.
Steps 1 and 2 run anywhere; from step 3 on you need the hardware, so they run on
the Pi.

If you want to try the software before building anything, skip to
[Simulation](simulation.md) — it needs no hardware and runs the same launch
files.

## Prerequisites

- **The robot, assembled.** Printing, assembly and wiring are covered in
  [Printing & assembly](hardware/assembly.md) and
  [Wiring & power](hardware/wiring.md); the parts list is
  [the BOM](hardware/bom.md).
- **A Raspberry Pi 5 running 64-bit Linux.** Ubuntu 24.04 and Raspberry Pi OS
  Bookworm both work — the
  [Raspberry Pi Imager](https://www.raspberrypi.com/software/) route is
  *Raspberry Pi OS (other)* → *Raspberry Pi OS Lite*. To hand a clean Pi its
  identity, network and build in one shot instead, render a cloud-init image
  with `pixi run provision` (see the [fleet runbook](fleet/README.md)).
- **[pixi](https://pixi.sh) installed.** Everything else — ROS 2 Jazzy, Nav2,
  the drivers — is pulled in by pixi. There is no system ROS install.

## 1. Clone

```bash
git clone --recurse-submodules https://github.com/ClachDev/Mote
cd Mote
```

Forgot `--recurse-submodules`? `pixi run submodules` fetches them afterwards.

## 2. Build

```bash
pixi run build
```

Artifacts land in `build/`, `install/` and `log/`, all git-ignored. If a build
complains about `CMakeCache.txt` naming the wrong source directory (a path
rename, usually), delete `build/` and build again.

Built on your workstation? Copy it to the Pi with `pixi run sync`, or
`pixi run -e dev sync-watch` to keep it copying on every save. Launch files,
config and Python go live without a rebuild (`colcon --symlink-install`); only
C++ needs `pixi run build` again on the Pi.

## 3. Set up the Pi

One time, on the robot:

```bash
pixi run setup
```

That installs the udev rules that give the hardware stable names
(`/dev/mote_servos`, `/dev/mote_lidar`, `/dev/mote_camera`), disables WiFi power
save, and installs the systemd units. The units are installed but **not
enabled** — autostart would flatten the battery on a desk. Opt in when the
robot lives on the floor:

```bash
sudo systemctl enable --now mote-bringup mote-health
```

See [Bringup & reliability](robot/bringup.md) for what those services do, and
what the pre-flight self-check refuses to start without.

## 4. Configure the servos

The drive wheels are expected at servo IDs **7** (left) and **9** (right) at
1 Mbaud. Fresh Feetech STS3215 servos all ship as ID 1, so assign IDs before
first use, connecting one servo at a time when prompted:

```bash
pixi run setup-ids
```

The [servo bus tools](hardware/servo-tools.md) cover the rest of the bus
utilities — pinging, swapping IDs, and calibrating `velocity_scale`.

!!! note "The arm is on the same bus"
    An SO-101 arm's six servos share the wheels' bus at IDs 1–6, so it needs no
    extra electronics — and no udev rule of its own. A serial port has no
    kernel-level exclusion, so exactly one process may hold it, and that process
    is the controller manager. See [the arm docs](arm/index.md).

## 5. Map a space

The stack runs in two phases: build a map with SLAM, then drive it without.

```bash
pixi run mapping   # bringup + SLAM
pixi run teleop    # in another terminal: drive it around
pixi run save-map  # save the map + posegraph into the active site
```

Instead of driving it yourself, `pixi run explore` covers the space
autonomously — run it *on the Pi*, so a WiFi drop cannot end the mission.

`save-map` writes an immutable map revision into the active site's floor, runs a
cleaning pass over it, and validates it. [Sites, maps &
zones](robot/sites.md) explains what a site bundle is, why maps are revisions,
and how to teach named places.

## 6. Drive the map

```bash
pixi run robot     # bringup + Nav2 against the saved map
```

Send goals from RViz (`pixi run rviz`, which selects the dev environment), or connect a Foxglove desktop to
`ws://<robot-id>:8765` — the bridge is part of the base bringup, and the shipped
[layout](robot/foxglove.md) has teleop, a map view and the pause control. Teleop
always wins over Nav2 without cancelling the mission; the
[drive path](robot/bringup.md#drive-path--who-gets-the-wheels) explains the
arbitration.

Once places are named, missions become commands:

```bash
pixi run tasks     # the behaviour-tree task layer, alongside `pixi run robot`
```

```bash
pixi run -- ros2 topic pub --once /task/command std_msgs/msg/String \
  "{data: goto kitchen}"
```

See [Missions](robot/missions.md) for the `fetch` and `goto` grammar.

**At this point you have a working robot stack: maps, SLAM, Nav2, teleop and
missions.** From here:

- [Perception](perception/index.md) turns the webcam into obstacles Nav2 can
  see and objects a mission can fetch — with the heavy models on
  [an inference server](inference-server.md) rather than the Pi.
- [The fleet layer](fleet/README.md) puts several robots behind one dashboard,
  with enrollment, dispatch and a shared map registry.
- [Simulation](simulation.md) runs all of it against Gazebo.

## Contributing

A [pre-commit](https://pre-commit.com/) config handles hygiene checks, shell
linting and Python error checking. Wire it in once per clone:

```bash
pixi run lint-install   # into .git/hooks
pixi run lint           # or across the tree by hand (~1 s)
```

To work on these docs:

```bash
pixi run docs           # live-reloading site on http://127.0.0.1:8000
pixi run docs-build     # what CI runs, with --strict
```

AI *aided* contributions are welcome, but only if you can explain and vouch for
every change.
