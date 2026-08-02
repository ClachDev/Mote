# Simulation

A Gazebo simulation of Mote runs the same controllers, the same scan pipeline
and — crucially — the *same launch files* as the real robot, so nothing can
drift between sim and hardware. It needs no hardware, and its dependencies live
in their own pixi environment (`sim`) so the robot install stays lean.

```bash
pixi run sim            # headless gz server + robot + controllers
pixi run sim-mapping    # the real mapping mission on top of it
pixi run sim-nav        # the real nav mission against the world's saved map
pixi run sim-test       # ~20 s headless smoke test (needs a GPU)
pixi run -e sim -- gz sim -g   # optional: attach the Gazebo GUI
```

The `sim`, `sim-*` and benchmark tasks are defined only in the sim
environment, so pixi selects it for you. An ad-hoc command has to name it:

```bash
pixi run -e sim -- ros2 launch mote_bringup slam_launch.py use_sim_time:=true
```

## How it delegates

`sim_launch.py` supplies a *base* — a Gazebo robot with `gz_ros2_control` in
place of `MoteHardware`, a simulated lidar, and the `/clock` and `/scan`
bridges — and then includes the real mission launch files with `base:=false`:

- `mode:=mapping` includes `mapping_launch.py` (that is `pixi run sim-mapping`)
- `mode:=nav` includes `robot_launch.py` (`pixi run sim-nav`)
- `mode:=none`, the default, is the base alone

So the missions are defined once. The sim also pulls `controllers.yaml`,
`laser_filters.yaml` and `localization_launch.py` straight out of
`mote_bringup`'s share, and the URDF is the same xacro with `use_sim:=true` —
without that flag its output is byte-identical to the robot's. The dependency
runs `mote_simulation` → `mote_bringup`, never the reverse.

## The world ladder

Worlds live in `mote_simulation/worlds/` and get harder in order. Pick one with
`world:=`:

```bash
pixi run sim world:=hospital_world.sdf
```

| World | Tier | What it stresses |
| --- | --- | --- |
| `mote_world.sdf` | easy | A simple walled room. The default, and the smoke test's world. |
| `office_world.sdf` | medium | A corridor of near-identical rooms — localisation. |
| `hospital_world.sdf` | hard | ~58 × 38 m, a looping corridor grid, ~50 rooms, clutter. |

The hospital world is *generated* by `worlds/gen_hospital.py`, which is
committed beside its output — edit the script's layout and regenerate rather
than hand-editing the SDF.

Every world has a sibling `<world>.zones.yaml` carrying the same waypoint zones
(`pickup`, `dropoff`, `home`) plus a few room zones with footprints, so the
[fetch and goto missions](robot/missions.md) run anywhere on the ladder with
matching coordinates. The generator asserts that every zone clears the walls
and the furniture, and lies inside its own footprint.

## Sim maps are real site bundles

`mote_simulation/sim_home/` is a committed, in-repo `MOTE_HOME`: one real
[site bundle](robot/sites.md) per world (site name = world stem, floor
`ground`) holding that world's SLAM map and zones. The sim environment points
`MOTE_HOME` at it, so `sim-nav` loads a world's own map and never touches the
robot's real `~/.mote`.

Those maps are built the same way the robot builds one:

```bash
pixi run sim-map-world <world.sdf> [budget_s]
```

That launches the real mapping mission headless, runs the same autonomous
coverage tool the robot runs (`explore`, with `--sim-time`), and saves into the
world's site. Sim maps are ground-truth clean, so the save keeps the raw
`map_saver` output — the FFT declutter pass is tuned for real sensor noise and
would strip thin true walls. Re-running adds a revision.

## Measuring, not just running

Three harnesses sit on top of the sim:

- **[Benchmark harness](simulation/benchmark.md)** — scores the nav mission
  against Gazebo ground truth into metrics JSON plus a report.
- **[Parameter sweep](simulation/sweep.md)** — runs the benchmark once per
  parameter set and ranks them against the baseline, so a Nav2 parameter is
  changed on evidence.
- **[Bag-replay scoring](simulation/bag-replay.md)** — replays a *real* robot's
  mapping bag through SLAM under N parameter sets and scores it truth-free. The
  reality check that complements the benchmark; it needs no sim at all.

`pixi run segment-eval` scores [room segmentation](robot/map-cleanup.md)
against `worlds/<world>.rooms.yaml`, the walkable rectangle of every enclosed
room.

## Isolation

Every entry point that starts a gz server — the benchmark, the smoke test,
`map_world.sh` — claims a free `ROS_DOMAIN_ID` and `GZ_PARTITION` through one
picker (`mote_simulation/tools/sim_domain.py`), so two runs on one machine stay
separate. The sim environment also pins
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, so a run neither advertises itself
to nor discovers a robot on the LAN.

That makes the sim invisible to a default-range tool, which is why there are
two RViz tasks: `pixi run rviz` for the robot, and `pixi run rviz-sim` joined to
the sim's host-local graph.

Teardown is scoped the same way. Each launch is `setsid`-ed, so its session id
is the exact reaping scope: it can clean up stragglers under any node name and
reach nothing else. Bare name matches (`pkill -f mote_world`) are what that
replaces — see [clearing stray ROS
processes](robot/bringup.md#clearing-stray-ros-processes--sweep_orphanspy).
