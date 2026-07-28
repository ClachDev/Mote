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
pixi run publish-map    # Offer that map to the fleet registry as a candidate (M4)
pixi run save-zone <n>  # Teach a zone: capture current robot pose (+ optional --radius) into the site
pixi run segment-map    # Propose a zone per room of a saved map (--write merges into zones.yaml)
pixi run site           # Site CLI: create / add-floor / use / use-map / list / info
pixi run teleop         # Keyboard teleoperation
pixi run tasks          # Task layer: behaviour-tree task_server (see mote_tasks)
pixi run arm            # SO-101 arm driver: joint states + safe jog control
pixi run arm-jog        # Interactive per-joint jog CLI (needs `pixi run arm`)
pixi run arm-check      # Standalone arm bus enumeration + health (read-only)
pixi run arm-calibrate  # Range calibration: centre the joints, sweep, emit limits
pixi run arm-pose       # Teach/replay named arm poses; narrow the envelope
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
pixi run enroll         # Ask the fleet server for this robot's id (writes ~/.mote)
pixi run agent          # Robot -> fleet bridge (mote-agent.service)
pixi run foxglove       # foxglove_bridge + teleop relay: the remote console
pixi run fleet-server   # Off-board: fleet API + operator dashboard
pixi run fleetctl       # Operator CLI: token / operator / robots / dispatch / audit / watch
pixi run fleet-broker   # Off-board: MQTT broker + WebSockets (a container)
pixi run fleet-deploy   # Fleet box: container stack up/update/rollback/backup
pixi run inference-deploy  # GPU box: blue/green deploy of the inference image
pixi run deploy-test    # Exercise the blue/green pipeline with stubs (no GPU)

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

# Fleet-server environment only (`fleet`: an MQTT broker + Python, no ROS — the
# deployable control-plane role. The dev env folds this feature in too, so a
# workstation can run broker + server + agent + a real behaviour tree at once.)
pixi run -e fleet fleet-server     # fleet API + dashboard on the fleet box
pixi run -e dev test-fleet      # mote_fleet tests incl. the real-broker e2e run

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

Milestone M0 of `docs/design/fleet.md`; the operator runbook is `docs/fleet/README.md` and the measurements behind it are `docs/fleet/m0-verification.md`. **`MOTE_HOME` (default `~/.mote`) is per-robot state; the package is shared config** — `mote_bringup/mote_home.py` is the one place that rule lives (`mote_dir()`, `path()`, and `override(name, packaged_default)` which prefers the per-robot file). `sites.py`, `mote_launch.py` (camera calibration), `perception_launch.py` (`perception.yaml`), `health_monitor.py` (`health.yaml`) and `self_check.py` (`self_check_status.yaml`) all resolve through it, so `MOTE_HOME` is honoured everywhere and an update can never clobber identity, site selection, calibration, maps or bags. **Identity** is `$MOTE_HOME/robot.yaml` (`mote_bringup/identity.py`, `pixi run identity show|id|set`): a `robot_id` constrained to a lowercase DNS label because it is simultaneously a MagicDNS hostname, an MQTT topic level and a directory name. It is deliberately not the hostname, and operator-set until M1's enrollment endpoint allocates it. Do not confuse `$MOTE_HOME/robot.yaml` (this robot's identity) with `mote_description/config/robot.yaml` (shared hardware description). **The overlay** is Tailscale (`pixi run tailnet`, `mote_bringup/tailscale/install.sh`), joining robots/servers as *tagged* devices and the workstation as a user device; a robot's tailnet hostname *is* its `robot_id`. **A clean Pi** is provisioned by one rendered cloud-init file (`pixi run provision`, `mote_bringup/provision.py` + `provisioning/user-data.template`): identity → tailnet (single-use tagged auth key baked into the image, shredded after use) → pixi/build → `pixi run setup`. **DDS**: the end state is `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` on the robot, which retires the `ROS_DOMAIN_ID` isolation question entirely — **landed with M2** (below), which supplied the `foxglove_bridge` off-box path that had to exist first; the systemd units carry the pin, an interactive `pixi run` keeps stock discovery, and `config/cyclonedds.xml` still narrows the interface and multicast (see DDS scoping under Environment). What M0 adds is the measurement: rmw_cyclonedds caps localhost discovery at `MaxAutoParticipantIndex=32`, i.e. 33 participants (≈ processes) per host, and `pixi run dds-check` reports the headroom from `/proc/net/udp` (measured 17/33 for the sim nav mission under both localhost and stock discovery; ~25 with perception + the M1 agent + M2's bridge and teleop relay). Re-check it whenever a milestone adds processes; raise it in the robot's `cyclonedds.xml` if it runs out.

## Fleet: the control plane (M1)

Milestone M1 of `docs/design/fleet.md`, built in the **`mote_fleet`** package (below); the wire it speaks is specified once, as a versioned contract, in **`docs/fleet/control-plane.md`** — read that before touching a payload, and `docs/fleet/README.md` §6–8 for the operator flow. **Identity is now server-allocated**: `pixi run enroll` presents an enrollment token plus a hardware fingerprint to `POST /v1/enroll`, and the server (`mote_fleet/server/fleet_server.py` + a SQLite `registry.py`) allocates `mote-NN`, records the row, and answers with the broker address; the robot writes `$MOTE_HOME/robot.yaml` (identity, as M0) and `$MOTE_HOME/fleet.yaml` (server + broker). Enrollment is **idempotent on the fingerprint** (SoC serial → machine-id → MAC), so a re-enrolled robot is the same robot, and it **adopts** an M0 operator-set id rather than renumbering it. Allocation runs in a `BEGIN IMMEDIATE` transaction because ids are derived from the rows already present. **The topic tree is `mote/v1/<robot_id>/{presence,health,pose,task/command,task/status}`** — the major version is in the topic root so a v2 can coexist during migration, and every payload additionally carries `schema: 1`. Everything is retained *except* `task/command` (a retained command would re-fire on every reconnect); `presence` doubles as the MQTT Last Will, which is what makes "robot dropped off" instant. **The single-in-flight rule lives in the agent, not the task layer** (`mote_fleet/dispatch.py`): `task/command` is a bare `std_msgs/String` with no request id, so the agent holds one command, rejects a second itself, treats a redelivery of the same id as a replay, and attributes ROS statuses using *both* that rule and the command text every status line echoes — anything unmatched is reported as `source: local`. **The agent is a bridge and a reporter, never in the control loop**, and is deliberately not part of `pixi run robot`/`mapping`: it runs as its own `mote-agent.service` (installed, not enabled) so a robot that cannot reach the fleet server still boots and navigates. Health is the health monitor's `/diagnostics_agg` roll-up *forwarded*, not recomputed, with `state: unknown` when no monitor is reporting; `battery` is in the schema and always null (nothing measures it). Two packaging gotchas, both worked around and recorded in `docs/fleet/m1-verification.md`: conda-forge's mosquitto puts the **broker** in `$PREFIX/sbin` (pixi only puts `bin` on PATH) and is built **without websockets**, which M3's dashboard needs — M3 answers that with a container broker (below).


## Fleet: the remote console (M2)

Milestone M2 of `docs/design/fleet.md`; the operator flow is `docs/fleet/README.md` §10 and the measurements are `docs/fleet/m2-verification.md`. **Foxglove is adopted, not built**: `ros-jazzy-foxglove-bridge` runs on the robot (`foxglove_launch.py`, `pixi run foxglove`, port 8765) and an operator connects to `ws://<robot-id>:8765` over the tailnet — this is what finally replaces joining the robot's DDS graph from a workstation, and it is where M3's per-robot deep-link (`FOXGLOVE_URL` in `fleet_server.py`) lands. It is **included in the base bringup by default** (`mote_launch.py`'s `foxglove:=true`, exactly like `health`), so every way of starting the robot gives something to connect to, while `mote-bringup.service` passes `foxglove:=false` because `mote-foxglove.service` runs it independently — the view must outlive a bringup restart, since a crash-looping mission is when it is most needed. **Teleop needed a node, not just a layout**: Foxglove's Teleop panel publishes only unstamped `geometry_msgs/Twist` while `DiffDriveController` consumes `TwistStamped`, so `twist_relay.py` adds the header — deliberately event-driven, with no timer and no memory of the last command, so that when commands stop the robot stops. The deadman is `cmd_vel_timeout: 0.5`, now pinned explicitly in `controllers.yaml` rather than left to a default, and the stamp is taken *on the robot*, so the operator's clock never enters the safety path. The shipped layout is `mote_bringup/foxglove/mote.json` (+ its README); `test_foxglove_layout.py` ties it back to `controllers.yaml` so changing a velocity limit or the timeout fails a test rather than silently invalidating what an operator is handed, and `test_foxglove_teleop.py` (dev env, `pixi run -e dev test-foxglove`) drives the real bridge the way the panel does. **M2 also spends the DDS pin M0 deferred**: every `mote-*.service` now carries `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` — all of them, because a localhost-range participant discovers a same-host default-range one but not the reverse — so a systemd-run robot is invisible to the LAN, while an interactive `pixi run` keeps stock discovery and bench work with RViz is unaffected. Two gotchas: bridge **3.3.0 speaks the `foxglove.sdk.v1` subprotocol** and refuses a client offering only the older `foxglove.websocket.v1` with a bare HTTP 400; and **teleop does not pre-empt Nav2** (both write `/diff_drive_controller/cmd_vel` — cancel the task first; a `twist_mux` is the tracked fix). Cost is 2 DDS participant slots, putting the full robot stack at ~25 of 33.

## Fleet: the operator view + dispatch API (M3)

Milestone M3 of `docs/design/fleet.md`, and the end of v0. The **HTTP** wire is specified as its own versioned contract in **`docs/fleet/fleet-api.md`** (M1's MQTT one is `control-plane.md`); the operator flow is `docs/fleet/README.md` §6–9 and the measurements are `m3-verification.md`. **The two directions of the loop take different paths on purpose.** *Reads* ride MQTT: the browser subscribes to `mote/v1/+/{presence,health,pose,task/status}` over WebSockets, and because all of those are retained it has the whole fleet's state within a second of loading — no polling, no service in the middle. *Writes* ride HTTP: `POST /v1/robots/<id>/dispatch` authorizes an operator token (`fleetctl operator new --name <you>`; the name is what the audit row records), writes the audit row, then publishes to the same `task/command` topic. **The topic tree did not change — only who publishes to it**, and `fleetctl dispatch` moved to the API too, so there is one write path rather than one per client. The command grammar is still parsed only by the robot's task layer: a parser in the server would be a second grammar to keep in step. **The browser cannot publish**: `server/ui/mqtt.mjs` is a hand-rolled subscribe-only MQTT 3.1.1 client that implements no PUBLISH packet, so the split is enforced by omission (M7 makes it structural with a subscribe-only broker credential). The UI is static ES modules — no bundler, no npm, no vendored library — served by the same stdlib `http.server`; `map.mjs` holds the Q5 world→pixel transform (`px = (wx-origin_x)/res`, `py = height - (wy-origin_y)/res`) and a pan/zoom/follow canvas, and only draws robots on the *same* site+floor as the selected one because a pose from another floor is a different map frame. **Basemaps come from site bundles on the fleet box** (`--maps-dir`, default `$MOTE_FLEET_HOME/sites`, the layout `sites.py` writes, seeded by rsync until **M4** makes the registry canonical behind the same two routes). **M1's websockets blocker is settled**: `pixi run fleet-broker` runs `eclipse-mosquitto` under docker with the repo's own `mosquitto.conf`, because conda-forge's build has none; `pixi run -e fleet fleet-broker-local` is the conda binary for a box without docker, and it strips the WS stanza and says so. Two things that run in the same file (`test_ui.py` → `ui_test.mjs`) are the MQTT codec and the transform, tested under node against the very files the browser loads; `browser_check.mjs` drives a real headless Chrome over CDP against a running stack and is an operator's tool, not a CI test.

## Fleet: the map registry (M4)

Milestone M4 of `docs/design/fleet.md`: the fleet server becomes the **canonical
registry of sites, floors and map revisions**. The HTTP routes are in
`docs/fleet/fleet-api.md`, the retained topic in `control-plane.md`, the operator
flow in `README.md` §11 and the measurements in `m4-verification.md`. **The whole
design is one rule: uploading is not publishing.** `pixi run publish-map` (after
`save-map`, deliberately separate because saving must work on a robot that has
never seen a fleet server) packs the floor's current revision and POSTs it; the
server validates it and keeps it as an inert **candidate** that changes nothing.
An operator's `fleetctl promote <site> <floor> <rev>` — or the picker beside the
dashboard's map — flips the floor's `map` symlink and publishes
`mote/v1/registry/site/<site>/floor/<floor>/current`, **retained**, which is the
entire distribution mechanism: an agent that was switched off through a whole
mapping session is handed the canonical revision the moment it reconnects, with
no polling and no missed-update case. That inertness is also the conflict answer
— two robots that map one floor leave two candidates, never a merge, because a
map frame's origin is an accident of where SLAM started (an id collision, ids
being per-second timestamps, stores the second as `<rev>-2`). **The shared,
ROS-free validator the design asked for is `mote_bringup/bundle.py`** — the
bundle's *content* (what a revision must hold, whether the map inside is usable,
how it packs for a wire) as against `sites.py`'s *layout* — and a real
occupancy check ("the map is not degenerate") over the map's own pixels. It
lives in `mote_bringup`
because the layout is `sites.py`'s and `mote_fleet` already depends on it — the
reverse would be a package cycle — and the deploy image copies just those two
ROS-free files, so the fleet box still installs no ROS. It reads YAML with **PyYAML** and
maps with **Pillow**, both installed in the fleet image: it first shipped with
hand-rolled readers for both, to keep the server's dependency list at exactly
"python", and **that rule was never in the design** — `fleet.md` asks only for
ROS-free and torch-free. Both hand-rolled readers were wrong in ways the
libraries are not (a zone named `Café` came back as `Caf\xE9` **silently**, the
polygon shape `segment-map` and `save-zone` emit did not parse at all so #69's
output was a bundle M4 refused, and the PNG decoder raised through a "never
raises" contract and left an upload with no HTTP response). Stdlib-only still
holds where the design does put it: `protocol.py`.
`save-map` now runs the same check locally, so a map the server would refuse is
refused while the mapping session is still up. On the robot, `mote_fleet/
mapsync.py` + a worker thread in the agent stage a pulled revision in a temp
directory, verify the announced sha256, rename it into `maps/<rev>/` and flip the
local symlink; **zones travel inside the revision** and replace the floor's
`zones.yaml` (the old one is kept as `zones.<old-rev>.yaml`), because a different
session's map makes previously taught zones wrong. Three deliberate consequences:
the flip and the announcement are reported separately (a broker that is down must
not half-promote a floor; the server re-announces every floor at startup, which
repairs it), an **upload carries no operator credential** — it names an enrolled
robot, is bounded and audited, and is inert until M7 gives robots a credential —
and a pulled map takes effect on the **next bringup**, since `map_server` reads
its map at startup, so health now carries the revision each robot is actually
running. M3's `/v1/maps` routes kept their shape and changed source; the
dashboard additionally draws the floor's taught zones (circle, polygon or
waypoint cross) from `/v1/maps/<site>/<floor>/zones.json`.

## Fleet: the server pipelines (Ms)

Milestone Ms of `docs/design/fleet.md`: how the two **non-robot** machines are built and updated, runbook in `docs/fleet/server-pipelines.md`, measurements in `docs/fleet/ms-verification.md`. Both are container deploys **driven by their operator, not by the fleet server** — a robot is fleet-managed, a server is infrastructure — and the pipelines differ in exactly one thing, state. **The fleet server** (`mote_fleet/deploy/`: `Dockerfile` + `docker-compose.yml` + `fleet-deploy.sh`, image `ghcr.io/clachdev/mote-fleet` built by `.github/workflows/fleet-image.yml`) is two containers — `eclipse-mosquitto:2` mounting **`server/mosquitto.conf`, the same file `fleet-broker` uses**, so deployed and workstation brokers cannot drift, and a python image carrying the API, the registry and the M3 dashboard — plus two named volumes holding the only state that matters: `/var/lib/mote-fleet` (registry.db + the `sites/` bundles the dashboard's basemaps come from) and the broker's retained messages. `.env` is the declared state; `BROKER_HOST` is the one value that must be right (handed to robots verbatim at enrollment, so the compose file refuses to start without it) and `BROKER_WS_PORT` is published *and* passed as `--broker-ws-port`, because it is the port the browser is told to reach the broker on. It holds state, so its update is a **gated recreate**, not blue/green: `fleet-deploy.sh update` tags the running image `:previous` before pulling, recreates, health-gates on `/healthz` over the *published* port, and puts the old image back automatically if that fails; `backup`/`restore` snapshot both volumes (registry via sqlite3's online backup API, not `cp`). **The inference server** (`mote_perception/deploy/inference-deploy.sh`, one file curled onto the host) is stateless, so it *is* blue/green: the candidate runs on shadow ports 5611/5612 while the current one keeps serving, and must pass `mote_perception/tools/probe.py` — health **and a real synthetic frame**, because a health sentinel is answered before the model has ever loaded and cannot see a broken weight download or a CUDA mismatch. The **flip is a stop-then-start**, deliberately: pushing a new port out to robots (as the design sketch had it) means editing `perception.yaml` on every robot, a worse outage than the seconds this costs, which the robot's warn-and-skip fallback makes a non-event. Every check runs *inside the image being deployed*, so the GPU box still installs nothing. `mote_perception/deploy/test/drill.sh` (`pixi run deploy-test`) exercises that whole pipeline with stub images on any machine with docker — no GPU — and `mote_fleet/test/test_fleet_outage.py` is the other half of the milestone's acceptance: kill the broker under a live agent and the robot still finishes its task, then the agent reconnects by itself.

## Sites (maps & zones)

Everything that is only meaningful relative to one mapped place — the Nav2 map pair, the slam_toolbox posegraph, and named zones — lives together as a **site bundle** under `~/.mote/sites/<site>/floors/<floor>/`, managed by `mote_bringup/sites.py` (CLI: `pixi run site`, docs in the module docstring). A floor is one SLAM session (one map frame); a site groups floors sharing a location. `~/.mote/active.yaml` selects the active site/floor per robot; launch files resolve the map (`nav2_launch.py`, `robot_launch.py`) and zones (`tasks_launch.py`) from it at launch time (zones fall back to the committed default). `MOTE_HOME` overrides `~/.mote` for tests/experiments. What a revision must *contain* — and how it validates, packs and travels — is `mote_bringup/bundle.py` (ROS-free, shared with the fleet server; see the map registry section above). Map artifacts are immutable **revisions** under `floors/<floor>/maps/<rev>/`, published by atomically flipping the `floors/<floor>/map` symlink once the revision is complete — a half-written save or interrupted transfer is never visible, and `site use-map <rev>` rolls back. `save-map` stores the posegraph alongside the map so mapping can be *continued* in the same frame later (extend, don't remap — remapping breaks zone coordinates). Mapping runs also record the `mapping` rosbag stream by default (`mapping_launch.py record:=true`; the sim passes false), and `save-map` stamps the session's bag into the revision's `meta.yaml` for provenance (`site info` shows it). Zones are taught by driving there and running `pixi run save-zone <name>`, not by editing YAML; a zone is a named pose (a fetch waypoint or a `goto <zone>` target) that may optionally carry an area **footprint** — a taught `--radius` circle, or a `polygon` outline that follows the actual room walls — so it reads as a room and answers "am I in it"; one concept, one `zones.yaml`, `site info` shows the zone/footprint counts. Maps are saved as PNG (map_server reads it natively; browsers can render it directly). `save-map` automatically runs an FFT structure-extraction **cleaning pass** (`mote_bringup/map_cleanup`, `sites._promote_cleaned`): it keeps the untouched map_saver output as `map_raw.png` and promotes the decluttered image to the served `map.png` (plus a `diagnostics.png`), so navigation always consumes the cleaned map while the raw is retained for provenance/audit. The `map.yaml` frame is identical for both, so zones/localization are unaffected; a cleaning failure falls back to serving the raw. The posegraph belongs to the raw map — mapping continuation extends from raw, never the cleaned image. **Zones no longer have to be taught one at a time**: `pixi run segment-map` (`map_cleanup/room_segmentation.py`, the ROSE² second stage the declutter pass left open) carves a saved map's free space into rooms and proposes one polygon zone per room, `--write` merging them into the floor's `zones.yaml` for the operator to rename — additive over hand-taught zones (a candidate covering an already-footprinted zone is dropped as named, so re-running is a no-op) and written beside `zones.yaml`, never into the immutable map revision. The method is one physical assumption — a doorway is narrow — applied to a grid the wall lines cut into faces: faces merge wherever their shared boundary has a clear span wider than a door, so it is indifferent to room size where a distance-transform threshold is not. Two consequences: a **corridor network is not proposed at all** (a footprint is a single outline, so a region encircling a block of rooms would claim them; those are dropped, taking with them any room wrongly absorbed into the corridor), and the geometry is **Manhattan after rotation** — an arbitrarily rotated map frame is fine, a building with wings at 30° to each other is not. Scored against ground-truth room rectangles on the sim ladder by `pixi run segment-eval` (30/33 mapped hospital rooms, 10/10 office, 1/1 mote, **zero merges**, unchanged with the map turned 17° or -31°); results in `docs/tuning/2026-07-27-room-segmentation.md`.

## Architecture

Mote is a differential-drive robot built on **ROS 2 Jazzy**, managed entirely through pixi (no system ROS install required). First-party packages:

### `mote_hardware` (C++)
A `ros2_control` `SystemInterface` plugin (`MoteHardware`) that drives two Feetech STS3215 servos via the SCServo SDK over a serial bus. Key implementation details:
- Servo IDs and all hardware params come from `robot.yaml` via the URDF's `<ros2_control>` tag, read by `MoteHardware::on_init` from `info_.hardware_parameters`
- Position is tracked cumulatively across the 12-bit encoder rollover using a half-range threshold
- The left wheel is mounted inverted, so its sign is negated in both `read()` and `write()`
- The serial port is opened in `on_activate` (not `on_init`), which also puts servos into wheel (continuous rotation) mode — an EEPROM write, skipped if already set
- Tools built from `mote_hardware/tools/` (`servo_debug`, `velocity_cal`, `swap_ids`, `setup_ids`) run as `pixi run -- ros2 run mote_hardware <tool>`; see `mote_hardware/tools/README.md`

### `mote_description` (CMake)
Contains `urdf/mote.urdf.xacro` and `config/robot.yaml`. The xacro loads robot.yaml at processing time and uses those values directly — no xacro args are needed or accepted. The `<ros2_control>` tag embeds the servo params so they reach `MoteHardware::on_init`.

### `mote_nav` (C++)
The C++ that runs *inside* other people's processes: a Nav2 plugin and two composable nodes, all too small to deserve a process each. The two hardware numbers they share — `max_wheel_speed` and `wheel_separation` — reach both through `include/mote_nav/wheel_speed.hpp` (`maxWheelSpeed`, `maxYawRate`), which is dependency-free so the odometry gate does not drag `dwb_core` in nor the critic `tf2_ros`.
- `WheelSpeedLimitCritic` (`src/wheel_speed_limit_critic.cpp`, exported to `dwb_core`) — DWB samples a v x w rectangle with no notion of the differential-drive coupling between the two, so it marks any sample needing a wheel faster than `max_wheel_speed` illegal. `wheel_separation`/`max_wheel_speed` are overlaid onto `controller_server` from `robot.yaml` by `nav2_launch.py`, so the hardware envelope has one source of truth.
- `mote_nav::OdomTfRelay` (`src/odom_tf_relay.cpp`, an `rclcpp_components` component; also installed as a standalone `odom_tf_relay` executable) — republishes the wheel pose as the inverted TF leaf kinematic_icp reads as its motion prior. Loaded into `localization_launch.py`'s container alongside its one consumer. It replaced a Python node of the same name: at the controller's 50 Hz update rate the interpreter wake-up, the GIL and the process hop each cost more than the twenty floating-point operations they existed to perform. The arithmetic is a deliberate transcription of the Python it replaces and the target is built `-ffp-contract=off`, so the output is bit-identical rather than merely close — which is how the two were compared.
- `mote_nav::IcpOdomGate` (`src/icp_odom_gate.cpp`, a component; also a standalone `icp_odom_gate` executable) — **owns `odom`→`base`**, accumulating kinematic_icp's increments and substituting the wheel increment for any the drive could not have produced. Real mapping bags catch the scan match emitting, in one scan, up to 1.2 m/s against a measured 0.218 m/s limit — once 0.12 m while the wheels reported the robot **stationary**. Slip cannot cause it in that direction (slip makes the *wheels* over-read), and the frames are **steps, not spikes**: the ICP-vs-wheel gap rate is identical either side, so the displacement is never given back and each one is permanent error in the map frame and in every zone taught in it. Since a TF broadcast cannot be retracted, kinematic_icp is left publishing only its odometry topic in a frame of its own (`odom_icp`) and the gate broadcasts the edge instead; kinematic_icp is unaffected, because its prior comes from the `odom_wheel` TF leaf and never from its own output. The bound is `robot.yaml`'s two numbers × 1.15, through the shared `wheel_speed.hpp`. Two things the data decided rather than taste: the **joint `|v| + S/2·|w|` bound the critic uses is unusable here** — the wheel odometry itself exceeds it in 19% of intervals and the legitimate and excursion populations overlap completely, the yaw term being inflated by 10 Hz resampling — so translation and yaw are bounded separately; and the substitute is the **wheel increment rather than a clamp**, because a stationary robot's wheels say 0 where a clamp still admits 0.025 m. Evidence, thresholds and before/after are `docs/tuning/2026-07-28-icp-velocity-gate.md`; `mote_bringup/tools/icp_excursions.py` characterises a bag's excursions (spike vs step) and `icp_gate_replay.py` replays a bag through the *compiled* gate so `odom_health.py` scores real output rather than an offline model of it.

### `mote_bringup` (Python/ament)
Launch files, config, udev rules, NetworkManager drop-ins, systemd services, and the fleet foundation: `mote_home.py` (per-robot state root), `identity.py` (`identity` console script), `provision.py` + `provisioning/user-data.template` (`provision`), `dds_participants.py` (`dds_participants`), `twist_relay.py` (`twist_relay`, the Foxglove teleop seam), the `foxglove/` layout, and `tailscale/install.sh`, plus `bundle.py` — the site bundle's *content*: a ROS-free reader/validator/packer for a map revision, shared with the fleet server's registry (M4) so both ends check a revision with the same code. See Fleet above.

**On-robot reliability** (see `mote_bringup/README.md`): `pixi run robot`/`mapping` include the health monitor, so a manual run publishes `/health` too; the systemd units are installed by `pixi run setup` but **not enabled** (autostart would drain the battery on a desk — opt in with `systemctl enable --now mote-bringup mote-health`). the systemd services restart with backoff and never permanently give up (`Restart=always`, `RestartSec`/`RestartSteps`/`RestartMaxDelaySec`, `StartLimitIntervalSec=0`), order after the udev-tagged `dev-mote_*.device` units, and bound the journal. A pre-flight self-check (`self_check.py`, run as `mote-bringup`'s `ExecStartPre`; `pixi run self-check`) gates bringup on servo ping + lidar/camera/disk/clock/config and keeps the robot idle with a clear reason on failure. A health monitor (`health_monitor.py`, `mote-health.service` with a `Type=notify` watchdog; `pixi run health`) publishes per-subsystem `diagnostic_msgs/DiagnosticArray` on `/diagnostics_agg` and a single OK/DEGRADED/FAULT summary on `/health`. Driver and nav2 nodes are `respawn=True` for per-node recovery under the whole-service systemd restart. Battery voltage is **not** software-measurable (the power bank exposes no telemetry); `system_monitor` reports the Pi's `get_throttled` flags as the only power signal — read via **`vcgencmd`**, since the Pi 4 sysfs node does not exist on a Pi 5, alongside the Active Cooler's `fan_rpm` from the `pwmfan` hwmon.

**Launch hierarchy:** the two mission launches (`mapping_launch.py`, `robot_launch.py`) each take a `base` arg (default true) that includes the hardware base, and a `use_sim_time` arg they forward to everything they include. The sim runs these *same* files with `base:=false`, supplying a Gazebo base in place of the drivers — so the missions are defined once and the sim exercises the real launch files.
- `robot_launch.py` — nav mission: `mote_launch.py` (if `base`) + `nav2_launch.py` (drive a saved map). Forwards a `map` arg, defaulting to the active site's map (see Sites).
- `mapping_launch.py` — mapping mission: `mote_launch.py` (if `base`) + `slam_launch.py` + `nav2_launch.py` (`localisation:=false`) + `record_launch.py` (`streams:=mapping`, unless `record:=false`): build/extend a map with SLAM *and* drive to goals autonomously while doing so, recording the session for map provenance.
- `mote_launch.py` — the hardware base: robot_state_publisher, ros2_control_node, controller spawners, sllidar, laser_filter, v4l2_camera, `localization_launch.py`, and `foxglove_launch.py` (`foxglove:=true`). Reads `robot.yaml` for wheel geometry (injected into DiffDriveController params) and sensor config. Asserts `use_sim_time` (default false) for the whole tree via `SetParameter`.
- `localization_launch.py` — kinematic_icp LIDAR odometry (the `odom`→`base` edge; the map→odom corrector is slam_toolbox when mapping or AMCL when navigating). Despite the name, it does *not* run AMCL — AMCL lives in `nav2_launch.py`. All three parts are components in one `localization_container` (`component_container_isolated`, as in `nav2_launch.py`): kinematic_icp; `mote_nav::OdomTfRelay` writing the `odom_wheel` leaf it reads as its motion prior; and `mote_nav::IcpOdomGate`, which is what actually **broadcasts `odom`→`base`** — three processes and three DDS participants become one, and the relay stops being a Python interpreter woken 50 times a second. kinematic_icp therefore runs with `publish_odom_tf:=false` and `lidar_odom_frame:=odom_icp` (`ICP_ODOM_FRAME`), publishing only its odometry topic, which the gate consumes: a TF broadcast cannot be retracted, so a gate downstream of ICP's own broadcast would be no gate at all. The leaf name is one constant (`WHEEL_ODOM_FRAME`) because a disagreement between writer and reader costs kinematic_icp its prior without failing anything, and the same is true of every new seam here (the gate's remap must match ICP's *namespaced* topic; ICP must not re-claim the edge) — `test_localization_composition.py` holds all of it, since composition's failure modes (an unnamed node taking defaults, a plugin string matching no registered component, two publishers on one TF edge) are all silent.
- `slam_launch.py` — slam_toolbox (accepts `use_sim_time:=true` for the sim)
- `nav2_launch.py` — Nav2 stack, **composed**: all nine servers plus both lifecycle managers are `ComposableNode`s loaded into one `nav2_container` (`component_container_isolated`, which gives each component its own executor thread — the shared-executor containers would serialise servers that block inside callbacks). Ten processes become one, which is also ten DDS participants become one. The drivers in `mote_launch.py` stay separate processes: composition trades crash isolation for efficiency, and the drivers are the crash-prone half. Two things are load-bearing and non-obvious. **The params file goes on the container as well as on each component** — Nav2's servers create further nodes of their own (`/local_costmap/local_costmap`, `/global_costmap/global_costmap`, the bt_navigator client nodes) which are not components and so are never named in a load request; they inherit the *process* command line, which inside a container is the container's, so without it the costmaps come up on library defaults and nav quietly degrades rather than failing. **Each `ComposableNode` must carry `name=`** matching its `nav2_params.yaml` key, because a composable node loaded without a name is matched against no section of the file and receives no file parameters at all. A `localisation` arg (default true) toggles the `map_server` + `amcl` half: true localises against a saved map; false drops them so the navigation servers run against a live slam_toolbox map and `map→odom` instead (used by `mapping_launch.py`). Recovery is now whole-stack: the container is `respawn=True`, and the component loads are re-issued on *every* `OnProcessStart` (via an `OpaqueFunction` returning fresh actions, since an executed action cannot run twice) so a respawned container is refilled rather than coming back empty. `slam_toolbox` is deliberately left as its own process, as upstream `nav2_bringup` leaves it
- `foxglove_launch.py` — the remote console: `foxglove_bridge` (WebSocket on 8765) plus the `twist_relay` teleop seam (`teleop:=true`). Included by `mote_launch.py`; run alone as `pixi run foxglove`, or as `mote-foxglove.service`
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
- `test/room_segmentation_eval.py` (`pixi run segment-eval`) — scores map room segmentation (see Sites) against `worlds/<world>.rooms.yaml`, the walkable rectangle of every enclosed room: emitted by `gen_hospital.py` for the generated world, read off the SDF for the two hand-written ones. Only rooms the exploration run actually mapped are scored (20 of the hospital's 53 were never entered), and `--rotate` turns map *and* truth together to exercise the wall-alignment step against real SLAM data, since every world on the ladder happens to be axis-aligned and a real map frame is not.
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
- `zones.py` — the one named-place concept: `load_zones` returns `{name: Zone(name, pose, footprint)}` from the `zones:` section. A **zone** is a pose the robot navigates to (both fetch waypoints *and* goto targets resolve against this single table); it *optionally* carries an area **footprint** — a `radius` circle or a `polygon` of explicit vertices — that's just optional metadata, not a second type. `zones.containing(zones, x, y)` is the "which zone am I in" membership query (nearest-pose first) over zones that have a footprint; `goto` itself only needs the pose. A polygon may be concave (ray-cast membership, so an L-shaped ward or a corridor stretch works where a circle can only under- or over-cover: the hospital wards are 4.7x5.6 m, of which a `radius: 1.5` circle claimed 7.1 m² of 26.5 m²), wins over a `radius` if a zone carries both, and — since polygons come from post-processing a map rather than from driving — may omit `x`/`y`, in which case the loader derives a pose guaranteed to lie inside the outline. `save-zone` therefore preserves an existing footprint when it re-teaches a pose; `--radius` is the explicit way to replace one. Polygon zones no longer have to be hand-written either: `pixi run segment-map` proposes one per room of a saved map (see Sites).
- `behaviours/` — `DriveTo` (Nav2 NavigateToPose action client as a behaviour; cancels in-flight goals on preemption), `AcquireObject` (label missions: publishes the label to `detect/labels`, waits for a matching `detected_objects` detection, writes a standoff goal — 0.4 m short of the object, facing it — to `object_pose`; zone missions pass through), and `TimedStub` (placeholder pick/place until the SO-101 arm is actuated).
- `trees/` — `common.py` (shared `WaitForTask` + the `task` blackboard key), `fetch.py` (wait → acquire object → drive to object → pick stub → drive to drop → place stub; blackboard keys `task`/`object_pose`/`object_label`/`drop_pose` are the seam between the command grammar and perception), and `goto.py` (wait → drive to the zone's pose; success == Nav2 success).
- `test/` — mock-`navigate_to_pose` tree ticks (`test_fetch_tree.py`, `test_fetch_object.py` against a mock detector, `test_goto_tree.py`) plus pure parser/loader tests (`test_parse_command.py`, `test_goto_command.py`, `test_zones.py` — which covers zone footprints and `containing`), no Gazebo/Nav2 needed, run by `pixi run test`.

### `mote_fleet` (Python/ament)
The fleet control plane — one package for both ends of one wire, the same split `mote_perception` makes for inference: a node on the robot, a server off-board, and a single shared wire module so they cannot drift. Details and rationale in the Fleet section above and `mote_fleet/README.md`. Contains:
- `protocol.py` — the versioned contract (topic tree, payload builders, task states). **Stdlib-only and ROS-free**, so the off-board server imports it from the source tree by path rather than vendoring a copy. `schema/*.schema.json` is its machine-readable mirror and `test_protocol.py` fails if code, schema and `docs/fleet/control-plane.md` disagree.
- `dispatch.py` — the single-in-flight tracker and the parser for `task_server`'s status strings. No ROS, no MQTT, so every ambiguous attribution case is a plain function call in `test_dispatch.py`.
- `agent.py` (`pixi run agent`, `mote-agent.service`) — the node. Presence/health/pose up (retained, LWT), one command at a time down. The MQTT client is injectable, which is how `test_agent.py` covers the whole bridge in CI without a broker.
- `enroll.py` (`pixi run enroll`) + `facts.py` — the robot side of enrollment and the hardware fingerprint it is idempotent on. `fleet_config.py` owns `$MOTE_HOME/fleet.yaml`.
- `server/` — ROS-free scripts for the fleet box: `fleet_server.py` (stdlib `http.server`: enrollment, roster, dispatch, audit, basemaps, and the UI), `registry.py` (SQLite rows — robots, enrollment tokens, operators, the audit log; state under `$MOTE_FLEET_HOME`, default `~/.mote-fleet`), `fleetctl.py` (`pixi run fleetctl`: token/operator/robots/dispatch/audit/watch), `ui/` (the dashboard: static ES modules, a subscribe-only MQTT client, the Q5 map transform), `mosquitto.conf` + `broker.sh` (conda or container, the latter for WebSockets). Every write to `task/command` — CLI or browser — goes through the API, so dispatch is authorized and audited in one place.
- `mapsync.py` + `publish.py` (`pixi run publish-map`) — the map registry's robot side (M4): pull the canonical revision announced on the retained topic, or offer a saved one as a candidate. ROS-free, so the whole distribution flow is testable as function calls.
- `server/bundle_store.py` — the registry's byte store: candidate revisions, validation on the way in (via `mote_bringup.bundle`), and the atomic symlink flip that publishes one. The filesystem is the truth about what is canonical; the database records who promoted it.
- `test/` — four tiers: contract (payloads, schema files, and every HTTP route over a real socket, including the registry's — `api_harness.py` is the live server they share), bridge (fake MQTT client; plus `test_mapsync.py`, the robot's map staging against a real server with no ROS), browser (`ui_test.mjs` under node — the MQTT codec, the map transform and zone placement, skipped without node), and the end-to-end pair `test_e2e_fleet.py` / `test_e2e_map_registry.py`, which run a real mosquitto and the real fleet server — the first with the actual `mote_tasks` tree against a mock Nav2 including a dispatch through the API, the second publishing and promoting a map and starting a *second* robot's agent afterwards, so only a retained message can have told it. Those skip without a broker, so `pixi run test` covers the rest and `pixi run -e dev test-fleet` covers all four.

### `mote_arm` (Python/ament)
SO-101 **follower** arm bring-up (synced to the Pi). There is no leader arm.
Uses **direct Feetech control** (not LeRobot): the arm servos are the same
STS-class Feetech bus as the drive wheels, so it reuses the servo stack rather
than pulling `torch` onto the lean Pi env — the sole new dep is the pure-Python
`feetech-servo-sdk` (`scservo_sdk`). All arm config (port, baud, servo IDs,
per-joint soft limits, home offsets, direction) lives in `robot.yaml`'s `arm:`
section. Contains:
- `config.py` — parses `arm:`; encoder<->radian conversion about `zero` + soft-limit clamping
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
  + `--save-zero` calibration snapshot. Run with the driver stopped (same port).
- **`zero` is not `home`.** `robot.yaml`'s `arm.joints[].zero` is the encoder
  count reading 0 rad — after calibration, the *middle* of the joint's travel.
  `home` is a taught *pose* in `~/.mote/arm_poses.yaml`, normally the arm's rest
  position. Both were spelled "home" until 2026-07-28 and it confused an
  operator at the bench, so the config key is `zero:` (`home:` still parses),
  `jog`'s command is `zero` (`home` aliases it with a note), and `arm-check` has
  `--save-zero`. Do not reintroduce the collision.
- `calibrate.py` + `arm_calibrate` (`pixi run arm-calibrate`) — **where the soft
  limits come from**, in LeRobot's two phases. A bus owner, not a driver client
  (the driver reports radians about the very zero under replacement, and the arm
  must stay limp). **Phase 1** records every joint at once in one live table (not one at a time).
  **Phase 2** moves each joint's zero to the *measured* middle of that sweep, by
  writing the servo's position-correction register (EEPROM, `SMS_STS_OFS_L/H` =
  address 31, sign-magnitude with bit 11 the sign): `present = actual - offset`,
  so this re-centres the travel in the 0-4095 frame. It reads the *existing*
  offset first — assuming zero would double-count on a re-run — and verifies
  afterwards that each reading moved by exactly the delta written, which is what
  would catch a wrong sign encoding. **Deliberately NOT LeRobot's order:** theirs
  asks the operator to hold every joint at mid-travel first (one `input()`, all
  six motors from that one pose) and takes the zero from it — awkward, and less
  accurate than the measurement the sweep takes anyway. Their order is
  load-bearing for *them*: `record_ranges_of_motion` is a plain min/max with no
  wraparound handling, so centring first is what keeps the sweep off the
  boundary. Sweeping first means it can wrap, so `SweepRecorder` unwraps and the
  centre comes from the unwrapped stream. LeRobot is Apache-2.0; no code copied,
  only the shape of the flow. **Offsets are modular** (`present =
  (actual - offset) mod 4096`), so a result outside the register's ±2047 is
  folded, never rejected — rejecting one aborted a real bench run. **Centring is
  not optional polish:** without it a joint whose travel straddles the encoder
  wrap cannot be described by any zero/limit pair, and on the real arm 2 of 6
  joints (`shoulder_pan`, `wrist_roll`) did exactly that; there is no software
  workaround, since the goal register is 0-4095 too. The emitted band is the
  swept range pulled *inward* by `--margin` (0.05 rad) — the opposite direction
  to `arm-pose limits` — and is symmetric about zero by construction, so it can
  never exclude its own zero (**the defect in the committed `shoulder_pan`
  limits: [0.010, 0.229] does not contain 0**). Refuses on travel exceeding one
  revolution (a continuous joint — no remedy, exclude it) and on a range too
  short for the margin; `--skip-homing` additionally hits the wrap and
  unreachable-zero cases, since it leaves the zero where it is.
  **It saves to `$MOTE_HOME/arm.yaml`, NOT the repo.** Zeros and limits are measurements of one physical arm, so they are
  per-robot state like `camera_calibration.yaml` and the site bundles — and
  `mote_description/config/robot.yaml` is shared by the fleet and read-only once
  installed from a channel. The package keeps the design (ids, names, direction,
  gains) plus defaults for an uncalibrated arm; `config.apply_calibration`
  overlays this robot's measured `zero`/`min`/`max` at load time, ignores a
  joint the package no longer has, and rejects inverted limits. Deleting the
  file reverts to the defaults. The file stores each measurement beside its
  value, including `homing_offset` — the only record of what went into servo
  EEPROM. **Taught poses are migrated automatically** (`poses.shift_poses`) — a pose is
  radians about the zero and the shift is exactly what calibration computed, so
  re-teaching by hand is pointless work; the old file is kept as `.bak` and any
  pose landing outside the new limits is reported, never clamped. **One
  confirmation only**, at the EEPROM write: torque release is done not asked
  (the arm is already limp unless a driver was SIGKILLed, detected via the
  torque register) and saving is the command's purpose. Three
  different files are called some form of robot config: `$MOTE_HOME/arm.yaml`
  (this arm's calibration), `$MOTE_HOME/robot.yaml` (fleet identity), and
  `mote_description/config/robot.yaml` (shared hardware description).
  A continuously-rotating joint is detectable **only** by being rotated past a
  whole turn (the refusal above); rotated less it is indistinguishable from a
  stopped joint, so do not add a threshold below one — it would miss most cases
  and fire on long-but-stopped joints. LeRobot instead hard-codes SO-101's
  `wrist_roll` as full-turn and skips its range; this arm's measures 5.88 rad
  (94%), so whether it truly has stops is unsettled. The live table shows
  `now`, both ends of travel, and the swept total, identically for every joint:
  the ends come off the *unwrapped* stream, so they are real positions even for a
  joint crossing 0/4095 (where the raw min/max read 17..4093 and describe the
  encoder, not the joint) and such a joint simply reads `low` above `high`.
  `--skip-homing` re-measures ranges without writing anything. The maths is
  ROS-free and unit-tested (`test_calibrate.py`).
- `arm_offsets` (`pixi run arm-offsets show|backup|restore|set`) — the offset
  register is the **only arm state with no copy outside the servo**, so
  overwriting it destroys the previous value. `arm-calibrate` snapshots the
  existing offsets to `~/.mote/arm_offsets_backup.yaml` before its first write,
  writes/verifies/confirms each servo one at a time, and on any failure stops
  and points here — *including* a failure to save `arm.yaml` afterwards, which
  leaves servos calibrated and the config file not, so the soft limits would
  describe a frame the arm has stopped using. This exists because a run once
  died mid-write on a dropped serial read, leaving a part-calibrated arm with no
  way back. **Servos can
  arrive with non-zero offsets** (this arm: 2027, -1723, 1772, -1706, -40,
  1317), so the existing value is always read and folded in.
- **Reads on this bus are hazardous twice over, and `FeetechBus._read` is the
  single choke point for both.** It clears the input buffer before every read,
  because a late reply is otherwise consumed as the answer to the *next*
  request: observed on hardware as a PRESENT_POSITION read returning 3902, which
  is exactly the -1854 just written to the offset register in sign-magnitude.
  Anything read right after an EEPROM write needs `read_position_settled` /
  `read_gains`-style two-agreeing-reads, not a single read. And: `scservo_sdk`'s
  `read2ByteTxRx` indexes the reply buffer before checking its length, so a
  dropped packet raises `IndexError` rather than reporting failure — which
  crashed the above. A read that does not come back is `None`, never an
  exception.
- `poses.py` + `arm_pose` (`pixi run arm-pose`) — teach/replay named poses
  (`~/.mote/arm_poses.yaml`, `MOTE_HOME`-overridable), the arm's analogue of
  `save-zone`. `go` refuses moves over `--max-travel`. Changing `home`
  invalidates stored poses. `arm-pose limits` is **not** the calibration path:
  it widens outward from taught poses, so it only describes where the arm has
  been and never finds the stops (which is why the committed limits give barely
  moved joints a near-zero band) — its remaining use is *narrowing* to a working
  envelope inside calibrated hard stops. The committed values are still that old
  envelope output, flagged as provisional in `robot.yaml` pending a real
  calibration pass (`mote_arm/BENCH.md` step 3) — a real run showed the as-found
  parked pose sits within ~20 counts of a hard stop on most joints, which is why
  those bands are ~0.2 rad against the 3.4-3.6 rad the joints actually travel.
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
- `arm_gains` (`pixi run arm-gains show|apply|sweep`) — the servos' position-loop
  gains live in EEPROM, i.e. invisible config a servo swap would silently
  revert, so `robot.yaml`'s `arm.gains` is the source of truth and this tool
  reconciles hardware with it. The arm shipped `Kp=16`, which left permanent
  droop under load (the servo settles where `Kp x error` balances the holding
  torque; `Ki=0` never integrates it away). Swept on elbow at -0.200 rad:
  Kp=16/32/64/128 -> error 0.068/0.031/0.014/0.008 rad at load 188/168/144/144
  of 1000, no ripple or reversals anywhere — error falls 8.2x for an 8x gain
  rise at a load nowhere near saturation, so it is droop, NOT torque
  saturation, and the 5 V supply was never the binding constraint (repeated to
  1-2 counts; same law on a 1.0 rad step and at double speed). **Kp=64 is
  applied**, not the better-scoring 128: the sweep only measures an unloaded
  static hold, and a stiffer loop reacts harder to the payloads and collisions
  a fetch arm exists for — revisit with a payload, not from the table. **Ki
  stays 0**: ki=8 closes the error to 0.001 rad but stretches settling 0.46s ->
  2.12s, which `arm-pose`'s 20 Hz streamed setpoints never wait for. The arm
  completes the full 3.19 rad home<->reachy move both ways with 0.012-0.028 rad
  residual (0.026-0.041 at Kp=32). Gotcha: an EEPROM
  read-back races the relock — wait ~150 ms and read twice, or a single read can
  return a garbled 250 and make a successful write look failed. `sweep` is how a
  gain is chosen rather than guessed: it steps one joint (`elbow_flex` — the only
  one with room in its soft limits) under each candidate gain, scores the
  response (`step_response.py`: error, kp*error, load, settling, ripple and
  reversals — the last two are the buzz check that bounds how high Kp may go),
  writes the trace to `~/.mote/arm_gain_sweeps/`, and restores the gains and
  limpness it started with, so a sweep on its own changes nothing.
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
