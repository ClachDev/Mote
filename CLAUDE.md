# CLAUDE.md

Rules and traps for this repository. Reasoning and measurements live in the docs
it points to; read the pointed-to doc before changing what it covers.

**Editing this file.** Add a rule only if breaking it fails silently, as one
bullet under the existing heading. The story, the evidence and the measurements
go in the package README, `docs/fleet/` or `docs/tuning/`. Never add a section
per feature. Keep the file under 5,000 words.

## Build & Common Commands

All tasks run via [pixi](https://pixi.sh). Never invoke `colcon` or `ros2` directly — always `pixi run <task>`.

```bash
pixi run build          # Build all packages with colcon + Ninja
pixi run submodules     # Fetch git submodules (sllidar_ros2, kinematic_icp)
pixi run launch         # Full robot bringup (hardware + lidar + camera + localization)
pixi run slam           # SLAM stack only (run alongside launch)
pixi run nav            # Nav2 stack (loads the active site's map)
pixi run mapping        # bringup + SLAM together (build/extend a map)
pixi run robot          # bringup + Nav2 together (drive the active site's map)
pixi run save-map       # Save map + slam posegraph into the active site floor
pixi run publish-map    # Offer that map to the fleet registry as a candidate
pixi run save-zone <n>  # Teach a zone from the current robot pose (--radius, --note, --no-navigable)
pixi run segment-map    # Propose a zone per room of a saved map (--write merges into zones.yaml)
pixi run site           # Site CLI: create / add-floor / use / use-map / list / info
pixi run teleop         # Keyboard teleoperation
pixi run explore        # Autonomous mapping coverage (beside `pixi run mapping`, on the Pi)
pixi run tasks          # Task layer: behaviour-tree task_server
pixi run arm            # SO-101 arm bench control stack (ros2_control, no mission)
pixi run arm-setup check|calibrate|limits|offsets|gains   # Arm bus tools (control stack stopped)
pixi run arm-pose       # Teach/replay named arm poses; narrow the envelope
pixi run arm-teleop     # Keyboard teleop: clamped, rate-limited arm_controller goals
pixi run arm-record | arm-replay | arm-mock | arm-teleop-test   # episodes; mock arm (mote_arm/TELEOP.md)
pixi run sync           # rsync project to Pi at SSH host 'mote'
pixi run setup          # One-time Pi setup: udev + wifi + systemd (needs sudo)
pixi run wifi-check     # who takes the roam decision (+ wifi-roaming/-roamlog/-powersave)
pixi run setup-ids      # Guided servo ID assignment tool
pixi run kill           # Kill this checkout's ROS processes and reset daemon
pixi run sweep          # Report ROS processes leaked by dead agent jobs (--kill to reap)
pixi run identity       # Fleet identity CLI: show / id / set --id --name --site
pixi run tailnet        # Join this machine to the Tailscale overlay (needs sudo)
pixi run dds-check      # DDS participant-slot headroom on this host
pixi run enroll         # Get this robot's id from the fleet server
pixi run agent          # Robot -> fleet bridge (mote-agent.service)
pixi run foxglove       # foxglove_bridge + teleop relay: the remote console
pixi run fleet-server   # Off-board: fleet API + operator dashboard
pixi run fleetctl       # Operator CLI: token / operator / robots / dispatch / promote / audit / watch
pixi run fleet-broker   # Off-board: MQTT broker + WebSockets (a container)
pixi run fleet-ui-check # Fake fleet + headless Chrome dashboard check (-- --keep to leave it up)
pixi run fleet-deploy   # Fleet box: container stack up/update/rollback/backup
pixi run inference-deploy  # GPU box: blue/green deploy (deploy-test: stubbed, no GPU)
pixi run test           # colcon test (gtest + pytest)
pixi run docs           # mkdocs site (docs-build: CI, --strict)

# Dev environment only (ros-jazzy-desktop)
pixi run rviz           # RViz2 with mote config (rviz-sim joins the sim's graph)

# Sim environment only (gz-sim Harmonic; own solve, never affects the Pi env).
# sim/sim-test tasks auto-select the env; ad-hoc commands need `pixi run -e sim -- ...`
pixi run sim            # Headless Gazebo sim: world + robot + controllers (no mission)
pixi run sim-mapping    # = sim mode:=mapping (the real mapping_launch.py)
pixi run sim-nav        # = sim mode:=nav (the real robot_launch.py + saved map)
#   pixi run sim world:=hospital_world.sdf   (mote_world easy, office_world medium, hospital_world hard)
pixi run sim-test       # ~20 s headless smoke test (local pre-PR gate, needs a GPU)
pixi run bench          # Nav benchmark vs Gazebo ground truth (mote_simulation/tools/benchmark)
pixi run segment-eval   # Score room segmentation
pixi run sim-map-reload-test  # A promoted map loads into a running Nav2 (~2 min, GPU)

pixi run -e dev test-fleet        # mote_fleet tests incl. the real-broker e2e run

pixi run -e lerobot arm-export -- --capture ~/.mote/episodes/<name>  # torch env, off-board

# Lint env (pre-commit; auto-selected)
pixi run lint           # all pre-commit hooks (~1 s cached)
pixi run lint-install   # wire pre-commit into .git/hooks (once per clone)
```

Artifacts go into `build/`, `install/`, `log/` (git-ignored). A CMakeCache "wrong source directory" error means a stale `build/`: delete it.

## Where to read first

| Topic | Doc |
|---|---|
| Fleet design + milestones (M0–M7, Ms) | `docs/design/fleet.md` |
| Fleet operator runbook | `docs/fleet/README.md` |
| MQTT wire contract | `docs/fleet/control-plane.md` |
| HTTP wire contract | `docs/fleet/fleet-api.md` |
| Per-milestone measurements | `docs/fleet/m*-verification.md`, `ms-verification.md` |
| Server/inference deploys | `docs/fleet/server-pipelines.md`, `docs/inference-server.md` |
| Mapping pipeline design | `docs/design/mapping-pipeline.md` |
| Sites, maps, zones | `docs/robot/sites.md`, `mote_bringup/mote_bringup/sites.py` docstring |
| Drive path, reliability, stray processes | `mote_bringup/README.md` |
| Wifi roaming | `mote_bringup/wifi/README.md` |
| Health monitor | `mote_health/README.md` |
| Arm | `mote_arm/README.md`, `BENCH.md`, `TELEOP.md` |
| Perception | `mote_perception/README.md`, `mote_perception/config/README.md` |
| Tasks | `mote_tasks/README.md` |
| Simulation | `docs/simulation.md`, `mote_simulation/tools/benchmark/README.md` |
| Tuning evidence | `docs/tuning/<date>-<topic>.md` |
| Hardware, BOM, wiring | `design/README.md`, `design/BOM.md`, `design/WIRING.md` |

## Robot configuration

**`mote_description/config/robot.yaml` is the single source of truth** for wheel geometry (`wheel_radius`, `wheel_separation`), the servo bus (port, baud, IDs, `velocity_scale`, acceleration), sensor device paths, and the arm's design (IDs, names, direction, gains, placeholder limits). The URDF reads it via `xacro.load_yaml`; launch files via `yaml.safe_load`. Neither duplicates a value. `velocity_scale` is an empirical calibration from the `velocity_cal` tool, not a datasheet number.

Three files are called some form of robot config — do not confuse them:
- `mote_description/config/robot.yaml` — shared hardware description.
- `$MOTE_HOME/robot.yaml` — this robot's fleet identity.
- `$MOTE_HOME/arm.yaml` — this arm's calibration.

## Per-robot state vs shared config

**`MOTE_HOME` (default `~/.mote`) is per-robot state; the package is shared config.** `mote_bringup/mote_home.py` is the one place that rule lives (`mote_dir()`, `path()`, `override(name, packaged_default)`). Everything that reads per-robot files resolves through it, so an update can never clobber identity, site selection, calibration, maps or bags. The one duplication is `mote_health/src/mote_home.cpp` (C++ cannot import Python), pinned to the same cases by `test_mote_home.cpp`. Details: `docs/fleet/README.md` §3.

## DDS scoping

- DDS transport is **loopback-only everywhere**: `config/cyclonedds.xml`, loaded via `CYCLONEDDS_URI` by pixi activation and the systemd units. No machine joins another's DDS graph. Foxglove is the off-box window. Camera calibration, the one flow needing a LAN peer, unsets the profile explicitly.
- Every `mote-*.service` sets `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` — **all of them**, because a localhost-range participant discovers a default-range one on the same host but not the reverse. Interactive `pixi run` keeps stock discovery.
- rmw_cyclonedds caps localhost discovery at **33 participants per host** (`MaxAutoParticipantIndex=32`). The full robot stack sits at ~27. Run `pixi run dds-check` whenever you add a process; raise the cap in `cyclonedds.xml` if needed.
- The `sim` env sets `LOCALHOST` too. Every gz entry point (`bench.py`, smoke test, `map_world.sh`) claims a free `ROS_DOMAIN_ID` + `GZ_PARTITION` through `mote_simulation/tools/sim_domain.py`. **Teardown must be scoped as well**: each launch is `setsid`-ed and reaped by session id, never by bare name (`pkill -f mote_world` is what this replaced).

## Fleet

Milestones and rationale: `docs/design/fleet.md`. Built: M0 (overlay + identity), M1 (agent + control plane), M2 (Foxglove), M3 (dashboard + dispatch API), M4 (map registry), Ms (server pipelines), M7 in part (API auth, tailnet policy, locked installs). Outstanding: M5 (OTA, waits on an aarch64 prefix.dev package), M6 (second robot), M7's broker half (per-principal broker credentials + ACL; the broker is still anonymous).

### Wire contracts

- **Two versioned contracts**: MQTT in `docs/fleet/control-plane.md`, HTTP in `docs/fleet/fleet-api.md`. Read the relevant one before touching a payload or route. `protocol.py`, `schema/*.schema.json` and `control-plane.md` are held together by `test_protocol.py`.
- Topic tree: `mote/v2/<robot_id>/{presence,health,pose,capabilities,mission/command,mission/status}`, plus `mote/v2/registry/site/<site>/floor/<floor>/current`. `registry` is therefore a reserved robot id. Every payload carries `schema: 1`.
- **Everything is retained except `mission/command`** — a retained command would re-fire on every reconnect. `presence` is also the MQTT Last Will.
- **Payloads are forwarded, never rebuilt.** The agent forwards task-server JSON byte for byte; the fleet server forwards what it last saw. No field is added, renamed or reinterpreted. There is one definition of each payload.
- `robot_id` is a lowercase DNS label: it is a MagicDNS hostname, an MQTT topic level and a directory name at once. It is not the hostname. Enrollment is idempotent on the hardware fingerprint and allocates `mote-NN` inside a `BEGIN IMMEDIATE` transaction.
- **A v1 robot and a v2 server do not interoperate**, deliberately: a translating shim would be a third definition of the wire.

### Specs (mission/v0, capability/v0, zone/v0)

Mote is the reference implementation of the Augere specs. Contracts live in `mote_bringup/mote_bringup/spec/` — **ROS-free and stdlib-only**, because the task layer, the agent and the fleet server all use them and none may depend on the others.
- `spec/mission.py` enforces three rules callers get wrong silently: `terminal` is computed from `state`; `failure` is required on `rejected`/`failed` and refused elsewhere; `recoverable` must be stated for `precondition`, `unresolved_zone` and `timeout`.
- The input validator is a bounded JSON Schema subset that **raises on any keyword it does not implement**. `check_schema` runs when a capability is declared.
- **Vendor no copy of the specs' schemas.** `test_spec_conformance.py` validates against the spec checkout (`$AUGEREAI_SPEC` or sibling `augereai-spec/`) and needs `jsonschema` (dev env only): run `AUGEREAI_SPEC=<checkout> pixi run -e dev test`.
- `ZONE_NAME_RE` is copied verbatim into zone/v0's zone reference; changing it needs a spec revision.
- The ROS seam is `std_msgs/String` carrying JSON, not a custom message, so the agent can forward bytes unchanged.

### Agent and dispatch

- The agent is **a bridge and reporter, never in the control loop**. It runs as its own `mote-agent.service` (installed, not enabled), not inside `pixi run robot`, so a robot that cannot reach the fleet still boots and navigates.
- **The lane belongs to the executor** (`task_server`): one mission at a time, `busy` names the holder. `dispatch.py` keeps only what the agent alone can answer: dedup of a redelivered id, one-hour terminal retention, the 20 s no-verdict timeout, and `source` (`local` for a mission the agent did not dispatch).
- Health is `/diagnostics_agg` forwarded, not recomputed; `state: unknown` with no monitor. `battery` is always null — nothing measures it.
- There is **no cancel**. `mission/cancel` is a reserved leaf with no publisher and both capabilities declare `cancellable: false`. The operator's tool is the `/pause_navigation` lock.

### Fleet server and API

- **Reads ride MQTT, writes ride HTTP.** The dashboard subscribes to retained topics over WebSockets. Every write to `mission/command`, from CLI or browser, goes through `POST /v1/robots/<id>/dispatch`, which authorizes and audits.
- **`fleet_server.ROUTES` is the route table and the auth gate.** A new route is authenticated by default and must opt out on its own line. Anonymous callers are refused before existence is checked, so a 404 reveals nothing. `test_fleet_server.py` walks the table; a route with a new path variable needs a `SAMPLES` entry. The open entries are `/healthz`, `POST /v1/enroll`, and the robot's map upload and `bundle.tar.gz` pull; the static UI is served to unmatched GETs.
- The server holds its own subscription (`BrokerFeed` → `RobotState`) so `GET /v1/robots[/<id>]` can answer without the client touching MQTT. Absent state is null per field; a never-heard-from robot is `200` with nulls, `404` means not enrolled. Nothing is persisted; a cleared retained topic clears the field.
- In `serve()`, `publisher` and `feed` are injected as a pair.
- The mission `input` is validated only by the robot against its own capability schema — never duplicate it in the server.

### Dashboard (`mote_fleet/server/ui/`)

- Static ES modules: **no bundler, no npm, no vendored library**. Served by the stdlib `http.server`.
- `mqtt.mjs` is a hand-rolled **subscribe-only** client with no PUBLISH packet. Keep it that way.
- The dispatch form is **generated from the robot's capability set**; a zone picker appears where a property `$ref`s zone/v0's zone reference. The page holds no list of capabilities or zone inputs.
- Basemaps are fetched with the token and decoded from a blob (`loadImage`), because `<img src>` sends no `Authorization` header.
- The 760 px breakpoint lives in `layout.mjs`; `ui_test.mjs` reads `style.css` to keep CSS and JS agreeing, and asserts `touch-action: none` on the canvas.
- **A canvas gets no CSS cascade**: read colours from computed style (`--dim`, ink). The basemap's free space is white in both themes.
- `[hidden]` needs the `display: none !important` rule already in `style.css`; don't remove it.
- `ui_test.mjs` runs under node against the files the browser loads. `fleet-ui-check` needs docker and Chrome, so it stays out of CI.

### Map registry (M4)

- **Uploading is not publishing.** `publish-map` uploads an inert candidate. Only an operator's promote (`fleetctl promote` or the review pane) flips the floor's `map` symlink and publishes the retained `…/current` topic. Two mappers leave two candidates, never a merge.
- `mote_bringup/bundle.py` is the shared, ROS-free validator (content); `sites.py` owns layout. It uses **PyYAML and Pillow** — do not replace them with hand-rolled readers (it was tried and broke on `Café`, polygons and PNG decoding; `m4-verification.md` §2, §8). The fleet image copies these two files; the fleet box installs no ROS.
- The flip and the announcement are reported separately; the server re-announces every floor at startup.
- On the robot, `mapsync.py` stages a pull in a temp dir, verifies sha256, renames into `maps/<rev>/` and flips; the agent then publishes latched `map/installed`.
- **`map_reloader`** (`map_reloader.py`, ROS-free half `map_reload.py`; started by `nav2_launch.py` only with `localisation:=true`) loads that revision into the running Nav2. Rules:
  - **Ask the lane holder, never infer.** It calls the task server's `task/idle`; a mission in flight defers the load to its terminal state. While `map/serving.loading` is set the task server refuses missions as `busy` (bounded at 30 s).
  - Load the revision's own `maps/<rev>/map.yaml`, never the symlink, so what is served is what is reported.
  - Re-seed AMCL from the last `amcl_pose` on `/initialpose` — not a TF lookup (a `TransformListener` costs the whole `/tf` stream). AMCL needs `first_map_only: false`, pinned in `nav2_params.yaml`.
  - A failed load changes nothing but health's `map.error`. Health's `map.revision` is what `map_server` serves; `map.installed` is the symlink.
  - The task server reloads the floor's zones when the served revision changes.
  - `site use-map` tells nobody, so a local rollback still needs a nav restart.
  - Flow and measurements: `docs/fleet/README.md` §11.
- The review pane's revision routes: `…/revisions/<rev>/{map.json,map.png,zones.json}`. `image_url` must be revision-aware; revision zones are not gated on a published map and report `source: revision|floor`.

### Zone editor

- **Editing is a derivation, never a mutation.** A save repacks the revision under review with the new zones and `accept()`s it as a new candidate. Stored revision bytes never change.
- A derivation is held to promote's bar (`accept(require_posegraph=False)` warns), not the upload's.
- The editor lives only in the review pane. It refuses client-side what the robot's loader refuses (`ambiguities` mirrors `bundle.ambiguities`).
- One `hitTest` decides drag target, cursor and hover highlight — don't fork it.
- `snapToPixel` for dragged coordinates (shift bypasses); a body drag snaps its delta, measured from the grab. **Never re-snap a coordinate the operator did not touch**, and never snap a typed one.
- Moved geometry is stamped `source: editor` via `sourced`. UI principles: `docs/fleet/README.md` §11.

### Zones

- **A zone is a coordinate in the floor's frame** — a fact about the building. A floor's zones are one file, `floors/<floor>/zones.yaml`, owned by the floor. A revision carries a copy; `mapsync.install` replaces the floor's file and keeps the old one as `zones.<old-rev>.yaml`.
- **A zone is a place-name**: `name` (human, e.g. `store room`; `zone.ZONE_NAME_RE`), `note`, `navigable`, `x`/`y`/`yaw`, optional `radius` or `polygon`, and `source` (`save-zone` | `segment-map` | `editor`). `source` is provenance only; nothing may decide anything from it.
- Retired fields (`kind`, `display_name`, `aliases`, `parent`, `tags`, `description`) are **accepted on read, never written or served** (`LEGACY_KEYS`). `description` reads into `note`; `kind: keepout|slow` seeds `navigable: false`.
- zone/v0's vocabulary and binding documents are **views built at the wire** by `spec/zone.py` (`vocabulary()`, `binding()`), never stored. The names-only view is **built from `VOCABULARY_KEYS`, never stripped**. `_ANCHOR_METHOD` is the one `source` → `anchor.method` mapping.
- `resolve` matches name exactly, then case-insensitive and whitespace-normalised. Non-navigable zones are refused as destinations. `load_zones` refuses an ambiguous set. `problems` over `/v1/zones` is reported, not enforced.
- `/v1/maps/...` is served beside a basemap and gated on one; `/v1/zones/...` serves names only and is gated on nothing.
- The one open disagreement with zone/v0 is its one-frame-per-platform premise (#629).

### Tailnet, provisioning, server deploys

- The tailnet policy is `mote_bringup/tailscale/policy.hujson`; its `tests` block asserts no robot reaches another. Every tag `install.sh` advertises must be declared there (`test_tailnet_roles.py`).
- A clean Pi is one rendered cloud-init file (`provision.py` + `provisioning/user-data.template`), which runs `pixi install --locked`.
- The deployed broker reads `server/mosquitto.conf`, the same file `fleet-broker` uses; its image tag is pinned once, to `2.1-alpine`. Deploys: `docs/fleet/server-pipelines.md`.
- conda-forge's mosquitto lacks websockets; use the container broker.

## Sites (maps & zones)

Everything meaningful relative to one mapped place lives in a **site bundle** under `$MOTE_HOME/sites/<site>/floors/<floor>/`, managed by `mote_bringup/sites.py` (docs in its docstring and `docs/robot/sites.md`).
- `$MOTE_HOME/active.yaml` selects site/floor; launch files resolve the map and zones from it at launch time.
- Map revisions are immutable under `maps/<rev>/`, published by atomically flipping the `map` symlink. `site use-map <rev>` rolls back.
- `save-map` stores the posegraph for **continuation** — extend, don't remap; it is what keeps a new map in the floor's frame. It also stamps the session's bag into `meta.yaml`, runs `bundle.py`'s validation locally, and runs the FFT **cleaning pass**: `map_raw.png` is kept, the cleaned image is served as `map.png`. The posegraph belongs to the raw map.
- `segment-map` (`map_cleanup/room_segmentation.py`) proposes one polygon per room; `--write` merges additively into the floor's zones. It assumes a doorway is narrow and walls are Manhattan after rotation; corridors are not proposed. Evidence: `docs/tuning/2026-07-27-room-segmentation.md`.

## Drive path

`DiffDriveController` has exactly one publisher: **`twist_mux`** (`twist_mux_launch.py`, included by `mote_launch.py` and `sim_launch.py`). Nav2 publishes `/cmd_vel_nav` (priority 10). Everything a human drives with publishes `/cmd_vel_teleop_stamped` (priority 100): `twist_relay`, `pixi run teleop`, the RViz teleop panel, and `explore`. The mux output keeps the name `/diff_drive_controller/cmd_vel`.
- **Teleop overrides Nav2; it does not cancel it.**
- **The teleop timeout (1.0 s) must stay longer than the controller's `cmd_vel_timeout` (0.5 s)**, so a takeover ends with a stopped robot. `test_twist_mux.py` holds the files together; `test_twist_mux_arbitration.py` measures a real mux.
- Nothing may re-publish a stale command: `twist_relay` and the mux publish only from input callbacks, so every source stopping means the wheels stopping.
- `use_stamped` is not a declared node parameter; don't set it in the launch.
- `/pause_navigation` (`std_msgs/Bool`, priority 50) locks out Nav2 but not teleop. A pause over ~10 s fails the Nav2 goal via its progress checker.
- Details: `mote_bringup/README.md` "Drive path".

## Architecture

Differential-drive robot on **ROS 2 Jazzy**.

### `mote_hardware` (C++)
`ros2_control` `SystemInterface` (`MoteHardware`) driving two Feetech STS3215 wheel servos over the SCServo SDK.
- Params come from `robot.yaml` via the URDF's `<ros2_control>` tag (`info_.hardware_parameters`).
- Position is tracked across the 12-bit encoder rollover with a half-range threshold. The left wheel is inverted in both `read()` and `write()`.
- The port opens in `on_activate`, which also sets wheel mode (an EEPROM write, skipped if already set). `port_guard.cpp` refuses activation if another process holds the port.
- **It is the single owner of the shared servo bus**: it also exports position interfaces for the six arm servos (`arm_joint.hpp` mirrors `mote_arm/config.py`). Arm state is read one joint per cycle, round-robin; goals go out as one sync-write only when changed. The loop is 50 Hz with the wheels on the same bus — keep arm traffic that cheap.
- Tools: `mote_hardware/tools/README.md`.

### `mote_description` (CMake)
`urdf/mote.urdf.xacro` + `config/robot.yaml`. No xacro args are needed except `use_sim:=true` (sim), `arm:=false` (sim) and `arm_config:=` (calibrated arm, passed by launch).
**Known defect:** `arm_mount_joint` is `rpy="0 0 0"` but the arm is physically mounted rotated 180° (issue #2). Joint-space work is unaffected; anything in base coordinates is 180° out.

### `mote_nav` (C++)
C++ that runs inside other processes. Shared numbers (`max_wheel_speed`, `wheel_separation`) come through dependency-free `include/mote_nav/wheel_speed.hpp`.
- `WheelSpeedLimitCritic` (DWB plugin) rejects samples needing a wheel faster than `max_wheel_speed`. `nav2_launch.py` overlays both numbers from `robot.yaml`.
- `OdomTfRelay` (component) writes the `odom_wheel` leaf kinematic_icp uses as its motion prior. Built `-ffp-contract=off` to stay bit-identical with the Python it replaced.
- `IcpOdomGate` (component) **owns `odom`→`base`**: it accumulates kinematic_icp's increments and substitutes the wheel increment for any the drive could not produce. Translation and yaw are bounded separately at `robot.yaml` limits × 1.15. Evidence and tools (`icp_excursions.py`, `icp_gate_replay.py`): `docs/tuning/2026-07-28-icp-velocity-gate.md`.

### `mote_health` (C++)
The health monitor, its own process because `mote-health.service` is `Type=notify` with a watchdog. Publishes the `mote` roll-up and per-subsystem statuses on `/diagnostics_agg`, and one line on `/health`. Config: `config/health.yaml` (override `$MOTE_HOME/health.yaml`).
- It is C++ because **a monitor's cost is its wake-ups, `/tf` included** (10.6 → 1.1% of a core). Evidence: `docs/tuning/2026-09-01-health-monitor-cpp.md`, `2026-08-11-monitor-cpu.md`.
- Subscriptions are generic (`create_generic_subscription` with a runtime type string), so `health.yaml` needs no codegen.
- `src/health_rollup.cpp` is free of rclcpp/tf2/yaml-cpp and holds the behaviour; test it with gtests.
- `test/compare_monitors.py` diffs two monitor builds against one synthetic stream — use it for behaviour changes.

### `mote_bringup` (Python/ament)
Launch files, config, udev rules, systemd units, `wifi/`, the fleet foundation (`mote_home.py`, `identity.py`, `provision.py`, `dds_participants.py`, `twist_relay.py`), `explore.py`, the `foxglove/` layout, `tailscale/`, `bundle.py` and `spec/`.

**Launch hierarchy.** The two mission launches take `base` (default true) and `use_sim_time`, forwarded to everything they include. The sim runs the *same* files with `base:=false`.
- `robot_launch.py` — `mote_launch.py` (if `base`) + `nav2_launch.py`. Takes `map`, defaulting to the active site's map.
- `mapping_launch.py` — `mote_launch.py` (if `base`) + `slam_launch.py` + `nav2_launch.py localisation:=false` + `record_launch.py` (`streams:=mapping`, unless `record:=false`).
- `mote_launch.py` — the hardware base: robot_state_publisher, ros2_control_node, spawners, sllidar, laser_filter, v4l2_camera, `localization_launch.py`, `twist_mux_launch.py`, `foxglove_launch.py` (`foxglove:=true`; the systemd unit passes false because `mote-foxglove.service` runs it independently), plus `system_monitor` and `slip_monitor`. Sets `use_sim_time` via `SetParameter`.
- `localization_launch.py` — kinematic_icp, `OdomTfRelay` and `IcpOdomGate` in one `localization_container`. It does not run AMCL. ICP runs `lidar_odom_frame:=odom_icp` with `invert_odom_tf:=true`, writing a `base`→`odom_icp` **leaf** that `slip_monitor` reads (the gated edge would hide exactly the faults it detects). Frame names are constants in `launch_utils.py`. `test_localization_composition.py` holds these seams — every failure here is silent.
- `nav2_launch.py` — all Nav2 servers + lifecycle managers as `ComposableNode`s in one `nav2_container` (`component_container_isolated`). **The params file goes on the container too** (costmaps and BT client nodes inherit the process command line). **Every `ComposableNode` needs `name=`** matching its `nav2_params.yaml` key, or it gets no parameters. Component loads re-issue on every `OnProcessStart` so a respawned container is refilled. `localisation` toggles map_server + AMCL. slam_toolbox stays its own process.
- `foxglove_launch.py` — bridge on 8765 + `twist_relay`.

**Config** (`mote_bringup/config/`):
- `controllers.yaml` — controller_manager, DiffDriveController (`cmd_vel_timeout: 0.5` pinned), the arm's `JointTrajectoryController`. Wheel geometry and arm joints are injected from `robot.yaml` via `launch_utils.joint_params_file`, keyed by node name.
- `twist_mux.yaml`, `laser_filters.yaml`, `nav2_params.yaml`, `mote.rviz`, `cyclonedds.xml`, `slip.yaml`.
- `slam_toolbox_params.yaml` — the live config: best-known-good values only, never deliberately hobbled.
- `slam_toolbox_build_params.yaml` — the offline-build copy. Every value must match the live file unless the key carries a `# DIVERGENCE:` note; `test_slam_build_params.py` enforces both directions.

**Foxglove.** Bridge 3.3.0 accepts only the `foxglove.sdk.v1` subprotocol. `twist_relay` stamps the panel's `Twist` on the robot, with no timer.

**On-robot reliability** (`mote_bringup/README.md`):
- systemd units are installed by `pixi run setup` but **not enabled**. They restart with backoff and never give up.
- `self_check.py` (`ExecStartPre`, `pixi run self-check`) gates bringup on servos, lidar, camera, disk, clock and config.
- `slip_monitor.py` compares wheel odometry with ICP over 1 s and reports `slip`/`stuck`/`icp_fault` — always DEGRADED, never FAULT. Only translation is thresholded. A stale source yields no verdict. The maths in `odom_residual.py` is shared with `tools/slip_replay.py`, which set the thresholds; keep them shared. Evidence: `docs/tuning/2026-07-28-slip-detection.md`.
- Battery is not measurable. `system_monitor` reads `get_throttled` via `vcgencmd` (no Pi 5 sysfs node) and the fan RPM.
- **Python monitors cost wake-ups, not work.** A `TransformListener` takes the whole `/tf` stream. `task_server` ticks at `idle_tick_period` between tasks; `_set_tick_rate` must reset the timer, not only re-period it.

**Stray ROS processes** (`sweep_orphans.py`, `mote_bringup/README.md`):
- Match processes on **identity in /proc** — ROS env, path under the checkout, not in the sweeper's ancestry — never on the command line. `pkill -f` matched and killed its own shell.
- **`ros2 run` does not forward SIGTERM.** Spawn it with `spawn_reapable` and stop it with `reap_group`, or the node survives as an orphan.

**Wifi** (`mote_bringup/wifi/README.md`): roaming is the firmware's (`roamoff=0`); the modprobe file must sort last by base name (`zz-mote-brcmfmac.conf`).

### `mote_simulation` (Python/ament)
Workstation-only; excluded from `pixi run sync`; built only in the `sim` env. Dependencies run one way: it includes from `mote_bringup` and `mote_tasks`, never the reverse.
- `sim_launch.py` runs headless gz, spawns the robot (URDF with `use_sim:=true`, swapping in `gz_ros2_control` and a sim lidar), bridges `/clock` and `/scan`, and reuses `mote_bringup`'s controllers, laser filters, localization and twist_mux. `mode:=mapping|nav` includes the real mission launches with `base:=false`, plus `tasks_launch.py` with the world's `worlds/<world>.zones.yaml`.
- `hospital_world.sdf` is **generated by `worlds/gen_hospital.py`** — edit the script, never the SDF.
- `sim_home/` is a committed sim `MOTE_HOME`, one site per world, built by `pixi run sim-map-world` and saved with `clean=False`.

### `mote_perception` (Python/ament)
Camera-derived perception. Nodes run on the robot; torch runs on an off-board inference server over TCP at `inference_host` (`config/perception.yaml`, overridable in `$MOTE_HOME`). Details: `mote_perception/README.md`, `docs/inference-server.md`.
- **L1 depth obstacles**: `depth_obstacle_node.py` (torch-free) → `/camera_obstacles` for Nav2's `camera_layer`. Supporting modules: `depth_wire.py`, `lidar_rescale.py`, `ground_projection.py`.
- `camera_layer` is **`spatio_temporal_voxel_layer`**, not `VoxelLayer`, because the cloud carries no floor points to clear with. **`clear_after_reading: True`** and **`filter: "passthrough"`** are load-bearing; both fail silently. `voxel_decay: 5.0`. `test_costmap_layers.py` holds these. Evidence: `docs/tuning/2026-07-29-camera-layer-decay.md`.
- **L2 open-vocabulary detection**: `object_detector_node.py` idles until labels arrive on `detect/labels`, then publishes `detected_objects` in the map frame. Floor-ray grounding is accurate only near the robot.
- `tools/inference_server.py` supervises the tenants in `SERVICES`; models load on demand and unload when idle.
- `perception_launch.py` is not part of mission bringup; run `pixi run perception` alongside.

### `mote_tasks` (Python/ament)
py_trees behaviour trees over Nav2 (py_trees from PyPI; no py_trees_ros). Details: `mote_tasks/README.md`.
- `capabilities.py` declares `goto {target}` and `fetch {target, destination}` using the standard registry's property names. Location inputs `$ref` zone/v0's zone reference.
- `task_server.py` publishes capabilities on latched `task/capabilities`, takes mission/v0 JSON on `task/command`, answers on `task/status`. It owns the lane, validates input against the capability schema, evaluates blocking preconditions (`localized`: `map`→`base_link` newer than 5 s; `zone_known`) and enforces `max_duration_s`. Failure class comes from what failed (`trees/common.py` `report_failure`).
- `zones.py`: `load_zones`, `resolve`, `containing`, `append_zone`. Polygons may be concave and may omit `x`/`y` (a pose inside is derived). A re-teach keeps name, note, `navigable` and footprint.
- `behaviours/`: `DriveTo`, `AcquireObject`, `TimedStub` (pick/place are still stubs). `trees/`: `fetch.py`, `goto.py`.
- Zones resolve from the active floor, then legacy `~/.mote/zones.yaml`, then `config/zones.default.yaml`.

### `mote_fleet` (Python/ament)
Both ends of the fleet wire. Details: `mote_fleet/README.md`.
- `protocol.py` — topic tree and payload builders; **stdlib-only, ROS-free**, imported by path on the server.
- `agent.py` takes an injectable MQTT client, so tests need no broker.
- `server/` — ROS-free: `fleet_server.py`, `registry.py` (SQLite under `$MOTE_FLEET_HOME`), `bundle_store.py` (filesystem is the truth about what is canonical), `fleetctl.py`, `ui/`, `mosquitto.conf`, `broker.sh`.
- e2e tests skip without a broker; `pixi run -e dev test-fleet` runs everything.

### `mote_arm` (Python/ament)
SO-101 follower arm (no leader arm), controlled directly over Feetech rather than via LeRobot, so the Pi carries no torch. Read `mote_arm/README.md` before changing anything here.
- **The arm is part of `MoteHardware`**, sharing the wheel bus (arm IDs 1–6, wheels 7/9). `arm_controller` is a `JointTrajectoryController` spawned **inactive**; claiming its interfaces enables torque. Arm/wheel ID collisions are rejected in both `config.py` and `MoteHardware`.
- `control.py` is the one command path. **Whether the arm is held is read from the controller manager, never assumed**; a refused switch is re-read before being reported.
- Calibration (`zero`/`min`/`max`) is per-robot, in `$MOTE_HOME/arm.yaml`. `launch_utils.resolved_arm` overlays it via `mote_arm.config.load` and passes `arm_config:=` to xacro. `mote_bringup` imports `mote_arm`, never the reverse.
- **`zero` is not `home`.** `zero` is the encoder count reading 0 rad; `home` is a taught pose in `arm_poses.yaml`. Do not reintroduce the collision.
- `arm-setup calibrate`: sweep first, then write each joint's offset (EEPROM 31, sign-magnitude, modular) **and immediately its goal-range fence (EEPROM 9/11)** — never one without the other. Existing offsets and fences are read, folded in and backed up first. Pose files migrate automatically.
- The **EEPROM fence refuses goals silently**, binds only under torque, and is compared against the corrected goal. The written band is the measured travel, wider than the soft limits. There is no `limits set`.
- **`FeetechBus._read` is the single read choke point**: it flushes the input buffer first, and a missing reply returns `None` (the SDK raises `IndexError`). After an EEPROM write, require two agreeing reads.
- **Every arm CLI uses `cli.shutdown` and `cli.parse`.** Shutdown joins the spin thread before destroying the node (otherwise exit 134). Parse strips `--ros-args` and parses strictly.
- Teleop: one process, one node. All safety rules (clamp, 0.5 rad/s rate limit, deadman, latched panic, re-seed on resume) live in ROS-free `teleop.py`. **The safety loop runs on its own thread**, because a service call from an executor callback never completes.
- Episode captures are stdlib-only on the Pi; `tools/lerobot_export.py` converts them through LeRobot's own API, never a hand-rolled writer.
- Gains are EEPROM state reconciled from `robot.yaml` (`arm-setup gains`): Kp=64, Ki=0, chosen by `sweep`.

### Third-party submodules (`third_party/`)
- `sllidar_ros2` — SLAMTEC RPLIDAR C1 driver
- `kinematic_icp` — LIDAR odometry (reads wheel odom TF as its prior)

## Device naming

`mote_bringup/udev/99-mote.rules` creates `/dev/mote_servos` (Waveshare bus board), `/dev/mote_lidar` (RPLIDAR C1) and `/dev/mote_camera` (USB webcam). With identical USB-serial adapters, pin by serial number (see the rules file).

## Environment

pixi activates `install/setup.sh`, sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and `CYCLONEDDS_URI`. Dependencies come from the `mote` prefix.dev channel, robostack-jazzy and conda-forge. The default env runs on the robot (Raspberry Pi 5, deployed with `pixi run sync`); `dev` adds `ros-jazzy-desktop`.

## Verification

- xacro/URDF changes: `pixi run -- xacro install/mote_description/share/mote_description/urdf/mote.urdf.xacro`.
- Controller param injection: run ros2_control_node on the xacro output with `mock_components/GenericSystem`, then `ros2 param get /diff_drive_controller wheel_separation`.
- Actual motion needs the Pi with hardware.
