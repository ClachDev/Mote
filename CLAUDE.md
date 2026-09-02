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
pixi run explore        # Autonomous mapping coverage (run beside `pixi run mapping`, on the Pi)
pixi run tasks          # Task layer: behaviour-tree task_server (see mote_tasks)
pixi run arm            # SO-101 arm: bench control stack (ros2_control, no mission)
pixi run arm-jog        # Interactive per-joint jog CLI (needs a stack owning the bus)
pixi run arm-check      # Standalone arm bus enumeration + health (read-only, base stopped)
pixi run arm-calibrate  # Range calibration: centre the joints, sweep, emit limits
pixi run arm-limits     # Servo goal-range fence (EEPROM 9/11): show / clear / restore
pixi run arm-pose       # Teach/replay named arm poses; narrow the envelope
pixi run arm-teleop     # Virtual-leader teleop: keyboard -> leader pose (mote_arm/TELEOP.md)
pixi run arm-mirror     # Mirror: leader pose -> clamped, rate-limited arm_controller goals
pixi run arm-mock       # The arm control stack's interface, no hardware (+ --camera)
pixi run arm-record     # Record teleop episodes into $MOTE_HOME/episodes
pixi run arm-replay     # Replay a recorded episode on the arm, gated
pixi run arm-teleop-test # Headless teleop->record->replay loop vs the mock arm
pixi run arm-bench-teleop # Guided hardware teleop session (needs a human)
pixi run sync           # rsync project to Pi at SSH host 'mote'
pixi run setup          # One-time Pi setup: udev + wifi + systemd (needs sudo)
pixi run udev           # Install udev rules + dialout group (needs sudo)
pixi run wifi-powersave # Disable WiFi power save via NetworkManager (needs sudo)
pixi run wifi-roaming   # Let the wifi firmware roam between the site's APs (needs sudo)
pixi run wifi-check     # Report what takes the roam decision (read-only)
pixi run wifi-roamlog   # Log BSSID/signal/loss during a roaming walk
pixi run setup-ids      # Guided servo ID assignment tool
pixi run kill           # Kill this checkout's ROS processes and reset daemon
pixi run sweep          # Report ROS processes leaked by dead agent jobs (--kill to reap)
pixi run identity       # Fleet identity CLI: show / id / set --id --name --site
pixi run tailnet        # Join this machine to the Tailscale overlay (needs sudo)
pixi run provision      # Render cloud-init user-data for a clean Pi
pixi run dds-check      # DDS participant-slot headroom on this host
pixi run camera-decay-check  # Time a real camera_layer mark expiring (~40 s, no hardware)
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

# LeRobot environment only (`lerobot`: torch + ffmpeg + the HuggingFace stack, no
# ROS; linux-64 only. Off-board, for the same reason `inference` is — the aarch64
# Pi records episodes but must not carry this.)
pixi run -e lerobot arm-export -- --capture ~/.mote/episodes/<name>  # capture -> LeRobotDataset
pixi run -e lerobot -- lerobot-dataset-viz --repo-id <id> --root <out> --episode-index 0

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

Milestone M0 of `docs/design/fleet.md`; the operator runbook is `docs/fleet/README.md` and the measurements behind it are `docs/fleet/m0-verification.md`. **`MOTE_HOME` (default `~/.mote`) is per-robot state; the package is shared config** — `mote_bringup/mote_home.py` is the one place that rule lives (`mote_dir()`, `path()`, and `override(name, packaged_default)` which prefers the per-robot file). `sites.py`, `mote_launch.py` (camera calibration), `perception_launch.py` (`perception.yaml`) and `self_check.py` (`self_check_status.yaml`) all resolve through it, so `MOTE_HOME` is honoured everywhere and an update can never clobber identity, site selection, calibration, maps or bags. The one place it is *stated twice* is `mote_health/src/mote_home.cpp`, because the health monitor is C++ and cannot import Python — the only such duplication, and `test_mote_home.cpp` pins it to the same cases. **Identity** is `$MOTE_HOME/robot.yaml` (`mote_bringup/identity.py`, `pixi run identity show|id|set`): a `robot_id` constrained to a lowercase DNS label because it is simultaneously a MagicDNS hostname, an MQTT topic level and a directory name. It is deliberately not the hostname, and operator-set until M1's enrollment endpoint allocates it. Do not confuse `$MOTE_HOME/robot.yaml` (this robot's identity) with `mote_description/config/robot.yaml` (shared hardware description). **The overlay** is Tailscale (`pixi run tailnet`, `mote_bringup/tailscale/install.sh`), joining robots/servers as *tagged* devices and the workstation as a user device; a robot's tailnet hostname *is* its `robot_id`. **A clean Pi** is provisioned by one rendered cloud-init file (`pixi run provision`, `mote_bringup/provision.py` + `provisioning/user-data.template`): identity → tailnet (single-use tagged auth key baked into the image, shredded after use) → pixi/build → `pixi run setup`. **DDS**: the end state is `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` on the robot, which retires the `ROS_DOMAIN_ID` isolation question entirely — **landed with M2** (below), which supplied the `foxglove_bridge` off-box path that had to exist first; the systemd units carry the pin, and `config/cyclonedds.xml` — loaded by pixi activation and the units alike — puts DDS transport on loopback only, so even an interactive run is invisible to the LAN (see DDS scoping under Environment). What M0 adds is the measurement: rmw_cyclonedds caps localhost discovery at `MaxAutoParticipantIndex=32`, i.e. 33 participants (≈ processes) per host, and `pixi run dds-check` reports the headroom from `/proc/net/udp` (measured 17/33 for the sim nav mission under both localhost and stock discovery; ~26 with perception, the M1 agent, M2's bridge and teleop relay, and the drive mux). Re-check it whenever a milestone adds processes; raise it in the robot's `cyclonedds.xml` if it runs out. The running total is kept in the Drive path section below, which is the latest thing to have moved it.

## Fleet: the control plane (M1)

Milestone M1 of `docs/design/fleet.md`, built in the **`mote_fleet`** package (below); the wire it speaks is specified once, as a versioned contract, in **`docs/fleet/control-plane.md`** — read that before touching a payload, and `docs/fleet/README.md` §6–8 for the operator flow. **Identity is now server-allocated**: `pixi run enroll` presents an enrollment token plus a hardware fingerprint to `POST /v1/enroll`, and the server (`mote_fleet/server/fleet_server.py` + a SQLite `registry.py`) allocates `mote-NN`, records the row, and answers with the broker address; the robot writes `$MOTE_HOME/robot.yaml` (identity, as M0) and `$MOTE_HOME/fleet.yaml` (server + broker). Enrollment is **idempotent on the fingerprint** (SoC serial → machine-id → MAC), so a re-enrolled robot is the same robot, and it **adopts** an M0 operator-set id rather than renumbering it. Allocation runs in a `BEGIN IMMEDIATE` transaction because ids are derived from the rows already present. **The topic tree is `mote/v2/<robot_id>/{presence,health,pose,capabilities,mission/command,mission/status}`** — the major version is in the topic root so a v3 can coexist during migration, and every payload additionally carries `schema: 1`. Everything is retained *except* `mission/command` (a retained command would re-fire on every reconnect); `presence` doubles as the MQTT Last Will, which is what makes "robot dropped off" instant. **The mission half of the tree is the open specifications' and not Mote's** — see "Fleet: adopting spec v0" below — which is what v2 is; the telemetry half moved with it unchanged, because a tree has one version and not one per leaf. **The lane belongs to the executor, and the agent keeps what only it can answer** (`mote_fleet/dispatch.py`): dedup of a redelivered id, retention of a terminal status for an hour, failing a mission the task server never answered, and `source` — a status for an id the agent did not dispatch is `source: local`, because on the robot a fleet mission and a bench one are the same message on the same topic. **The agent is a bridge and a reporter, never in the control loop**, and is deliberately not part of `pixi run robot`/`mapping`: it runs as its own `mote-agent.service` (installed, not enabled) so a robot that cannot reach the fleet server still boots and navigates. Health is the health monitor's `/diagnostics_agg` roll-up *forwarded*, not recomputed, with `state: unknown` when no monitor is reporting; `battery` is in the schema and always null (nothing measures it). Two packaging gotchas, both worked around and recorded in `docs/fleet/m1-verification.md`: conda-forge's mosquitto puts the **broker** in `$PREFIX/sbin` (pixi only puts `bin` on PATH) and is built **without websockets**, which M3's dashboard needs — M3 answers that with a container broker (below).


## Fleet: the remote console (M2)

Milestone M2 of `docs/design/fleet.md`; the operator flow is `docs/fleet/README.md` §10 and the measurements are `docs/fleet/m2-verification.md`. **Foxglove is adopted, not built**: `ros-jazzy-foxglove-bridge` runs on the robot (`foxglove_launch.py`, `pixi run foxglove`, port 8765) and an operator connects to `ws://<robot-id>:8765` over the tailnet — this is what finally replaces joining the robot's DDS graph from a workstation, and it is where M3's per-robot deep-link (`FOXGLOVE_URL` in `fleet_server.py`) lands. It is **included in the base bringup by default** (`mote_launch.py`'s `foxglove:=true`, exactly like `health`), so every way of starting the robot gives something to connect to, while `mote-bringup.service` passes `foxglove:=false` because `mote-foxglove.service` runs it independently — the view must outlive a bringup restart, since a crash-looping mission is when it is most needed. **Teleop needed a node, not just a layout**: Foxglove's Teleop panel publishes only unstamped `geometry_msgs/Twist` while `DiffDriveController` consumes `TwistStamped`, so `twist_relay.py` adds the header — deliberately event-driven, with no timer and no memory of the last command, so that when commands stop the robot stops. The deadman is `cmd_vel_timeout: 0.5`, now pinned explicitly in `controllers.yaml` rather than left to a default, and the stamp is taken *on the robot*, so the operator's clock never enters the safety path. The shipped layout is `mote_bringup/foxglove/mote.json` (+ its README); `test_foxglove_layout.py` ties it back to `controllers.yaml` so changing a velocity limit or the timeout fails a test rather than silently invalidating what an operator is handed, and `test_foxglove_teleop.py` (dev env, `pixi run -e dev test-foxglove`) drives the real bridge the way the panel does. **M2 also spends the DDS pin M0 deferred**: every `mote-*.service` now carries `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` — all of them, because a localhost-range participant discovers a same-host default-range one but not the reverse — so a systemd-run robot is invisible to the LAN, while an interactive `pixi run` keeps stock discovery and bench work with RViz is unaffected. One gotcha: bridge **3.3.0 speaks the `foxglove.sdk.v1` subprotocol** and refuses a client offering only the older `foxglove.websocket.v1` with a bare HTTP 400. Cost is 2 DDS participant slots, which took the full robot stack to ~25 of 33 at the time (~26 now — see Drive path). M2's other gotcha — teleop not pre-empting Nav2 — was closed afterwards by the drive mux (see Drive path below), which is why `twist_relay` now writes `/cmd_vel_teleop_stamped` rather than the controller's topic.

## Fleet: the operator view + dispatch API (M3)

Milestone M3 of `docs/design/fleet.md`, and the end of v0. The **HTTP** wire is specified as its own versioned contract in **`docs/fleet/fleet-api.md`** (M1's MQTT one is `control-plane.md`); the operator flow is `docs/fleet/README.md` §6–9 and the measurements are `m3-verification.md`. **The two directions of the loop take different paths on purpose.** *Reads* ride MQTT: the browser subscribes to `mote/v2/+/{presence,health,pose,capabilities,mission/status}` over WebSockets, and because all of those are retained it has the whole fleet's state within a second of loading — no polling, no service in the middle. *Writes* ride HTTP: `POST /v1/robots/<id>/dispatch` authorizes an operator token (`fleetctl operator new --name <you>`; the name is what the audit row records), writes the audit row, then publishes to the same `mission/command` topic. **The topic tree did not change — only who publishes to it**, and `fleetctl dispatch` moved to the API too, so there is one write path rather than one per client. The mission's `input` is validated only by the robot, against the schema its own capability declared: a copy in the server would be a second contract to keep in step, and it would refuse missions a newer robot understands. **The browser cannot publish**: `server/ui/mqtt.mjs` is a hand-rolled subscribe-only MQTT 3.1.1 client that implements no PUBLISH packet, so the split is enforced by omission (M7 makes it structural with a subscribe-only broker credential). The UI is static ES modules — no bundler, no npm, no vendored library — served by the same stdlib `http.server`; `map.mjs` holds the Q5 world→pixel transform (`px = (wx-origin_x)/res`, `py = height - (wy-origin_y)/res`) and a pan/zoom/follow canvas, and only draws robots on the *same* site+floor as the selected one because a pose from another floor is a different map frame. **Basemaps come from site bundles on the fleet box** (`--maps-dir`, default `$MOTE_FLEET_HOME/sites`, the layout `sites.py` writes, seeded by rsync until **M4** makes the registry canonical behind the same two routes). **M1's websockets blocker is settled**: `pixi run fleet-broker` runs `eclipse-mosquitto` under docker with the repo's own `mosquitto.conf`, because conda-forge's build has none; `pixi run -e fleet fleet-broker-local` is the conda binary for a box without docker, and it strips the WS stanza and says so. Two things that run in the same file (`test_ui.py` → `ui_test.mjs`) are the MQTT codec and the transform, tested under node against the very files the browser loads; `browser_check.mjs` drives a real headless Chrome over CDP against a running stack and is an operator's tool, not a CI test — `pixi run fleet-ui-check` is that stack in one command (broker on ephemeral ports, server, a temp `MOTE_FLEET_HOME`, the sim's `office_world` bundle as the basemap, and `test/fake_robots.py`, which publishes `protocol.py` and `spec/` payloads, imports `mote_tasks`' own capability set rather than writing one, and is *not* a second robot implementation), torn down afterwards; `-- --keep` leaves it up for UI work. It stays out of CI because it needs docker (conda's mosquitto still has no websockets) *and* a chrome, which the arm runner has not — the decision, and what wiring it in would take, are recorded in `m3-verification.md` §2 rather than left looking like coverage. **A fourth pane, `review`, is where a candidate map is looked at and promoted** (`server/ui/review.mjs`; routes and rationale under the map registry below). It is a *mode*, not a column: opening it stands the operations panes down at every width, because two canvases — one canonical with robots on it, one a candidate without — is the confusion a dedicated view exists to remove. **The phone is the realistic off-LAN client**, so below 760 px the panes become one at a time behind a bottom tab bar (`server/ui/layout.mjs`), selecting a robot in the roster navigates to the map — what the desktop layout gets for free by showing both — and the canvas gained pinch-to-zoom (`pinchSpan`/`pinchUpdate` in `map.mjs`, pure and tested, because a division by a zero span puts NaN in the view scale and blanks the map for good) plus a fingertip-sized hit target. The breakpoint is a **silent** seam — CSS decides what is displayed, JS decides when a selection navigates, and disagreement yields a tab bar over stacked panes rather than an error — so it lives in `layout.mjs` and `ui_test.mjs` reads the stylesheet and holds it there, as it does for every pane having a tab and for `touch-action: none` on the canvas (without which the browser eats the drag and the pinch before a single pointer event arrives). Dispatch's form is **generated from the robot's own capability set** (retained on the broker): a select of the keys it offers, one field per input property, and a **zone picker** exactly where a property's schema `$ref`s zone/v0's zone reference — so the page holds no list of capabilities and no list of which inputs are places, and a keyboard is needed only where the schema really wants free text. Three pre-existing bugs fell out, all of which a desk hides: `hidden` does not hide an element whose class sets `display` (the empty promote picker), the canvas backing store was resized on width alone so a height change left the previous frame's scale bar under the new one, and the scale bar was drawn in the dark theme's near-white on a white basemap — a canvas gets no cascade, so it now reads `--dim` off the element. Measurements, including `browser_check.mjs`'s phone pass, are `m3-verification.md` §9; **a real device is still the acceptance** — emulation gets the viewport and the touch points right and the thumb wrong.

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
`mote/v2/registry/site/<site>/floor/<floor>/current`, **retained**, which is the
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
local symlink; **the binding travels inside the revision** and replaces the
floor's (the old one is kept as `binding.<old-rev>.yaml`), because a different
session's map makes previously taught coordinates wrong. The **vocabulary does
not** — the names of the rooms did not change when the robot re-mapped the
floor, which is the practical dividend of the zone/v0 split. Three deliberate consequences:
the flip and the announcement are reported separately (a broker that is down must
not half-promote a floor; the server re-announces every floor at startup, which
repairs it), an **upload carries no operator credential** — it names an enrolled
robot, is bounded and audited, and is inert until M7 gives robots a credential —
and a pulled map takes effect on the **next bringup**, since `map_server` reads
its map at startup, so health now carries the revision each robot is actually
running. M3's `/v1/maps` routes kept their shape and changed source; the
dashboard additionally draws the floor's taught zones (circle, polygon or
waypoint cross) from `/v1/maps/<site>/<floor>/zones.json`.
**Promotion is a decision, so the operator has to be able to see what they are
deciding about** — and until the review pane existed the only thing on screen
was a timestamp, the canvas beside the picker being always the *canonical*
basemap. Three GETs answer it, the same three questions `/v1/maps` answers asked
of a revision that is not canonical: `…/revisions/<rev>/{map.json,map.png,
zones.json}`. Three things are load-bearing. `read_map`'s **`image_url` is
revision-aware**, because a transform from the revision and pixels from
`/v1/maps` would draw the published map under the candidate's label — convincing
and wrong, which is the exact failure being removed. The zones read is
**revision-scoped and deliberately not gated on a published map** where
`read_zones` is (the review that matters most is the first candidate on a floor
with nothing published), which does not loosen the vocabulary/binding split:
naming a revision is naming a map frame, and these stay under `/v1/maps`-shaped
paths and never under `/v1/zones`. And it reports **`source: revision|floor`**,
because `_zones_file` falls back to the floor's `zones.yaml` and inherited zones
were taught in a *previous* session's frame — they draw perfectly over the new
map and are wrong by however far the two origins differ, which no coordinate can
say. One UI ordering bug fell out and is fixed in both panes: the floor's
revisions were fetched only *after* its basemap loaded, behind an early return,
so a floor whose only revisions were candidates listed none of them and **the
first promotion on any floor could never be made from a browser**.

## Fleet: editing a candidate's zones

The write half of that pane (`ui/zone_editor.mjs`, route `POST /v1/sites/<site>/
floors/<floor>/zones`, operator flow `docs/fleet/README.md` §11, contract
`fleet-api.md`): drag vertices, poses and whole zones on the map, double-click
an edge or vertex to add or remove one, and name the places. **The whole design
is one rule: editing is a derivation, never a mutation.** Saving re-packs *the revision under review* with the submitted zones
and accepts the result as an ordinary candidate — same `accept()` as a robot's
upload, inert until promoted — so a stored revision's bytes never change, which
is what the announced digests depend on, and promotion stays the only write that
moves a floor. Five things are load-bearing. **The edit names a source
revision**, which is what makes an *unpromoted* map editable: `segment-map`
hands over `zone_01`..`zone_07`, and deriving only from the canonical revision
meant promoting those placeholders in order to be allowed to fix them —
publishing a map because it was wrong — besides rebinding coordinates drawn on
one frame onto another's. **A derivation is held to `promote`'s bar, not the
upload's** (`accept(require_posegraph=…)`): a revision with no posegraph cannot
be extended, which is an error for a robot's upload where the session can be
re-run and a *warning* on something already stored, so the strict bar put an
`edit zones` button beside a `promotable` verdict that could only ever fail
(found in a browser; every sim bundle is such a revision). **It lives in the
review pane and nowhere else** — the operations canvas draws the *published*
basemap, so an editor there could only ever edit the published revision, and
"which map are these coordinates against" must have one answer. That also
retires the MVP's frozen-overlay workaround: after a save the pane selects the
new candidate and re-reads its zones, so what is on screen is the saved set from
the server rather than a held-over copy of what was typed. **`navigable` travels
verbatim**: it used to be dropped when it agreed with the zone's kind, so that a
kind the operator had just changed did not carry the old kind's default with it,
and with the kind retired there is nothing for it to agree with. And the editor
**refuses client-side exactly what the robot's loader refuses**: a name nobody
could have meant, and two zones answering one query (`ambiguities` mirrors
`bundle.ambiguities`), since a stored candidate no robot will load is worse than
a rejected save. Editing is a
*mode*: the floor picker, the revision list and promote are disabled while it is
up, there is no autosave, and `cancel` discards. One pre-existing defect fell
out and is fixed: the map pane's review button was hidden unless the floor on
screen had candidates, and above 760 px the tab bar is hidden too — so the pane
built for floors no robot is reporting was reachable only through a floor a robot
was reporting.

**What the operator can see is the other half of it**, and the first build of
this editor failed it in four ways an operator found in one sitting. **One
`hitTest` answers what a press will take** — the drag reads it, the cursor reads
it, and the hover highlight draws it — because three targets (vertex, pose, zone
body) plus a fourth meaning "this drag pans the map" is unguessable from a
static canvas, and three copies of that ordering would eventually disagree with
each other. Its highlight ring is *ink*, not white: a canvas gets no cascade, and
the surface under it is not the theme's background but the basemap, whose free
space is white in both themes (measured: white moved 1.5% of the pixels around a
handle, ink moves 12%). **A row is a list, not a form** — it carries what is
compared *across* zones (the name, and whether it is a point or an area) and the
rest of the record edits in a panel for the *selected* zone, so a new field costs
no column, and a 2560 px monitor no longer stretches a twelve-character zone name
into a text box the size of a paragraph (rows cap at 640 px). **There is one
list, not two**: one renderer draws a revision's zones read-only and editable
alike, `edit zones` putting controls into cells that were already there and
standing empty — two renderers were two layouts that drifted. Nothing in that column moves when editing opens (asserted in
`browser_check.mjs`), the editing surface has no border of its own, and a zone
is always selected: an empty panel needs a caption, and any caption for it
("select a zone to name it") names one of the several things it is for. And a control exists only where dragging cannot
reach — `⌖` (place a pose) appears **only** for a zone that has none, a
`segment-map` room being an outline with no `x`/`y` and so no cross to drag; a
row's name is a *button*, because in a list the name is what you select by and
an input there put a caret where a click meant "this one" (renaming moved into
the panel, beside the zone's other fields). Two controls were built and then cut
for failing that test: a `+ area`/`− area` cell, and a paragraph of instructions
standing in for the hover feedback before it existed. **A control sits at the
level of the thing it acts on**, which three buttons in one row denied: `save as
candidate` and `cancel` end the mode, so they take the place of `edit zones`
above the list rather than standing beside a control that adds one zone, and
`add zone` is the list's last line. The save's message moved with them, out
of the column of fields for one zone and under the list it is about — with no
height until there is something to say, since a strip reserved for the longest
message (the refusal quoting both zones that answer one query) is a gap over the
first zone that nothing explains. The room comes out of the list instead, the
one thing in that box which scrolls, so nothing above the message moves. It
carries what a save *did*, not that one is happening: the outcome used to be set
on the pane's own line in the far column while this one was cleared, so the only
thing appearing where the work was happening was `saving…`, lasting as long as
the request. The mode's banner is gone with it — `save as candidate` says what
saving does, and the one thing a banner was needed for, the shift modifier, is a
`title` on the surface it applies to.

**A place is named once**, and the name is the human one — `store room`,
`Café`, spaces and accents and all (see "Fleet: zones are place-names"). The
field marks an unusable name *as it is typed*: the rule is the loader's and the
save enforces it, but a field that looks like free text until a save fails does
not look like a field with a rule. What it refuses is now only what nobody could
have meant — a stray space at either end, which looks identical on screen to the
same name without and resolves differently — so `slugify`, `isGeneratedName` and
the display-name field that fed them are gone with the split they existed to
bridge.

A zone is drawn under `map.zoneLabel` — its name — by the operations map and the
editor's own overlay alike, so a place cannot answer to one name in the list and
another while it is being edited. `color-scheme` is declared per theme in the
stylesheet for a related class of reason: a `select`'s dropdown, a checkbox and a
scrollbar are the browser's to paint, and left to the *system* preference while
the page follows its own, a dark page grows a white dropdown list.

**Every coordinate an edit writes lands on a pixel centre** (`snapToPixel`;
shift is the way off it, chosen over alt because a desktop's window manager
takes alt-drag and a modifier the page never receives is no modifier at all).
The map's resolution is the precision available, so a free-hand vertex is digits
nothing can back — and two zones meant to share a wall land millimetres apart,
differently every time. Three consequences: a *body* drag snaps its delta rather
than each vertex (`snapDelta`), so a room traced onto its walls keeps its shape,
and it is measured from the grab rather than accumulated per move, which would
drift the zone behind the pointer by whatever each rounding threw away; the
outline `add zone` invents starts on the grid; and **nothing re-snaps a
coordinate the operator did not touch**, whoever put it there — a pose a robot
measured by driving to it, and one an earlier edit placed, are alike in being
already recorded. Drag that pose and it snaps, because a drag is a fresh
coordinate with the map's precision and no more. A pose **typed** into the
details column is not snapped either, for the same reason shift-drag is not: a
number somebody wrote is already the number they meant, and the grid exists to
stop a *drag* claiming precision the map does not have.

**An edit says who placed it.** zone/v0's `anchor.method` is how a later reader
decides whether to trust a coordinate once the map moves, and the three ways
geometry reaches a floor answer differently: `taught` was measured by driving a
robot there, `derived` was read off a map by `segment-map`, and a click is
neither. The editor stamps `external` (`EDITOR_ANCHOR`, through `reanchored` in
`zone_editor.mjs`) on every zone whose geometry it moves — pose, vertex or body — and leaves an
untouched one carrying whatever it arrived with, so a `segment-map` room stays
`derived` until someone reshapes it. `external` is zone/v0's closest fit rather
than an exact one (the spec glosses it as an off-platform localisation system);
the enum is closed, and the alternatives were recording a click as a
measurement or as an algorithm's output. A successor revision should carry a
method for it (#616). The browser says only the method and `by: zone-editor`;
`bundle_store._stamp_anchor` fills in `at` and which operator held the token,
because a browser's clock is the operator's laptop. The anchor then travels: the
editor packs it into the derived candidate's `zones.yaml`, `bundle.parse_zones`
carries it through the reader instead of dropping it, and `zone.split` honours
it instead of stamping `taught` over the top.

**Geometry is a property, not a type**, so the row says `point <x>, <y>` or
`area · <n> corners` and there is no kind to declare. `add zone` makes an area
(a square at the view centre, to drag onto the walls) and `⌖` gives an outline a
pose; converting an existing zone between the two — what `withKind` and
`POINT_KINDS` did while a kind decided it — has no control in this revision, and
is the one thing the pane lost with the taxonomy.

## Fleet: adopting spec v0 (capability set, typed failure)

Mote is the reference implementation for the Augere specifications
(`mission/v0`, `capability/v0`, `zone/v0`), and the load-bearing parts of the
mission spec were extracted *from* M1 — dispatcher-chosen correlation ids, dedup
on redelivery, one in-flight mission, retained status, never-retained command,
the 20 s no-verdict timeout. **What the specs add is typing**, and adopting it
is what `mote/v2` is. The contracts live in **`mote_bringup/spec/`**
(`mission.py`, `capability.py`) for the reason `bundle.py` lives there: the task
layer executes missions, the agent bridges them and the off-board server
dispatches them, and none of those three may depend on the others. Stdlib-only
as well as ROS-free, because the fleet box's container installs no framework.

**A capability set replaces the command grammar.** `mote_tasks/capabilities.py`
declares `goto` and `fetch` — both *standard registry* keys, with the registry's
own property names (`fetch` delivers to `destination`, not the old grammar's
third word) — and `task_server` publishes it on a latched ROS topic that the
agent forwards retained. Forwarded, never authored: a robot whose task server is
down advertises nothing, which is true. A location-taking input `$ref`s zone/v0's
zone reference, which is what lets a tool tell *mechanically* which inputs are
places; the dashboard's dispatch form is generated from exactly that, so it
holds no list of capabilities and no list of which inputs are zones. Both declare
`cancellable: false`, because the task layer has no cancel and saying `true`
would promise a message nothing handles — `mission/cancel` is a reserved leaf
with no publisher.

**A typed failure replaces the detail string.** `rejected: busy with '…'` is now
`failure.class: "busy"` with `recoverable: true` and the in-flight mission's id
in `detail`. Three rules are enforced in `spec/mission.py` rather than left to
callers, because each fails *silently* in a caller: `terminal` is computed from
`state` rather than supplied; `failure` is required on `rejected`/`failed` and
refused everywhere else; and `recoverable` **must be stated** for `precondition`,
`unresolved_zone` and `timeout` — the spec's "depends" rows, where the answer is
a fact about the instance and a class-level default would be a guess wearing a
contract's clothes. Which class a *tree* failure gets is decided by the
behaviour that failed (`trees/common.py`'s `report_failure`): Nav2 refusing a
goal is `unreachable`, Nav2 aborting after its own recoveries is `obstructed`
and retryable, no action server at all is `internal`. A tree that fails saying
nothing is `internal`, which is the honest reading — not one of the world-shaped
classes.

**The input validator is a bounded subset of JSON Schema** and the property that
makes that acceptable is that it **raises on a keyword it does not implement**
rather than ignoring it: an ignored `maxItems` is a promise nothing keeps,
quietly, forever. `check_schema` runs when a capability is *declared*, so that
failure lands on the platform that wrote it. A general engine was not worth a
dependency on the robot image and the fleet container for two schemas.

**The lane moved to the executor.** It was the agent's only because
`task/command` had no correlation id; now that a mission has one, the rule
belongs to the thing that holds the lane — which also sees missions issued
locally, where the agent could only infer them. `dispatch.py` kept dedup,
retention (an hour, so a restarted dispatcher still learns the outcome), the
no-verdict timeout, and `source`. **Preconditions are now evaluated rather than
written in a docstring**: `localized` wants a `map`→`base_link` transform newer
than 5 s and `zone_known` wants the zone to resolve, so a `goto` sent to an
unlocalised robot is refused with a reason instead of flailing in Nav2 — and an
unmet *non-blocking* one lands in `warnings` on the accepted status rather than
refusing. `max_duration_s` is enforced too, since the spec makes an unbounded
uncancellable capability undispatchable.

The ROS seam is still `std_msgs/String`; the string is JSON. A custom message
would have bought type-checking inside the graph and cost the property that
matters more — that the agent forwards these payloads **byte for byte**, so
there is one definition of the wire and the bridge cannot reinterpret it. It
also cost the bench flow, so `ros2 run mote_tasks mission goto target=kitchen`
(`mote_tasks/mission.py`) is what replaced typing a sentence into `ros2 topic
pub`; `--list` prints what the robot will accept. **Mote vendors no copy of the
specs' schemas** — a copy drifts — so `mote_bringup/test/test_spec_conformance.py`
validates real payloads against the specification's own where a checkout is
present (`$AUGEREAI_SPEC`, or a sibling `augereai-spec/`) and skips where it is
not. **A v1 robot and a v2 server do not interoperate**, deliberately: a
translating shim would be a third definition of the wire.

## Fleet: the zone vocabulary/binding split (zone/v0)

`zones.yaml` held names and coordinates in one file. zone/v0 pulls them apart,
and the split is what the whole spec is for: **names are shared, coordinates
are not, maps are never shared.** A floor is now two documents —
`floors/<floor>/vocabulary.yaml` (site, floor, and what the places are
*called*: `name`, `note`, `navigable` — see "Fleet: zones are place-names") and
`floors/<floor>/binding.yaml` (this robot's poses, footprints and `anchor`,
stamped with `platform_id`, `frame_id` and `map_revision`). Both are built by
**`mote_bringup/spec/zone.py`**, which also holds the containment geometry, so
the robot and the fleet server give the same answer on a boundary case; the
vocabulary rules moved there out of `bundle.py`, which re-exports them.

**The split is structural, not a rule to remember.** The vocabulary is *built*
from the fields a vocabulary may carry, never stripped of the ones it may not —
stripping holds only until someone adds a geometry key and forgets, and the
leak would be a plausible-looking coordinate rather than a crash. Tests assert
it by walking the whole document for geometry-shaped keys.

**What it buys is a distinction the robot could not draw.** A name in the
vocabulary that the binding this robot holds carries no geometry for now
resolves `unbound` — "I know that place, nothing has said where it is" — where
before it was `unknown_name`, which sent an operator hunting for a typo that
was not there. `unbound` is a fact about the revision, not about where the
robot has been: the promoted revision may bind it for nobody, this robot may be
running an older one, or it may be a name nothing has ever placed. So the
refusal names all three remedies — place it in the dashboard's zone editor and
promote, pull the revision that binds it, or drive there and `save-zone` — and
not just the last.
`task_server` loads with `zones.load_floor` (vocabulary ∪ binding) rather than
`load_zones` (bound only) for exactly that. A binding whose name the vocabulary
does not carry is a **local extension**: usable here, never advertised, because
one robot must not invent shared vocabulary for its neighbours.

**Migration is a side effect of writing, never a step.** `bundle.read_floor`
accepts a floor directory *or* a legacy combined file, and reads the latter
through `zone.split` so both paths produce the same structure by construction;
the first `save-zone` or `segment-map --write` on an old floor writes the pair
and keeps the original as `zones.yaml.premigration`. The sim worlds' committed
`<world>.zones.yaml` files stay combined on purpose — one file is the right
shape for a fixture with one robot in it — and are migrated on read.

**Which half travels where.** A map revision carries the **binding**, because a
coordinate means nothing without the frame beside it; installing a pulled
revision replaces the floor's binding and leaves the vocabulary alone, so
re-mapping a floor no longer costs an operator the names they typed. The
**vocabulary** is floor-level and is what `/v1/zones` serves — and, from the
dashboard's zone editor, a candidate carries *both* halves and promotion is
what lifts its vocabulary to floor level: uploading is not publishing, applied
to names as well as to coordinates. A polygon-only zone gets its binding pose
**derived once, on write** (`zone.representative_point`), because zone/v0
requires a binding to carry a pose and it is right to — a footprint alone
cannot say where a mission navigates to.

Still unanswerable here: `wrong_floor` (the robot holds one floor at a time)
and `stale_revision` (the bundle declares no frame continuity — which zone/v0
says is out of its own scope too). Broadcasting the vocabulary over the retained
registry subtree, so a second robot at a site learns the names before it has
driven a metre, is M6's and is the thing this split was the prerequisite for.

## Fleet: the zone vocabulary

The API served the roster, the basemaps and dispatch, but not the one thing a
dispatcher most needs — *what places can I name?* — so an MCP front door had to
work around it out of band or by scraping the list a robot prints when it
refuses an unknown zone, which is an accident of an error message rather than a
contract. `GET /v1/zones` and `/v1/zones/<site>/<floor>` answer it from the site
bundles the server already reads for maps, in the shape of **zone/v0**
(`docs/fleet/fleet-api.md`, operator flow `README.md` §12; the spec's own
`spec/zone/v0/README.md`). **The whole design is one rule: names travel,
coordinates do not.** A zone's pose is a coordinate in one robot's map frame,
whose origin is an accident of where its SLAM session started, so `(2.0, 3.5)`
is a different physical point on the robot beside it and no fleet-level
transform fixes that; the *name* is true for both. So the **vocabulary** —
the `name` of each place, a free-text `note`, and `navigable` — is published and
the **binding** is not. The split is expressed by the route rather
than by a rule someone has to remember: everything under `/v1/maps` is bound to
a basemap and served to the client that already has one, everything under
`/v1/zones` is bound to nothing. Four things are load-bearing. The payload is
**built** from the fields a vocabulary may carry, never *stripped* of the ones
it may not — stripping holds only until someone adds a geometry key to
`zones.yaml` and never reads this code, and the leak would be a
plausible-looking coordinate rather than a crash (`test_zone_vocabulary.py`
walks the whole payload for geometry-shaped keys rather than checking the ones
it thought of). The vocabulary is deliberately **not gated on a published map**
where the binding rightly is: names are a fact about the building, so a floor
someone has named but no robot has mapped still answers, which is the
portability the split buys. `problems` is **reported, not enforced** — two places
called the same thing, or a name with a stray space at one end, leave the map
perfectly good, and refusing to serve a floor's basemap over it would be the
wrong price; the name is served verbatim, because inventing one is a rename
nobody asked for. But the *robot* refuses an ambiguous vocabulary at
`load_zones`, since it could not honour `goto` unambiguously. Contradictions
with no reading at all — a legacy `keepout` marked `navigable: true`, a
`navigable` that is neither true nor false — are refused at the parse by the
shared `mote_bringup/bundle.py`, so `save-map` catches them locally and the
server catches them on upload. `save-zone` teaches the name and `--note`;
`segment-map` emits geometry and nothing else, because an enclosure with walls
round it is all it found. Related: mote #249 (the rest of spec v0 — capability
set and typed failures).

## Fleet: zones are place-names

A zone's vocabulary was seven fields: `kind` from a fifteen-value taxonomy,
`display_name` beside a machine `name`, `aliases`, `parent`, `tags`,
`description`, `navigable`. It is now **one human name and a `note`**, and the
principle is that a zone is a place-name: a name bound to geometry, carrying
only what a prior cannot guess. The mission layer's LLM resolver already knows
what a store room is; what it cannot know is that *this* building's store room is
where the stationery lives — which is exactly what the note says, and the whole
of what the record adds. `navigable` stays, unchanged, because it is not
vocabulary at all: it is the planner's contract, and it travels with the names
only because it is not a coordinate.

**The name is the human one.** `store room`, `Café`, `Ward 3B` — printable text
with no leading or trailing space (`zone.ZONE_NAME_RE`), matched exactly and
then case-insensitively and whitespace-normalised. A machine name beside a
display name was two fields for one fact, and it took `slugify`, a
proposal-on-typing rule and a third naming field to keep them in step; the
resolver reads the human spelling either way. Other names a place answers to go
in the note, since free text is what an LLM resolver reads and a hand-maintained
alias list was the part nobody maintained. Two consequences: `resolve` no longer
matches aliases or display names (`goto galley` stops reaching `kitchen`), and
an ambiguity can now only be two places called the same thing.

**Retirement is on the write, not the read.** `zone.term` still accepts every
retired field, so a floor taught before this loads without being re-taught, and
`LEGACY_KEYS` names them. Two are read for meaning rather than merely tolerated:
`description` is `note`'s former spelling and is read into it, and `kind:
keepout`/`slow` still seeds `navigable: false` — dropping *that* would have
turned every barrier on every already-taught floor into a destination, silently,
on the first load after the upgrade, and writing `navigable` back is the
migration. An unknown `kind` is now ignored where it used to be refused: there is
no list to be outside of, and refusing a floor over a field nothing reads would
be the wrong price. Nothing retired is ever written or served — `VOCABULARY_KEYS`
is `("note", "navigable")` and the payload is *built* from it, so this cannot
regress by someone forgetting to strip a field.

**Writers stopped writing it.** `save-zone` takes `--note TEXT` and
`--no-navigable` where it took `--kind K` (the flag named a taxonomy in order to
set one flag); `segment-map` emits geometry and no vocabulary at all, because an
enclosure with walls round it is all it found; the dashboard's editor has no kind
select. The sim worlds' committed `<world>.zones.yaml` and
`mote_tasks/config/zones.default.yaml` were re-emitted without `kind`, and
`gen_hospital.py` with them.

Files: `mote_bringup/spec/zone.py` (the record and its rules), `bundle.py`
(re-exports; `ZONE_KINDS`/`POINT_KINDS` are gone, `CONSTRAINT_KINDS` survives as
the legacy `navigable` seed), `mote_tasks/zones.py` (`Zone`, `resolve`,
`append_zone`), `mote_fleet/server/ui/` (the review pane, per the "Zone
Gazetteer" design), `docs/fleet/fleet-api.md` §the zone vocabulary. **The
specification's own `spec/zone/v0/README.md` is not in this repo** and still
describes the seven-field vocabulary; a successor revision there is outstanding.

## Fleet: the server pipelines (Ms)

Milestone Ms of `docs/design/fleet.md`: how the two **non-robot** machines are built and updated, runbook in `docs/fleet/server-pipelines.md`, measurements in `docs/fleet/ms-verification.md`. Both are container deploys **driven by their operator, not by the fleet server** — a robot is fleet-managed, a server is infrastructure — and the pipelines differ in exactly one thing, state. **The fleet server** (`mote_fleet/deploy/`: `Dockerfile` + `docker-compose.yml` + `fleet-deploy.sh`, image `ghcr.io/clachdev/mote-fleet` built by `.github/workflows/fleet-image.yml`) is two containers — mosquitto mounting **`server/mosquitto.conf`, the same file `fleet-broker` uses**, so deployed and workstation brokers cannot drift, with the **image tag pinned once** in the compose file and read back by `broker.sh` so they cannot drift on the binary either (a *minor* series, `2.1-alpine`, not the floating `:2`: 2.0 links libwebsockets and 2.1 implements websockets natively, so `:2` moving again could take the dashboard's read path with it, and the broker healthcheck probes 9001 as well as 1883 because mosquitto stays up and healthy-looking when only the websockets listener fails to open), and a python image carrying the API, the registry and the M3 dashboard — plus two named volumes holding the only state that matters: `/var/lib/mote-fleet` (registry.db + the `sites/` bundles the dashboard's basemaps come from) and the broker's retained messages. `.env` is the declared state; `BROKER_HOST` is the one value that must be right (handed to robots verbatim at enrollment, so the compose file refuses to start without it) and `BROKER_WS_PORT` is published *and* passed as `--broker-ws-port`, because it is the port the browser is told to reach the broker on. It holds state, so its update is a **gated recreate**, not blue/green: `fleet-deploy.sh update` tags the running image `:previous` before pulling, recreates, health-gates on `/healthz` over the *published* port, and puts the old image back automatically if that fails; `backup`/`restore` snapshot both volumes (registry via sqlite3's online backup API, not `cp`). **The inference server** (`mote_perception/deploy/inference-deploy.sh`, one file curled onto the host) is stateless, so it *is* blue/green: the candidate runs on shadow ports 5611/5612 while the current one keeps serving, and must pass `mote_perception/tools/probe.py` — health **and a real synthetic frame**, because a health sentinel is answered before the model has ever loaded and cannot see a broken weight download or a CUDA mismatch. The **flip is a stop-then-start**, deliberately: pushing a new port out to robots (as the design sketch had it) means editing `perception.yaml` on every robot, a worse outage than the seconds this costs, which the robot's warn-and-skip fallback makes a non-event. Every check runs *inside the image being deployed*, so the GPU box still installs nothing. `mote_perception/deploy/test/drill.sh` (`pixi run deploy-test`) exercises that whole pipeline with stub images on any machine with docker — no GPU — and `mote_fleet/test/test_fleet_outage.py` is the other half of the milestone's acceptance: kill the broker under a live agent and the robot still finishes its task, then the agent reconnects by itself.

## Sites (maps & zones)

Everything that is only meaningful relative to one mapped place — the Nav2 map pair, the slam_toolbox posegraph, and named zones — lives together as a **site bundle** under `~/.mote/sites/<site>/floors/<floor>/`, managed by `mote_bringup/sites.py` (CLI: `pixi run site`, docs in the module docstring). A floor is one SLAM session (one map frame); a site groups floors sharing a location. `~/.mote/active.yaml` selects the active site/floor per robot; launch files resolve the map (`nav2_launch.py`, `robot_launch.py`) and zones (`tasks_launch.py`) from it at launch time (zones fall back to the committed default). `MOTE_HOME` overrides `~/.mote` for tests/experiments. What a revision must *contain* — and how it validates, packs and travels — is `mote_bringup/bundle.py` (ROS-free, shared with the fleet server; see the map registry section above). Map artifacts are immutable **revisions** under `floors/<floor>/maps/<rev>/`, published by atomically flipping the `floors/<floor>/map` symlink once the revision is complete — a half-written save or interrupted transfer is never visible, and `site use-map <rev>` rolls back. `save-map` stores the posegraph alongside the map so mapping can be *continued* in the same frame later (extend, don't remap — remapping breaks zone coordinates). Mapping runs also record the `mapping` rosbag stream by default (`mapping_launch.py record:=true`; the sim passes false), and `save-map` stamps the session's bag into the revision's `meta.yaml` for provenance (`site info` shows it). Zones get their geometry three ways, and only the first needs a robot: `pixi run save-zone <name> [--note TEXT]` captures the pose the robot is standing at (the one way that also measures an approach heading), `segment-map` reads room outlines off a saved map, and the fleet dashboard's zone editor places and drags them on a candidate revision. A zone is a named pose (a fetch waypoint or a `goto <zone>` target) that may optionally carry an area **footprint** — a taught `--radius` circle, or a `polygon` outline that follows the actual room walls — so it reads as a room and answers "am I in it"; `site info` shows the zone/footprint counts and how many names the binding carries no geometry for. A floor's zones are **two files, not one** (zone/v0): `vocabulary.yaml` holds what the places are called and `binding.yaml` holds where geometry says they are — stored apart because only the names are portable off the map frame they were measured in; the binding travels inside the revision that names that frame. A legacy combined `zones.yaml` is still read and is migrated the first time anything writes. See "Fleet: the zone vocabulary/binding split" and "Fleet: zones are place-names". Maps are saved as PNG (map_server reads it natively; browsers can render it directly). `save-map` automatically runs an FFT structure-extraction **cleaning pass** (`mote_bringup/map_cleanup`, `sites._promote_cleaned`): it keeps the untouched map_saver output as `map_raw.png` and promotes the decluttered image to the served `map.png` (plus a `diagnostics.png`), so navigation always consumes the cleaned map while the raw is retained for provenance/audit. The `map.yaml` frame is identical for both, so zones/localization are unaffected; a cleaning failure falls back to serving the raw. The posegraph belongs to the raw map — mapping continuation extends from raw, never the cleaned image. **Zones no longer have to be taught one at a time**: `pixi run segment-map` (`map_cleanup/room_segmentation.py`, the ROSE² second stage the declutter pass left open) carves a saved map's free space into rooms and proposes one polygon zone per room, `--write` merging them into the floor's zones for the operator to rename — additive over zones already bound (a candidate covering an already-footprinted zone is dropped as named, so re-running is a byte-identical no-op) and written at floor level, never into the immutable map revision. A proposed room is anchored `derived`, not `taught`: it was read off a map by an algorithm, which is what tells an operator later that a re-map invalidates it. The method is one physical assumption — a doorway is narrow — applied to a grid the wall lines cut into faces: faces merge wherever their shared boundary has a clear span wider than a door, so it is indifferent to room size where a distance-transform threshold is not. Two consequences: a **corridor network is not proposed at all** (a footprint is a single outline, so a region encircling a block of rooms would claim them; those are dropped, taking with them any room wrongly absorbed into the corridor), and the geometry is **Manhattan after rotation** — an arbitrarily rotated map frame is fine, a building with wings at 30° to each other is not. Scored against ground-truth room rectangles on the sim ladder by `pixi run segment-eval` (30/33 mapped hospital rooms, 10/10 office, 1/1 mote, **zero merges**, unchanged with the map turned 17° or -31°); results in `docs/tuning/2026-07-27-room-segmentation.md`.

## Drive path (who gets the wheels)

`DiffDriveController` has exactly one publisher: **`twist_mux`**, started with the base by `twist_mux_launch.py` (included by `mote_launch.py` and, so the sim base matches, by `sim_launch.py`). Nav2's `controller_server`/`behavior_server` publish `/cmd_vel_nav` (priority 10), everything a human drives with publishes `/cmd_vel_teleop_stamped` (priority 100) — `twist_relay` for the Foxglove panel, `pixi run teleop`, the RViz teleop panel in `mote.rviz`, and `pixi run explore` (it stands in for a human driver) — and the mux forwards one of them to `/diff_drive_controller/cmd_vel`, whose name is deliberately unchanged so bags, the benchmark and the sim smoke test still watch the command the wheels got. Table in `config/twist_mux.yaml`; rationale and measurements in `mote_bringup/README.md` "Drive path". It is **adopted, not built** (`ros-jazzy-twist-mux` 4.5.0, which supports `TwistStamped` via a `use_stamped` that defaults true — the parameter is not declared by the node, so setting it in the launch would be a no-op that read like a setting): a first-party mux would have been a Python node in a 20 Hz path, the thing #73 just removed from a 50 Hz one. Three decisions are load-bearing. **Teleop overrides Nav2, it does not cancel it** — cancelling from the drive path would wire velocity arbitration into the action layer, and a nudge to straighten the robot in a doorway would destroy a fetch mission. **The teleop input's timeout (1.0 s) is deliberately longer than the controller's `cmd_vel_timeout` (0.5 s)**, so after the operator's last command the wheels halt before Nav2 regains the topic: a takeover always ends with a stopped robot, never a handback mid-motion (measured 1.00–1.05 s of silence; pre-emption itself ~50 ms). Invert those two numbers and the property vanishes silently, so `test_twist_mux.py` holds the files together and `test_twist_mux_arbitration.py` measures a real mux. **The deadman is unchanged**: twist_mux publishes only from an input callback, with no timer and no stored last command, so every source stopping still means the drive topic stopping — asserted, not assumed, because a mux that re-published would turn "the link dropped" into "the robot keeps going". To hold autonomy off entirely there is a twist_mux **lock** on `/pause_navigation` (`std_msgs/Bool`, priority 50 — masks navigation, not teleop; a Publish panel in the shipped layout sends it), with one consequence worth knowing: a goal held off the wheels while the robot stands still fails Nav2's own `SimpleProgressChecker` after ~10 s, so a long pause ends the task. Cost is one process and **one DDS participant**, putting the robot stack at ~26 of 33. There is still **no `cancel` command in the task layer** — `task_server` accepts only `fetch`/`goto` — so "cancel the task first" was never actually possible; the pause lock is what an operator has.

## Architecture

Mote is a differential-drive robot built on **ROS 2 Jazzy**, managed entirely through pixi (no system ROS install required). First-party packages:

### `mote_hardware` (C++)
A `ros2_control` `SystemInterface` plugin (`MoteHardware`) that drives two Feetech STS3215 servos via the SCServo SDK over a serial bus. Key implementation details:
- Servo IDs and all hardware params come from `robot.yaml` via the URDF's `<ros2_control>` tag, read by `MoteHardware::on_init` from `info_.hardware_parameters`
- Position is tracked cumulatively across the 12-bit encoder rollover using a half-range threshold
- The left wheel is mounted inverted, so its sign is negated in both `read()` and `write()`
- The serial port is opened in `on_activate` (not `on_init`), which also puts servos into wheel (continuous rotation) mode — an EEPROM write, skipped if already set
- It is also **the single owner of the shared servo bus**: the SO-101 arm's six servos sit on the same port as the wheels, so `MoteHardware` exports position command/state interfaces for them too (`arm_joint.hpp` holds the clamp and the encoder<->radian maths, mirroring `mote_arm/config.py` (`zero_counts`, deliberately not "home" — see the arm section)). Arm state is read one joint per cycle round-robin and arm goals go out as one sync-write only when a goal changed, so the arm costs ~1 extra bus transaction per cycle and nothing at all when idle — the wheels are on this bus and the loop runs at 50 Hz. `port_guard.cpp` refuses activation when another process already holds the port
- Tools built from `mote_hardware/tools/` (`servo_debug`, `velocity_cal`, `swap_ids`, `setup_ids`) run as `pixi run -- ros2 run mote_hardware <tool>`; see `mote_hardware/tools/README.md`

### `mote_description` (CMake)
Contains `urdf/mote.urdf.xacro` and `config/robot.yaml`. The xacro loads robot.yaml at processing time and uses those values directly — no xacro args are needed or accepted. The `<ros2_control>` tag embeds the servo params so they reach `MoteHardware::on_init`.

### `mote_nav` (C++)
The C++ that runs *inside* other people's processes: a Nav2 plugin and two composable nodes, all too small to deserve a process each. The two hardware numbers they share — `max_wheel_speed` and `wheel_separation` — reach both through `include/mote_nav/wheel_speed.hpp` (`maxWheelSpeed`, `maxYawRate`), which is dependency-free so the odometry gate does not drag `dwb_core` in nor the critic `tf2_ros`.
- `WheelSpeedLimitCritic` (`src/wheel_speed_limit_critic.cpp`, exported to `dwb_core`) — DWB samples a v x w rectangle with no notion of the differential-drive coupling between the two, so it marks any sample needing a wheel faster than `max_wheel_speed` illegal. `wheel_separation`/`max_wheel_speed` are overlaid onto `controller_server` from `robot.yaml` by `nav2_launch.py`, so the hardware envelope has one source of truth.
- `mote_nav::OdomTfRelay` (`src/odom_tf_relay.cpp`, an `rclcpp_components` component; also installed as a standalone `odom_tf_relay` executable) — republishes the wheel pose as the inverted TF leaf kinematic_icp reads as its motion prior. Loaded into `localization_launch.py`'s container alongside its one consumer. It replaced a Python node of the same name: at the controller's 50 Hz update rate the interpreter wake-up, the GIL and the process hop each cost more than the twenty floating-point operations they existed to perform. The arithmetic is a deliberate transcription of the Python it replaces and the target is built `-ffp-contract=off`, so the output is bit-identical rather than merely close — which is how the two were compared.
- `mote_nav::IcpOdomGate` (`src/icp_odom_gate.cpp`, a component; also a standalone `icp_odom_gate` executable) — **owns `odom`→`base`**, accumulating kinematic_icp's increments and substituting the wheel increment for any the drive could not have produced. Real mapping bags catch the scan match emitting, in one scan, up to 1.2 m/s against a measured 0.218 m/s limit — once 0.12 m while the wheels reported the robot **stationary**. Slip cannot cause it in that direction (slip makes the *wheels* over-read), and the frames are **steps, not spikes**: the ICP-vs-wheel gap rate is identical either side, so the displacement is never given back and each one is permanent error in the map frame and in every zone taught in it. Since a TF broadcast cannot be retracted, kinematic_icp is left publishing only its odometry topic in a frame of its own (`odom_icp`) and the gate broadcasts the edge instead; kinematic_icp is unaffected, because its prior comes from the `odom_wheel` TF leaf and never from its own output. The bound is `robot.yaml`'s two numbers × 1.15, through the shared `wheel_speed.hpp`. Two things the data decided rather than taste: the **joint `|v| + S/2·|w|` bound the critic uses is unusable here** — the wheel odometry itself exceeds it in 19% of intervals and the legitimate and excursion populations overlap completely, the yaw term being inflated by 10 Hz resampling — so translation and yaw are bounded separately; and the substitute is the **wheel increment rather than a clamp**, because a stationary robot's wheels say 0 where a clamp still admits 0.025 m. Evidence, thresholds and before/after are `docs/tuning/2026-07-28-icp-velocity-gate.md`; `mote_bringup/tools/icp_excursions.py` characterises a bag's excursions (spike vs step) and `icp_gate_replay.py` replays a bag through the *compiled* gate so `odom_health.py` scores real output rather than an offline model of it.

### `mote_health` (C++)
The robot-level health monitor, and the only first-party C++ that runs **as a process of its own** — which is why it is not in `mote_nav`, whose charter is the C++ that runs inside other people's processes, and not in `mote_bringup`, which is `ament_python` and cannot build C++. It publishes the `mote` roll-up plus one `DiagnosticStatus` per subsystem on `/diagnostics_agg`, and one line on `/health`; what it watches is `config/health.yaml` (moved here from `mote_bringup`, still overridable at `$MOTE_HOME/health.yaml`). It stays its own process because `mote-health.service` is `Type=notify` with a watchdog, so composing it into a container would bind that watchdog to something else's liveness.

**It is C++ because of what a monitor costs, which is its wake-ups.** Measured on mote-01, an rclpy wake-up is ~0.78 ms of CPU per message and this node consumes ~152 msg/s, so ~12 points of a core go before it does anything with them — against a 5-point budget (`docs/tuning/2026-08-11-monitor-cpu.md`). **The lever is therefore every subscription, `/tf` included**: a `TransformListener` takes the whole `/tf` stream whatever handful of edges it asks about, 4.8 of the Python node's 16.9 points, so a port keeping the TF watches in Python would keep a third of the cost. Measured paired on mote-01, both builds at the same instant against one synthetic input stream at the robot's real rates and payloads: **10.6 → 1.1% of a core, and 66.9 → 26.2 MB resident** (`docs/tuning/2026-09-01-health-monitor-cpp.md`). Two things that measurement settled and are worth not re-deriving: **neither contention nor payload size is the variable** (load1 3.0 against 0.1 moves the Python build 0.8 points; a 40 KB JPEG against a 4 KB one moves it 0.1), and the bench is a *lighter* operating point than a robot in mission — the Python build reads 10.6 here against 17.1 on the live stack, a gap that is neither load nor payload and is presumably the real DDS graph and TF tree, so these figures are floors.

Three things are load-bearing. **The config-driven design survived the port**: `create_generic_subscription(topic, type_string, qos, cb)` takes the message type as a runtime string exactly as `get_message(spec["type"])` did, so `health.yaml`'s topic list needed no change and there is no codegen per type — and the callback is handed a serialized message it never opens, which is what `raw=True` bought in Python. **The roll-up is where the behaviour lives, and it is free of rclcpp, tf2 and yaml-cpp** (`src/health_rollup.cpp`): the node contributes only arrivals and transform lookups, so severity mapping, slow-vs-stale and the summary are gtests rather than a running graph — the Python monitor's own test cases, ported one for one. **Equivalence was measured, not asserted**: no bit-identity was available as it was for `OdomTfRelay`, so `test/compare_monitors.py` runs two monitor commands *at the same instant* against one synthetic input stream through healthy / stale / degraded / recovered and diffs every status name, order, level, message, value and cadence — the two measured values, `rate_hz` and `age_s`, being the only things allowed to differ, and then only within a tolerance. It found nothing, down to the tf2 exception string and the self-check timestamp read through yaml-cpp against PyYAML. Two pieces had to come across with it: `sd_notify` **moved** (`mote-health.service` is the only `Type=notify` unit, so no Python copy was left behind), and `mote_home`'s override rule is **duplicated** in `src/mote_home.cpp` because Python still needs it — the only such duplication, pinned by `test_mote_home.cpp`.

### `mote_bringup` (Python/ament)
Launch files, config, udev rules, the `wifi/` roaming configuration (below), systemd services, and the fleet foundation: `mote_home.py` (per-robot state root), `identity.py` (`identity` console script), `provision.py` + `provisioning/user-data.template` (`provision`), `dds_participants.py` (`dds_participants`), `twist_relay.py` (`twist_relay`, the Foxglove teleop seam), `explore.py` (`explore`, `pixi run explore` — autonomous mapping coverage: left-wall following with a Nav2 frontier-seek escape and a stuck detector for lidar-invisible obstacles like rug edges; run beside `pixi run mapping` **on the Pi**, so a wifi drop cannot end the mission; `--sim-time` gates it on /clock for the sim, and `--cruise`/`--obstacle`/`--desired-left`/`--follow-band` tighten the corridor-scale defaults for domestic layouts), the `foxglove/` layout, and `tailscale/install.sh`, plus `bundle.py` — the site bundle's *content*: a ROS-free reader/validator/packer for a map revision, shared with the fleet server's registry (M4) so both ends check a revision with the same code — and `spec/` (`mission.py`, `capability.py`), Mote's implementation of the open specifications' payloads, here for the same reason and additionally stdlib-only. See Fleet above.

**Stray ROS processes** are `sweep_orphans.py`'s subject, from both ends (`mote_bringup/README.md` "Clearing stray ROS processes"). *Cure* is `pixi run sweep`: agent worktrees leave nodes behind that reparent to init and outlive the job by days — 44 of them, 371 MB, were found on the dev box — and they are the exact process names a benchmark measures, against system-wide counters no `overhead.py`-style scoping can isolate. **Matching is on identity in /proc, never on the command line**, which is not fastidiousness: `pixi run kill`'s old `pkill -9 -f '<driver names>'` matched the shell running the task (the names are in its own command line), SIGKILLed it, and so **never reached the `ros2 daemon` reset that followed it** — reproduced, exit 137 — besides matching every other checkout on the box. Both modes now require a ROS environment, a path under the directory in question, and absence from the sweeper's own ancestry; `kill` keeps that scope (`--scope .`) and drops the orphan/age tests, since a wedged stack is live by definition. The sweep adds them, because a `setsid`-detached run — the sim smoke test's — is indistinguishable from a leak by ancestry alone, so `--min-age` (30 min) is what separates them and reporting is the default. *Prevention* is `spawn_reapable`/`reap_group` in the same module: **`ros2 run` is a wrapper that `Popen`s the real executable and handles no SIGTERM** (only `KeyboardInterrupt`, assuming a terminal Ctrl-C reached the whole group), so `proc.terminate()` on it kills the wrapper and hands the node to init — once per run, *on the success path*. Measured on `test_twist_mux_arbitration.py`: 6 tests pass, 1 `twist_mux` survives; 0 after. Every `ros2 run` spawn goes through it (`test_twist_mux_arbitration.py`, `test_foxglove_teleop.py`, `tools/icp_gate_replay.py`; `bench.py`/`replay.py`/`camera_layer_decay.py` already did it by hand). The other half — a job killed outright, taking pytest with it before any teardown — no fixture can fix, and is what the sweep exists for.

**Wifi roaming** is `mote_bringup/wifi/` (its README carries the source paths and the measurements). The flat has six APs on one SSID because signal is poor throughout, and the robot would associate once and never move — measured at -80 dBm on a 5 GHz AP with a same-SSID 2.4 GHz one at -55 dBm in view, dropping off the network entirely when carried between rooms. **The cause was not a sticky firmware**: Raspberry Pi OS ships `roamoff=1` in `/usr/lib/modprobe.d/rpi-brcmfmac.conf`, so the firmware's roaming engine had been off since the Pi was imaged and every observation of "sticky firmware roaming" here was an observation of no roaming at all. **What looks like the opening — take the decision in userspace, where thresholds are configurable — is closed on this card**, and that was established by shipping it and walking the robot: a 2 min 19 s walk on 2026-09-01 under iwd logged one BSSID throughout, 70 s at or below -85 dBm, `visible_same_ssid = 1` on all 133 rows, and a kernel scan cache last written at the boot six days earlier. iwd (3.8) declines twice in `src/netdev.c` and `src/station.c`: `netdev_cqm_rssi_update()` returns before sending `CMD_SET_CQM` for a `CONNECTION_TYPE_FULLMAC` connection ("Fullmac cards handle roaming in firmware"), so no RSSI threshold is ever armed and `station_low_rssi()` is never called — and the connection *is* fullmac, because a WPA2-PSK one is softmac only with the `authenticate`/`associate` commands and offloaded only with `4WAY_HANDSHAKE_STA_PSK`, and this wiphy advertises `connect`/`disconnect` and an extended-feature set of exactly `CQM_RSSI_LIST` + `DFS_OFFLOAD`; the RSSI *polling* fallback then returns immediately because `CQM_RSSI_LIST` is advertised. Turn firmware roaming on and `station_cannot_roam()` stands iwd aside deliberately, naming brcmfmac in its comment. `RoamThreshold`/`RoamThreshold5G` are read and reach no decision either way. wpa_supplicant is no better: it scans while associated only under a per-network `bgscan=`, **NetworkManager exposes no property for it**, and NM's own scan while connected measured one every ~300 s — 30x too slow for a robot that crosses a room in ten. So the decision goes back to the firmware: **`options brcmfmac roamoff=0`**, the driver's own default, and both backends then recognise it and leave it alone. Three consequences. The thresholds are **`WL_ROAM_TRIGGER_LEVEL` -75 dBm and `WL_ROAM_DELTA` 20 dB, compiled into the driver and reachable from no userspace interface** — the cost of this route, stated plainly, because if -75 dBm is wrong for this flat the answer is a mechanism (an `nmcli`-driven RSSI watchdog, or `wlan0` out of NM entirely under a bgscan-ing wpa_supplicant), not a config file. **The file name is load-bearing**: modprobe concatenates every `options` line sorted by *base name across all its directories* and the kernel takes the last duplicate, so `/etc/modprobe.d/99-mote-brcmfmac.conf` was read *before* the vendor's `rpi-brcmfmac.conf` and silently overridden — priority between `/etc` and `/usr/lib` applies to files of the same name, not to the merged list — hence `zz-mote-brcmfmac.conf`, and `install.sh` reads `modprobe -c` back and warns if its line is not last. And **the acceptance is a walk**: `pixi run wifi-roamlog` writes BSSID/signal/RTT once a second *plus the best other same-SSID AP in view*, since the firmware scans inside the chip and tells the host nothing, so without that column a log with no roam in it cannot separate "nothing better was there" from "it would not move" (the logger scans for itself through whichever daemon owns the radio — needing no root either way, and it has to be the right one, since `nmcli`'s rescan under iwd triggers no scan at all and answers from iwd's network list — every 15 s and only below -70 dBm, since a scan sweeps both bands and costs ~4 s of 90-114 ms round trips, which a `scanned` column marks so it is not read as the link degrading). `pixi run wifi-check` reports the one fact that settles who is deciding — `iw phy` prints `Device supports roaming.` exactly when `roamoff` is 0 — and reads it without root, unlike `/sys/module/brcmfmac/parameters/roamoff`. **Walked on 2026-09-01 and it roams**: four roams across three APs in 2 min 1 s, every one fired between -75 and -83 dBm onto an AP 25-35 dB stronger, each costing ~3 s of round trips with the tailnet ssh session surviving. That walk also found **a second fault the first was hiding, and it is not the robot's — worth knowing before deploying into any building**: **one SSID need not be one network.** An AP answering to the site's SSID with the site's key ran its own DHCP and its own NAT on a second subnet, so the roam onto it succeeded at every layer wifi is responsible for (the four-way handshake completes) and then moved the robot's address, costing 54 s of the walk at **-35 dBm** and 72 Mbps with every ping to the gateway lost — none of it wifi's fault. **What breaks is address-bound access, not everything**: the tailnet rebound and resettled in 4 s, but resettled *worse*, `NetInfo` flipping to `varies=true` — that AP's NAT on top of the site gateway's is the symmetric case that pushes traffic onto a relay, a real degradation for a camera stream. It looks exactly like a roaming fault and is not one, and **roaming working is what exposes it** (a robot that never leaves its first AP never meets the second subnet); the signature is the `ipv4` column changing across a roam, and the fix is on the network, not the robot. **The residual is that under load the firmware roams late**: walked again with 8 Mbit/s of TCP running, the address held but all three roams left an access point at -83 to -86 dBm (or none at all) for one 35-51 dB better, costing 29 of 99 samples their gateway ping in runs of 3-15 s with the negotiated rate on the 6 Mbps floor. `visible_same_ssid` stayed 1 for all 99 rows though two scans fired, where the idle walk on the same route produced candidates — the radio has no time off-channel while the stream runs, so the only scan that succeeds is the one after the link has degraded enough for traffic to stall, which is also when the roam finally happens. **More APs would not fix it**: the robot jumped -86 → -35 dBm in one second, so an excellent AP was already in range and being ignored. Recovery is prompt (full rate within 1-2 s, TCP catch-up bursts to 169 Mbit/s), and a mapping run is unaffected since `explore` runs on the Pi — it is teleop that pays. **The gap is not the scan**, which the arithmetic bounds at ~2.5-3 s (13 + 4 active channels at ~30 ms, ~16 DFS ones at ~110 ms passive) and the logger's own full scan measures at ~4 s — the rest is the firmware not starting one until traffic has stalled, so the ceiling is a policy compiled into the driver rather than a property of the radio, and the userspace watchdog the README specifies is not bounded by scan time either (its own scan problem is answered by disconnecting rather than scanning). **Accepted as it stands**: the robot roams, which is what the task was for. It is also why `roamlog` logs the interface address and calls a change out like a roam — that walk needed the DHCP journal to explain itself. Still unrun: the second walk under Foxglove and camera load.

**On-robot reliability** (see `mote_bringup/README.md`): `pixi run robot`/`mapping` include the health monitor, so a manual run publishes `/health` too; the systemd units are installed by `pixi run setup` but **not enabled** (autostart would drain the battery on a desk — opt in with `systemctl enable --now mote-bringup mote-health`). the systemd services restart with backoff and never permanently give up (`Restart=always`, `RestartSec`/`RestartSteps`/`RestartMaxDelaySec`, `StartLimitIntervalSec=0`), order after the udev-tagged `dev-mote_*.device` units, and bound the journal. A pre-flight self-check (`self_check.py`, run as `mote-bringup`'s `ExecStartPre`; `pixi run self-check`) gates bringup on servo ping + lidar/camera/disk/clock/config and keeps the robot idle with a clear reason on failure. A health monitor (**`mote_health`**, a C++ package of its own — see below; `mote-health.service` with a `Type=notify` watchdog; `pixi run health`) publishes per-subsystem `diagnostic_msgs/DiagnosticArray` on `/diagnostics_agg` and a single OK/DEGRADED/FAULT summary on `/health`. **Wheel slip needs no IMU**: kinematic_icp takes wheel odometry as its prior and corrects it against the scan, so the correction already measures how wrong the wheels were — `slip_monitor.py` (started by `mote_launch.py` beside `system_monitor`) compares the two over a 1 s sliding window and publishes `slip` / `stuck` / `icp_fault` as the `slip` status on `/diagnostics`, plus the raw residual on `slip/residual`. All three are DEGRADED, never FAULT: each is a reason to stop and re-plan, not to refuse to drive, and a monitor that can halt the robot on a threshold is a worse failure than the slip. The maths is ROS-free in `odom_residual.py` and **shared with `tools/slip_replay.py`**, which is what set `config/slip.yaml`'s thresholds from the residual distribution over the real mapping bags — a threshold calibrated offline only means something if the robot computes the same number. Two things are load-bearing. **Only translation is thresholded**: the yaw residual is published but its p99 reaches the yaw rate itself, so no threshold survives a hard turn — and a lag sweep puts the two streams within ±10 ms, so this is scan-match jitter, *not* the stamp skew task 165 suspected. **A stalled lidar must not read as slip**: the window would freeze at the last pose while the wheels keep turning, growing without bound, so a source older than `max_lag` yields no verdict at all. `health_monitor` lifts named statuses off the shared `/diagnostics` by exact name, listed in `health.yaml`'s `diagnostic_statuses`. Six real events (two stuck robots, four scan-match excursions) were found in the existing "known-good" bags; the derivation and the sim demonstration are in `docs/tuning/2026-07-28-slip-detection.md`. Driver and nav2 nodes are `respawn=True` for per-node recovery under the whole-service systemd restart. Battery voltage is **not** software-measurable (the power bank exposes no telemetry); `system_monitor` reports the Pi's `get_throttled` flags as the only power signal — read via **`vcgencmd`**, since the Pi 4 sysfs node does not exist on a Pi 5, alongside the Active Cooler's `fan_rpm` from the `pwmfan` hwmon.

**What a Python monitor costs is its wake-ups, not its work** (`docs/tuning/2026-08-11-monitor-cpu.md`; `pixi run node-cpu` is the per-node sampler, and it can weigh two builds of one node against each other in a single run because hardware drifts between runs — this robot's servo bus answered in one run and not the next, moving `/tf` by 18 Hz and inverting a sequential before/after). Measured on mote-01: a bare rclpy node costs 0.5% of a core, four subscriptions carrying 101 msg/s cost 7.9 points more, and deserializing them costs only 1.2 on top — so **`raw=True` can only ever recover ~13%**, and `health_monitor`'s watches take it (they count arrivals and never read a field, the `/diagnostics` one excepted). A `TransformListener` is the most expensive thing a node can hold: it takes the whole 51 Hz `/tf` stream for 4.8 points whatever handful of edges it asks about, which is most of what `task_server` costs while idle (`AcquireObject` creates one in `setup()` and uses it only during a fetch). `task_server` therefore ticks its tree at `tick_period` only while a task runs and at `idle_tick_period` between them — worth 0.9 points, not the 90% the tick-rate drop suggests — and `_set_tick_rate` **resets** the timer as well as re-periodding it, because setting a period does not move the expiry already pending, so without the reset the first tick of an accepted tree waits out the rest of the idle period. The consequence for anything new here: a 1 Hz monitor written in Python cannot get under ~5% of a core while watching the robot's real topic rates — `health_monitor` sat at ~17 — and the fix is a language change, not a smarter callback. **That change has been made** (`mote_health`, below, and `docs/tuning/2026-09-01-health-monitor-cpp.md`); `slip_monitor` remains the largest Python consumer and is deliberately *not* a port candidate as it stands, because its maths is shared with `tools/slip_replay.py` and that sharing is what set `slip.yaml`'s thresholds from real bags.

**Launch hierarchy:** the two mission launches (`mapping_launch.py`, `robot_launch.py`) each take a `base` arg (default true) that includes the hardware base, and a `use_sim_time` arg they forward to everything they include. The sim runs these *same* files with `base:=false`, supplying a Gazebo base in place of the drivers — so the missions are defined once and the sim exercises the real launch files.
- `robot_launch.py` — nav mission: `mote_launch.py` (if `base`) + `nav2_launch.py` (drive a saved map). Forwards a `map` arg, defaulting to the active site's map (see Sites).
- `mapping_launch.py` — mapping mission: `mote_launch.py` (if `base`) + `slam_launch.py` + `nav2_launch.py` (`localisation:=false`) + `record_launch.py` (`streams:=mapping`, unless `record:=false`): build/extend a map with SLAM *and* drive to goals autonomously while doing so, recording the session for map provenance.
- `mote_launch.py` — the hardware base: robot_state_publisher, ros2_control_node, controller spawners, sllidar, laser_filter, v4l2_camera, `localization_launch.py`, `twist_mux_launch.py`, and `foxglove_launch.py` (`foxglove:=true`). Reads `robot.yaml` for wheel geometry (injected into DiffDriveController params) and sensor config. Asserts `use_sim_time` (default false) for the whole tree via `SetParameter`.
- `localization_launch.py` — kinematic_icp LIDAR odometry (the `odom`→`base` edge; the map→odom corrector is slam_toolbox when mapping or AMCL when navigating). Despite the name, it does *not* run AMCL — AMCL lives in `nav2_launch.py`. All three parts are components in one `localization_container` (`component_container_isolated`, as in `nav2_launch.py`): kinematic_icp; `mote_nav::OdomTfRelay` writing the `odom_wheel` leaf it reads as its motion prior; and `mote_nav::IcpOdomGate`, which is what actually **broadcasts `odom`→`base`** — three processes and three DDS participants become one, and the relay stops being a Python interpreter woken 50 times a second. The gate consumes kinematic_icp's *topic*, not its broadcast: a TF broadcast cannot be retracted, so a gate downstream of ICP's own `odom`→`base` would be no gate at all. ICP therefore runs `lidar_odom_frame:=odom_icp` (`ICP_ODOM_FRAME`) with `invert_odom_tf:=true`, which swaps the frame ids as well as the transform and so writes `base`→`odom_icp` — a *leaf*, exactly like `odom_wheel`, rather than a second claim on the base's parent. **That leaf is not decoration: `slip_monitor` reads it.** Its `icp_fault` verdict fires on a body speed above the drive envelope, which is precisely what the gate removes from `odom`→`base`, so pointed at the gated edge the check could never fire again and a scan match degrading behind a working gate would be reported by nobody — while the monitor went on publishing OK. Both leaf names are constants in `mote_bringup/launch_utils.py` (a launch file cannot import another, and `mote_launch.py` needs `ICP_ODOM_FRAME` for the monitor) because a disagreement between any writer and reader costs the reader its input without failing anything — as does every other seam here (the gate's remap must match ICP's *namespaced* topic; ICP must not re-claim the edge) — and `test_localization_composition.py` holds all of it, since composition's failure modes (an unnamed node taking defaults, a plugin string matching no registered component, two publishers on one TF edge, a monitor watching the wrong frame) are all silent.
- `slam_launch.py` — slam_toolbox (accepts `use_sim_time:=true` for the sim)
- `nav2_launch.py` — Nav2 stack, **composed**: all nine servers plus both lifecycle managers are `ComposableNode`s loaded into one `nav2_container` (`component_container_isolated`, which gives each component its own executor thread — the shared-executor containers would serialise servers that block inside callbacks). Ten processes become one, which is also ten DDS participants become one. The drivers in `mote_launch.py` stay separate processes: composition trades crash isolation for efficiency, and the drivers are the crash-prone half. Two things are load-bearing and non-obvious. **The params file goes on the container as well as on each component** — Nav2's servers create further nodes of their own (`/local_costmap/local_costmap`, `/global_costmap/global_costmap`, the bt_navigator client nodes) which are not components and so are never named in a load request; they inherit the *process* command line, which inside a container is the container's, so without it the costmaps come up on library defaults and nav quietly degrades rather than failing. **Each `ComposableNode` must carry `name=`** matching its `nav2_params.yaml` key, because a composable node loaded without a name is matched against no section of the file and receives no file parameters at all. A `localisation` arg (default true) toggles the `map_server` + `amcl` half: true localises against a saved map; false drops them so the navigation servers run against a live slam_toolbox map and `map→odom` instead (used by `mapping_launch.py`). Recovery is now whole-stack: the container is `respawn=True`, and the component loads are re-issued on *every* `OnProcessStart` (via an `OpaqueFunction` returning fresh actions, since an executed action cannot run twice) so a respawned container is refilled rather than coming back empty. `slam_toolbox` is deliberately left as its own process, as upstream `nav2_bringup` leaves it
- `twist_mux_launch.py` — the drive path's velocity arbiter, so the controller has one publisher (see Drive path above). Included by `mote_launch.py` and by `sim_launch.py`, because the drive path must not depend on which base is under the mission
- `foxglove_launch.py` — the remote console: `foxglove_bridge` (WebSocket on 8765) plus the `twist_relay` teleop seam (`teleop:=true`). Included by `mote_launch.py`; run alone as `pixi run foxglove`, or as `mote-foxglove.service`
- `rviz_launch.py` — RViz2 (dev environment only)

**Config files** (`mote_bringup/config/`):
- `controllers.yaml` — controller_manager update rate, DiffDriveController settings, and the arm's `JointTrajectoryController` (wheel geometry *and* the arm's joint list are injected from `robot.yaml` at launch time, not stored here — `launch_utils.joint_params_file` writes them to a temp params file keyed by node name, since a plain dict would never reach the controller node)
- `twist_mux.yaml` — the drive path's inputs, priorities and timeouts (see Drive path above)
- `laser_filters.yaml` — filters lidar blind spots
- `nav2_params.yaml` — Nav2 parameters
- `slam_toolbox_params.yaml` — SLAM toolbox parameters, as run on the robot during capture. Best-known-good values only: a bad scan match cannot be taken back mid-mission, so this file is never deliberately hobbled
- `slam_toolbox_build_params.yaml` — the same parameters for an **offline map build** from a recorded bag (mapping-pipeline stage 2), where a bad solve costs one discarded candidate rather than a mission. A whole copy rather than an overlay, because slam_toolbox loads one file; every value must match the live one unless the key carries a `# DIVERGENCE:` note saying what the build buys, and `test_slam_build_params.py` fails on an undeclared divergence *and* on a declared one that no longer diverges. It also records what was measured and rejected, so the same sweep is not run twice — notably a finer `coarse_angle_resolution`, which does not help: Karto refines over ±half that value in `fine_search_angle_offset` steps, so the reachable angles are a contiguous 0.2° grid at any coarse value and there is no orientation lattice to escape. Measurements in `docs/tuning/2026-08-25-slam-build-params.md`
- `mote.rviz` — RViz2 display config
- `cyclonedds.xml` — loopback-only DDS transport: the graph never touches a radio, so a wifi flap cannot stall topic delivery between on-robot processes (Cyclone otherwise prefers a radio interface's locators even for same-host peers, and `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` confines discovery, not transport). Loaded via `CYCLONEDDS_URI` everywhere — pixi activation and the systemd units alike — so no off-board machine joins the graph: Foxglove (a WebSocket server, not a DDS peer) is the window, and camera calibration, the one flow needing a LAN DDS peer, unsets the profile explicitly (`mote_perception/config/README.md`)

### `mote_simulation` (Python/ament)
Workstation-only Gazebo simulation, kept separate from `mote_bringup` so it can be excluded from the robot sync (`pixi run sync` skips `mote_simulation/`). Built only in the `sim` pixi environment. Contains:
- `launch/sim_launch.py` — Gazebo sim: headless gz server, robot spawn, ros_gz bridge (/clock, /scan), controllers, laser_filter, and the shared `localization_launch.py`. Takes a `world:=` arg (file in `mote_simulation/worlds/`, default `mote_world.sdf` — the simple smoke-test room; `office_world.sdf` is a medium hospital-ward corridor for stress-testing localisation; `hospital_world.sdf` is the hard tier — a ~58x38 m looping hospital, generated by `worlds/gen_hospital.py`). The URDF is processed with `use_sim:=true`, which swaps `MoteHardware` for `gz_ros2_control` and adds a simulated lidar (specs from `robot.yaml` `lidar.sim`). Without that flag the xacro output is unchanged. Controller params are merged into one temp file (gz_ros2_control loads a single `<parameters>` file referenced in the URDF). It pulls `controllers.yaml`, `laser_filters.yaml`, and `localization_launch.py` from `mote_bringup`'s share so the sim and the real robot can't drift apart. It asserts `use_sim_time:=true` for the whole process tree via `SetParameter`. A `mode:=mapping|nav` arg includes the real `mapping_launch.py` / `robot_launch.py` with `base:=false` (default `none` = sim only): the sim provides the base and *delegates* the mission to the actual launch files, so it can't re-encode or drift from them, and `pixi run sim-mapping` / `sim-nav` put those mission launches under test. Mission modes also include `tasks_launch.py` with the loaded world's sibling `worlds/<world>.zones.yaml` as `zones_file`, so the fetch mission runs anywhere on the world ladder with matching zone coordinates — and since that file's room zones carry footprints, `goto <zone>` runs on the ladder too. (`pixi run mapping`/`robot` are the hardware entry points — same files, `base` defaulting true, wall-clock time.) The dependency direction stays one-way: `mote_simulation` includes from `mote_bringup` and `mote_tasks`, never the reverse.
- `worlds/` — an easy->hard ladder: `mote_world.sdf` (easy smoke-test room), `office_world.sdf` (medium hospital-ward corridor), and `hospital_world.sdf` (hard ~58x38 m looping hospital with ~50 rooms and clutter). The hard world is generated by `worlds/gen_hospital.py` (committed alongside its output — edit the script's layout and regenerate rather than hand-editing the SDF). Every world has a sibling `<world>.zones.yaml` with the same waypoint zones (`pickup`/`dropoff`/`home`) plus a few room zones (reachable by `goto`) carrying a footprint: the two smaller worlds use a `radius` circle, the hospital's rooms use `polygon` outlines of the actual ward rectangles. The hospital's is emitted by the generator, which asserts every zone clears the walls and furniture and lies inside its own footprint.
- `test/sim_smoke/` — `run_sim_smoke.sh` + `verify_sim.py`, the `pixi run sim-test` gate. Claims its own domain and partition (`tools/sim_domain.py`) and logs them, so it can run beside a benchmark or another worktree's sim.
- `tools/sim_domain.py` — the shared domain/partition picker (stdlib-only, so the bash entry points `eval` it; `test_sim_domain.py` pins the port arithmetic and the shell contract). See DDS scoping under Environment.
- `test/room_segmentation_eval.py` (`pixi run segment-eval`) — scores map room segmentation (see Sites) against `worlds/<world>.rooms.yaml`, the walkable rectangle of every enclosed room: emitted by `gen_hospital.py` for the generated world, read off the SDF for the two hand-written ones. Only rooms the exploration run actually mapped are scored (20 of the hospital's 53 were never entered), and `--rotate` turns map *and* truth together to exercise the wall-alignment step against real SLAM data, since every world on the ladder happens to be axis-aligned and a real map frame is not.
- `sim_home/` — a committed, in-repo **sim MOTE_HOME**: one real Site bundle per world (site name == world stem, floor `ground`) holding that world's SLAM map + zones. The sim pixi env points `MOTE_HOME` here (`[feature.sim.activation.env]`), so `sim-nav` loads a world's own map and never touches the robot's real `~/.mote`; `sim_launch.py` nav mode resolves the map from the `world` arg via the world's site and passes it to `robot_launch.py`. Only the bundles are committed — `active.yaml`/`bags/` are gitignored.
- `tools/map_world.sh` — how those sim sites are built, the same way the robot maps: `pixi run sim-map-world <world.sdf> [budget_s]` launches the real mapping mission headless, runs `mote_bringup`'s `explore` with `--sim-time` (the same autonomous-coverage tool the robot runs — see `mote_bringup`), then `save-map`s into the world's site. Sim maps are ground-truth-clean, so the save uses `save_map(clean=False)` (serve the raw map_saver output): the FFT declutter pass, tuned for real-sensor noise, would strip the thin true walls. Re-running adds a new revision.

### `mote_perception` (Python/ament)
Home for camera-derived perception. Runs on the robot (feeds Nav2), so unlike `mote_simulation` it is synced to the Pi. Contains:
- `mote_perception/camera_monitor.py` — a dependency-light camera health monitor (rclpy + sensor_msgs only, no OpenCV). Subscribes to `image` and logs measured frame rate, resolution, and encoding on a timer, warning on dropouts. Registered as the `camera_monitor` console_script.
- **L1 depth-obstacle pipeline** — turns the mono camera into `/camera_obstacles` (PointCloud2) for Nav2's `camera_layer`. That layer is a **`spatio_temporal_voxel_layer`, not `nav2_costmap_2d::VoxelLayer`**, and has to be: the cloud carries only *above-floor* points (the floor is stripped to keep the stream off the Wi-Fi), and a voxel layer clears only by raytracing towards a point in a *later* observation — so a transient that walks through and leaves takes its points with it, no clearing ray is ever cast that way, and the mark is permanent (measured: still marked after 20 s with the robot stationary; `docs/tuning/2026-07-29-camera-layer-decay.md`). STVL expires a voxel `voxel_decay` seconds (5.0 — ~1.1 m of travel at the measured `max_wheel_speed`, bounded below by the robot needing to remember an obstacle while steering round it and above by how long a transient may linger) after it was last marked instead: what is still in view is re-marked and never expires, what leaves goes on its own, measured at 5.2 s. Two settings are load-bearing rather than cosmetic and both fail *silently*. **`clear_after_reading: True`** — the measurement buffer holds its newest cloud until something empties it and every costmap update re-marks what it reads, restamping the voxels with the current time; left at the default, STVL is exactly as permanent as the layer it replaced (measured). **`filter: "passthrough"`** — STVL applies `min/max_obstacle_height` *as that filter's z limits*, so `"none"` drops the go-under gate rather than just the downsampling. Frustum clearing is deliberately off: a mis-stated FOV would clear real obstacles the lidar plane cannot see, and decay already answers staleness. `test_costmap_layers.py` holds those settings plus a pluginlib-index check (the package is not in the nav2 metapackage set, so it reaches the robot only because `pixi.toml` names it, and a missing layer class is one buried log line while nav2 comes up around it); `pixi run camera-decay-check` times a real mark against a real costmap in ~40 s with no hardware. Split across: `depth_obstacle_node.py` (torch-free rclpy node: compressed image → server → rescale → back-project → level → z/range gates), `depth_wire.py` (the socket protocol spec + `DepthClient`, shared by node/server/tools), `lidar_rescale.py` (per-frame Theil-Sen affine-in-disparity metric rescale anchored to lidar), `ground_projection.py` (camera↔base geometry: `GroundProjector`, floor-plane fit, leveling, pixel→floor rays). Split by concern, not by machine: `depth_obstacle_node` runs on the robot (launched by `perception_launch.py`, in its DDS graph), reaching the torch server over TCP at `inference_host`. `tools/depth_server.py` runs in the `inference` pixi env (torch, no ROS) wherever the GPU is; `pixi run inference` starts it beside the detect server. `pixi run inference-rocm` is the AMD GPU variant: the same servers in the `inference-rocm` env (torch from the pytorch.org ROCm wheel index, own solve; `HSA_OVERRIDE_GFX_VERSION` set for unsupported AMD iGPUs). As a *deployed role* on a dedicated NVIDIA machine (gaming PC or cloud GPU) the same servers ship as a **container image** (`mote_perception/deploy/Dockerfile` -> `ghcr.io/clachdev/mote-inference`, built by `.github/workflows/inference-image.yml`): that host installs no repo, pixi, or scripts — one `docker run --gpus all --restart unless-stopped`. Every variant runs the same supervisor (`tools/inference_server.py` — add a tenant by adding a row to `SERVICES`). Models load on demand and release after `--idle-timeout` (default 300 s) via `tools/model_host.py`, so the machine isn't holding VRAM while idle. The full role (host decision, update story, cloud scaling + the unauthenticated-socket caveat, `pixi run inference-health`, `pixi run inference-bench`, multi-service pattern, fallback matrix) is in `docs/inference-server.md`; wire modules carry a `HEALTH_MAGIC` request so `WireClient.health()` reports each server's model/device/version (`MOTE_VERSION` baked into the image, else `git describe`). The server takes `--device auto|cpu|cuda` (auto → GPU when available, else CPU) and optional `--fp16`. The iGPU doesn't beat the CPU at idle (small ViT, bandwidth-bound) but stays flat under CPU load where the CPU-only server degrades to ~1–2 s/frame; fp16 and larger models can crash/hang on unsupported iGPUs (gfx1103), so keep fp32 + V2-Small there. Needs `/dev/kfd` access (render/video groups).
- **L2 open-vocabulary detection** — turns "fetch the red box" into a map pose for the task layer. Same node-on-robot / server-off-board split: `tools/detect_server.py` (OWLv2 in the `inference` pixi env; `pixi run detect-server`, or `pixi run inference` for both), `detect_wire.py` (protocol + `DetectClient`; the query labels ride in each request), `object_detector_node.py` (torch-free rclpy node: idles until labels arrive on `detect/labels` — String, comma-separated, transient_local, empty = idle — then grounds each bbox bottom-centre through the floor plane and publishes `detected_objects`, vision_msgs/Detection3DArray in the map frame at the capture stamp). Floor-ray grounding is metre-accurate only near the robot (camera is at ~0.10 m), gated by `range_max`.
- `tools/` — offline bag harnesses (`depth_bag_replay`, `depth_bag_eval`, `depth_obstacles`, `detect_bag`, `bag_overlay`, shared `bag_utils`) and the live `measure_camera_pitch`; see `mote_perception/README.md` for the inventory.
- `launch/perception_launch.py` — declares `use_sim_time` (applied via `SetParameter`) and starts `camera_monitor` (with `image` remapped to `/image_raw`) plus the depth/detect nodes. Which nodes run, their `server_port`s, and the shared `inference_host` come from `config/perception.yaml` (not launch args), so inference can move machines without editing launch. Not part of the mission bringup — run `pixi run perception` alongside `pixi run mapping`/`robot`.
- `config/` — camera-calibration + perception-runtime home. `camera_info.default.yaml` is a committed fallback calibration for the UGREEN webcam; a per-robot `~/.mote/camera_calibration.yaml` (outside the repo) overrides it. `mote_launch.py` prefers the `~/.mote` file when present, else `robot.yaml`'s `camera.default_info_url`, passing the result to `v4l2_camera_node` as `camera_info_url`. `perception.yaml` (`inference_host`, per-node `enabled`/`server_port`) is read by `perception_launch.py` with the same `~/.mote/perception.yaml` override. `config/README.md` documents when/how to calibrate (with the printable checkerboard).
- Compressed transport is already provided by the `image-transport-plugins` dep, so the camera publishes `/image_raw/compressed`; off-board/RViz consumers should prefer it. See `mote_perception/README.md`.

### `mote_tasks` (Python/ament)
The task layer: py_trees behaviour trees on top of Nav2 (synced to the Pi). py_trees is a pixi *PyPI* dependency (not packaged on robostack/conda-forge); the ROS glue is first-party and small — no py_trees_ros. Contains:
- `capabilities.py` — what this robot can be asked to do, as a capability/v0 document: `goto` with `{target}` and `fetch` with `{target, destination}`, both standard-registry keys with the registry's own property names. See "Fleet: adopting spec v0" above.
- `task_server.py` — node hosting the mission trees. Three `std_msgs/String` topics carrying JSON: it publishes its capability set on `task/capabilities` (latched), takes mission/v0 commands on `task/command` and answers with mission/v0 statuses on `task/status`. A command names a capability key and carries a typed `input` validated against that capability's `input_schema`; the failures it publishes are typed (`unknown_capability`, `invalid_input`, `busy`, `precondition`, `unresolved_zone`, and on the way out `obstructed`/`unreachable`/`timeout`/`internal`). **It owns the lane** — one mission at a time, rejected with `busy` naming the holder — and evaluates the blocking preconditions before accepting. Zone names → map poses come from a zones YAML resolved via Sites (active floor, then legacy `~/.mote/zones.yaml`, then the committed `config/zones.default.yaml` whose poses match `mote_world.sdf`; in the sim, `sim_launch.py` passes the loaded world's own zones file instead). `save_zone` (`pixi run save-zone <name> [--radius R]`) teaches zones from the live robot pose; `mission.py` (`ros2 run mote_tasks mission`) dispatches one from a terminal, which is what a JSON seam took away from `ros2 topic pub`.
- `zones.py` — the one named-place concept: `load_zones` returns `{name: Zone(name, pose, footprint)}` from the `zones:` section. A **zone** is a pose the robot navigates to (both fetch waypoints *and* goto targets resolve against this single table); it *optionally* carries an area **footprint** — a `radius` circle or a `polygon` of explicit vertices — that's just optional metadata, not a second type. `zones.containing(zones, x, y)` is the "which zone am I in" membership query (nearest-pose first) over zones that have a footprint; `goto` itself only needs the pose. A polygon may be concave (ray-cast membership, so an L-shaped ward or a corridor stretch works where a circle can only under- or over-cover: the hospital wards are 4.7x5.6 m, of which a `radius: 1.5` circle claimed 7.1 m² of 26.5 m²), wins over a `radius` if a zone carries both, and — since polygons come from post-processing a map or from the dashboard's editor rather than from driving — may omit `x`/`y`, in which case the loader derives a pose guaranteed to lie inside the outline. `save-zone` therefore preserves an existing footprint when it re-teaches a pose; `--radius` is the explicit way to replace one. Polygon zones no longer have to be hand-written either: `pixi run segment-map` proposes one per room of a saved map (see Sites). A `Zone` also carries the **vocabulary** half — `note` and `navigable` (zone/v0; see "Fleet: zones are place-names") — both optional, so no existing `zones.yaml` needed rewriting and a zone that says nothing but its name is somewhere a robot may drive to. Three consequences here. `resolve(zones, query)` is what `goto`/`fetch` match on: the name exactly, then case-insensitively and whitespace-normalised, which is what makes `store room` typeable; both then refuse a **non-navigable** zone rather than driving to it — `fetch` explicitly, because falling through to its label branch would send the detector hunting for an object called "server room". `load_zones` **refuses a vocabulary with a collision** (two zones answering one query), because loading it would resolve `goto` by dict order — silently, once per boot, differently after an edit; the rules live once in `mote_bringup.bundle` (`zone_term`, `ambiguities`, `check_vocabulary`) so the robot, `save-map` and the fleet server cannot disagree about what a vocabulary means. And `append_zone` carries the vocabulary through a re-teach and bumps `vocabulary_revision`: a better coordinate is not a rename.
- `behaviours/` — `DriveTo` (Nav2 NavigateToPose action client as a behaviour; cancels in-flight goals on preemption), `AcquireObject` (label missions: publishes the label to `detect/labels`, waits for a matching `detected_objects` detection, writes a standoff goal — 0.4 m short of the object, facing it — to `object_pose`; zone missions pass through), and `TimedStub` (placeholder pick/place until the SO-101 arm is actuated).
- `trees/` — `common.py` (shared `WaitForTask` + the `task` blackboard key), `fetch.py` (wait → acquire object → drive to object → pick stub → drive to drop → place stub; blackboard keys `task`/`object_pose`/`object_label`/`drop_pose` are the seam between the command grammar and perception), and `goto.py` (wait → drive to the zone's pose; success == Nav2 success).
- `test/` — mock-`navigate_to_pose` tree ticks (`test_fetch_tree.py`, `test_fetch_object.py` against a mock detector, `test_goto_tree.py`) plus pure parser/loader tests (`test_parse_command.py`, `test_goto_command.py`, `test_zones.py` — which covers zone footprints and `containing`), no Gazebo/Nav2 needed, run by `pixi run test`.

### `mote_fleet` (Python/ament)
The fleet control plane — one package for both ends of one wire, the same split `mote_perception` makes for inference: a node on the robot, a server off-board, and a single shared wire module so they cannot drift. Details and rationale in the Fleet section above and `mote_fleet/README.md`. Contains:
- `protocol.py` — the versioned contract (topic tree, payload builders, task states). **Stdlib-only and ROS-free**, so the off-board server imports it from the source tree by path rather than vendoring a copy. `schema/*.schema.json` is its machine-readable mirror and `test_protocol.py` fails if code, schema and `docs/fleet/control-plane.md` disagree.
- `dispatch.py` — the single-in-flight tracker and the parser for `task_server`'s status strings. No ROS, no MQTT, so every ambiguous attribution case is a plain function call in `test_dispatch.py`.
- `agent.py` (`pixi run agent`, `mote-agent.service`) — the node. Presence/health/pose up (retained, LWT), one command at a time down. The MQTT client is injectable, which is how `test_agent.py` covers the whole bridge in CI without a broker.
- `enroll.py` (`pixi run enroll`) + `facts.py` — the robot side of enrollment and the hardware fingerprint it is idempotent on. `fleet_config.py` owns `$MOTE_HOME/fleet.yaml`.
- `server/` — ROS-free scripts for the fleet box: `fleet_server.py` (stdlib `http.server`: enrollment, roster, dispatch, audit, basemaps, and the UI), `registry.py` (SQLite rows — robots, enrollment tokens, operators, the audit log; state under `$MOTE_FLEET_HOME`, default `~/.mote-fleet`), `fleetctl.py` (`pixi run fleetctl`: token/operator/robots/dispatch/audit/watch), `ui/` (the dashboard: static ES modules, a subscribe-only MQTT client, the Q5 map transform), `mosquitto.conf` + `broker.sh` (conda or container, the latter for WebSockets). Every write to `mission/command` — CLI or browser — goes through the API, so dispatch is authorized and audited in one place.
- `mapsync.py` + `publish.py` (`pixi run publish-map`) — the map registry's robot side (M4): pull the canonical revision announced on the retained topic, or offer a saved one as a candidate. ROS-free, so the whole distribution flow is testable as function calls.
- `server/bundle_store.py` — the registry's byte store: candidate revisions, validation on the way in (via `mote_bringup.bundle`), and the atomic symlink flip that publishes one. The filesystem is the truth about what is canonical; the database records who promoted it. It is also where the vocabulary/binding split is enforced in reads: `read_zones` (the binding) is gated on a published map, `read_vocabulary`/`vocabularies` are not, and only the latter go out over `/v1/zones`.
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
- **The arm is part of `mote_hardware`'s ros2_control component**, not a driver
  of its own: `MoteHardware` exports position command interfaces for the six arm
  joints alongside the wheels' velocity ones, from one `open()` of the shared
  bus (below). `control.py` is the one place that knows how to command it.
- `arm_launch.py` lives in **`mote_bringup`** (`pixi run arm`) — bench bring-up:
  the same controller_manager, URDF and `controllers.yaml` a mission uses,
  without lidar/camera/Nav2. During a mission the arm needs nothing extra; it is
  already there. The dependency runs `mote_bringup` -> `mote_arm` and never back:
  the base launch imports `mote_arm.config` to resolve this robot's calibration
  into the URDF (below), so `mote_arm` must not import `mote_bringup`.
- **The calibration has to reach the URDF.** `zero`/`min`/`max` are measurements
  of one arm and live in `$MOTE_HOME/arm.yaml`; robot.yaml holds placeholders.
  Since `MoteHardware` enforces the clamp and reads its limits from the URDF,
  `launch_utils.resolved_arm` overlays the two via `mote_arm.config.load` (the
  one implementation) and passes the result to xacro as `arm_config:=`. A bare
  `xacro mote.urdf.xacro` falls back to the placeholders — fine for checking
  generation, wrong for driving a calibrated arm, because calibration moves the
  zero and every commanded angle then names a different position.
- `jog` (CLI, `pixi run arm-jog`) — interactive per-joint jog; a *client of
  `arm_controller`* (publishes clamped single-point trajectories, limps on
  exit). It never opens the bus, so there is no contention to guard against.
- **Every arm CLI exits and parses through `cli.py`**, because both properties
  fail silently when hand-rolled. `cli.shutdown(node, spinner)` shuts the
  context down, **joins the spin thread, and only then destroys the node**:
  destroying a node `spin()` still holds aborts the interpreter (exit 134,
  "terminate called without an active exception") *after* the tool has done its
  work, so the run succeeds and the process still crashes — measured on `jog`
  and `arm-pose list`, 3 of 3 runs each, with no hardware attached. This is not
  a rare race, so a new arm CLI must not hand-roll the teardown.
  `cli.parse(parser)` cuts the `--ros-args ... --` block out and then parses
  strictly: `ros2 run` hands the tool ROS's arguments too, so a plain
  `parse_args` rejects `--ros-args` outright while `parse_known_args` — the
  usual workaround — silently discards a mistyped `--max-travel` or `--speed`
  and drives on the default. `test_cli.py` pins both, the abort via a child
  process's exit status since nothing in-process can catch it.
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
- `arm_limits` (`pixi run arm-limits show|clear|restore`) — **a fourth place a
  limit can live, and the only one not in a file.** EEPROM registers 9 and 11
  (`Min_Angle_Limit`/`Max_Angle_Limit`) fence which goals a servo accepts and
  refuse the rest **in silence**: no error, no status bit, no log line, so the
  joint stops at the same angle every time, in one direction, at any load —
  indistinguishable from running out of torque. This arm arrived with five of
  six joints fenced *inside their own travel*, and it presented as teleop being
  "stuttery and not going its full range": `shoulder_lift` stopped at -0.865 rad
  against a configured -1.7785, at 0% load, with the command running 0.8 rad
  past it, and its `Min_Angle_Limit` read 1478 = -0.874 rad about zero 2048.
  Two properties hid it. The fence binds **only under torque**, so
  `arm-calibrate` sweeps a limp joint straight through it and measures travel
  the arm will then refuse — the calibration and the arm disagree and only the
  arm is wrong. And the band is compared against the **corrected** goal, so
  moving a zero moves what it fences without changing any number a person can
  read. `arm-calibrate` therefore clears the fence in phase 2 *before* it writes
  an offset, snapshotting the as-found bands to `~/.mote/arm_limits_backup.yaml`
  first; `arm-check` reports the band beside the configured one. **Cleared, not
  narrowed to match**: the guard is the soft limit in `$MOTE_HOME/arm.yaml`,
  enforced by `MoteHardware::clamp_rad` and `teleop.py`, which is versioned and
  printed by three commands — a second copy in EEPROM adds nothing until the two
  disagree, and then it wins invisibly. Hence no `arm-limits set`.
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
  on `/dev/mote_servos`), so it needs no udev rule. A serial port has no
  kernel-level exclusion, so exactly one process may hold it — and that process
  is the **controller_manager**. `MoteHardware` drives both halves: velocity
  interfaces for the wheels, position interfaces for the arm. That is what lets
  the arm move *during* a mission with Nav2 live, which is the whole point of
  having it. `arm_controller` is a `JointTrajectoryController` spawned
  **inactive** — claiming its command interfaces is what enables servo torque
  (`perform_command_mode_switch`), so "limp until asked" is a property of the
  control stack rather than a rule a driver has to remember. Two guards stay:
  an arm ID colliding with a wheel ID is rejected in `config.py` *and* in
  `MoteHardware`, and both `MoteHardware::on_activate` and `mote_arm.bus` refuse
  a port another process already holds (naming the PID) — so the read-only bench
  tools (`arm-check`, `arm-gains`), which still open the bus directly, need the
  control stack stopped (`pixi run kill`). `jog` and `arm-pose` do not.
- Torque policy, control interfaces, and calibration in `mote_arm/README.md`;
  the human bench runbook in `mote_arm/BENCH.md`.
- **Virtual-leader teleop + episode recording** (`mote_arm/TELEOP.md`) — teleop
  with **no leader arm**: a leader pose held in software, moved by the keyboard
  (`virtual_leader`, `pixi run arm-teleop`), published on `leader/joint_states`,
  which `arm_mirror` (`pixi run arm-mirror`, or `pixi run arm mirror:=true`)
  turns into `arm_controller` trajectories through `control.py`, like every
  other command client. **The frontend is deliberately replaceable** — the
  mirror's whole contract is `leader/joint_states` + a latched `teleop/estop`,
  so a slider GUI or a gamepad is a drop-in. LeRobot's own teleop was rejected
  for the reason the bring-up rejected LeRobot on the robot at all: it would put
  torch on the Pi. **Every safety rule lives in `teleop.py`** and nowhere else —
  soft-limit clamping, a 0.5 rad/s rate limit (so a leader that *jumps* becomes
  a ramp), the deadman (the leader's *liveness* is the deadman: a released key,
  a closed window and a dropped SSH session all arrive as "no fresh pose", and
  the mirror then issues one goal at the arm's present position so it stops
  there rather than coasting on), the latched panic (deactivates
  `arm_controller` — torque *is* controller activation — and refuses goals until
  cleared), and re-seeding from measured on every resume so a pause cannot bank
  up motion. **The mirror ticks on its own thread, not on a ROS timer**: taking
  hold of the arm is a `switch_controller` call, and a service call made from
  inside an executor callback can never complete, because the future is resolved
  by the executor the callback is blocking (`arm-jog` avoids this by driving
  from its REPL thread). `mock_arm` (`pixi run arm-mock`) presents that same
  ros2_control surface — trajectory topic plus `switch_controller` — with no bus
  and an optional pure-zlib synthetic camera, so the whole loop runs on a
  workstation: `pixi run arm-teleop-test` drives it headless and is the
  pre-bench gate; `pixi run arm-bench-teleop` is the guided hardware session.
  **Episodes**: `episode_record` samples `joint_states` (observation), the
  `arm_controller/joint_trajectory` topic (action — read off the wire rather
  than from the mirror, so an `arm-jog` session records too) and
  `/image_raw/compressed` into a **capture** under `$MOTE_HOME/episodes/` — JSON
  lines plus the compressed frames stored byte-for-byte, written with the
  standard library alone, because the Pi carries no parquet or ffmpeg.
  `tools/lerobot_export.py` (`pixi run -e lerobot arm-export`) converts a
  capture into a real `LeRobotDataset` **through LeRobot's own API**
  (`create`/`add_frame`/`save_episode`/`finalize`, then loads it back to verify)
  rather than emitting the files — the format already moved once (v2.1 → v3.0)
  and a hand-rolled writer would be wrong the next time. It resamples onto the
  exact 1/fps grid first, since LeRobot derives timestamps from the frame index
  and would otherwise silently stretch a slipped capture. `episode_replay`
  (`pixi run arm-replay`) reads the *capture*, not the dataset, so replay needs
  nothing off-board; it approaches the first pose, replays at a quarter speed,
  and stops on sustained lag (`motion.py`, shared with `arm-pose go`). Stop the
  leader before replaying — two things commanding `arm_controller` fight.
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
- **Physical note (GitHub #2):** the camera and the arm fouled each other, so
  the arm is mounted **rotated 180 degrees** (option 1 of that issue). The
  camera clears it, barely, and the cost is forward reach. **`arm_mount_joint`
  in `mote.urdf.xacro` is still `rpy="0 0 0"`** and so describes the old
  orientation: joint-space work is unaffected (nothing there asks where the
  gripper is in the base frame), but TF draws the arm facing the wrong way and
  anything reasoning in base coordinates — a fetch standoff, an IK stack —
  would be 180 degrees out.
  The arm *is* part of the mission bringup now (it is in `mote_hardware`), but it
  stays limp until a controller claims it.

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

**DDS scoping.** DDS transport is loopback-only in every environment (`config/cyclonedds.xml`, loaded via `CYCLONEDDS_URI` set in `[activation.env]` and repeated by the robot's systemd units): the robot's graph must never depend on a radio — a wifi flap otherwise stalls even same-host delivery — and a workstation's sims/benchmarks are same-host graphs anyway. No machine joins another's DDS graph; Foxglove is the off-box window, and camera calibration (the one flow needing a LAN DDS peer) unsets the profile explicitly (`mote_perception/config/README.md`). The `sim` environment additionally sets `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`; every entry point that brings up a gz server — `bench.py`, the smoke test, `map_world.sh` — claims a free `ROS_DOMAIN_ID` + `GZ_PARTITION` per invocation through the one picker (`mote_simulation/tools/sim_domain.py`), so two runs on one machine stay separate. Isolation is only half of it: **teardown must be scoped too**, or a run reaps another's processes however cleanly their graphs are separated. Each launch is `setsid`-ed, so its session id is the exact scope — it reaps stragglers under any node name and can reach nothing else — with a repo-path-scoped `pkill` for a `gz sim` that escaped even that. Bare name matches (`pkill -f mote_world`) are what this replaces. Discovery visibility is one-way: a `LOCALHOST` participant still finds same-host default-range ones, but not vice-versa — hence `pixi run rviz-sim` (RViz joined to the sim's host-local graph) alongside `pixi run rviz` (default range, for same-host graphs such as a bag replay).

Non-code directories: `design/` holds the BOM (`design/BOM.md`) and CAD files (step/stl/3mf); `docs/images/` holds README photos and the logo (webp).

## Verification note

xacro generation (and therefore all URDF/config changes) can be verified on a workstation with `pixi run -- xacro install/mote_description/share/mote_description/urdf/mote.urdf.xacro`. Controller param injection can also be verified on a workstation by running ros2_control_node against the xacro output with the plugin swapped to `mock_components/GenericSystem` (plus robot_state_publisher to publish the description topic), then `ros2 param get /diff_drive_controller wheel_separation`. Actual motion requires the Pi with hardware connected.
