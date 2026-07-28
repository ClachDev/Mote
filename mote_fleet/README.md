# mote_fleet

The fleet control plane: the agent that runs **on** a robot, and the enrollment,
map-registry and dispatch server that runs **off** it. One package for both halves, because
the thing that matters most is that they agree about the wire — and the wire is
a single module, [`protocol.py`](mote_fleet/protocol.py), that both import.

Same arrangement as `mote_perception`: a torch-free node on the robot, a server
off-board, and one shared wire module (`depth_wire.py`) so the two cannot drift.
Here the robot-side half is the ament package and the off-board half is
[`server/`](server/) — plain scripts, because the fleet box runs them without
installing anything.

The design doc calls the robot-side process **`mote_agent`**, and that is what it
is called everywhere it is visible: the node is `mote_agent`, the service is
`mote-agent.service`, the console script is `agent`.

| | |
|---|---|
| **Interface contracts** | [`control-plane.md`](../docs/fleet/control-plane.md) — the MQTT topic tree and payload schemas · [`fleet-api.md`](../docs/fleet/fleet-api.md) — the HTTP routes, dispatch and audit |
| **Operator runbook** | [`docs/fleet/README.md`](../docs/fleet/README.md) §6–9, and §11 for maps |
| **Deploying the server** | [`server-pipelines.md`](../docs/fleet/server-pipelines.md) — the container stack, gated updates, backup/restore |
| **What was measured** | [`m1-verification.md`](../docs/fleet/m1-verification.md) · [`m3-verification.md`](../docs/fleet/m3-verification.md) · [`m4-verification.md`](../docs/fleet/m4-verification.md) · [`ms-verification.md`](../docs/fleet/ms-verification.md) |
| **Design** | [`docs/design/fleet.md`](../docs/design/fleet.md) — M1, M3, M4 and Ms, and Q1/Q2/Q3/Q4/Q5 |

## On the robot

```bash
pixi run enroll -- --server http://fleet-box:8080 --token <token>
pixi run agent
pixi run publish-map            # after save-map: offer the map to the registry
```

| Module | |
|---|---|
| [`agent.py`](mote_fleet/agent.py) | the node: presence/health/pose up, one task at a time down |
| [`dispatch.py`](mote_fleet/dispatch.py) | the single-in-flight rule; ROS-free and MQTT-free, so the awkward cases are plain function calls |
| [`protocol.py`](mote_fleet/protocol.py) | the wire contract — topics, payloads, states. Stdlib only |
| [`enroll.py`](mote_fleet/enroll.py) | the `enroll` CLI |
| [`facts.py`](mote_fleet/facts.py) | hardware facts and the fingerprint enrollment is idempotent on |
| [`fleet_config.py`](mote_fleet/fleet_config.py) | `$MOTE_HOME/fleet.yaml` — where this robot's fleet lives |
| [`mapsync.py`](mote_fleet/mapsync.py) | the map registry's robot side: pull the canonical revision, publish a candidate. ROS-free |
| [`publish.py`](mote_fleet/publish.py) | the `publish-map` CLI |

**The agent is a bridge and a reporter, never in the control loop.** Nav2, SLAM
and the behaviour tree run locally and keep running with the fleet server
unplugged; a dropped link means the agent stops reporting, not that the robot
stops. It is also the robot's *sole* egress — nothing off-box joins the ROS
graph — which is what makes the fleet-scale DDS story a non-problem rather than
a design (`fleet.md` Q1/Q3).

## Off the robot

```bash
pixi run fleet-broker                                      # mosquitto + WebSockets
pixi run -e fleet fleet-server -- --broker-host fleet-box  # API + dashboard
pixi run -e fleet fleetctl -- dispatch mote-01 goto kitchen
```

| Script | |
|---|---|
| [`server/fleet_server.py`](server/fleet_server.py) | the fleet API: enrollment, roster, dispatch, audit, basemaps, and the UI — stdlib `http.server` |
| [`server/registry.py`](server/registry.py) | the SQLite row store: robots, enrollment tokens, operators, the audit log, transactional id allocation |
| [`server/bundle_store.py`](server/bundle_store.py) | the map registry's byte store: candidate revisions, validation on the way in, the atomic flip that publishes one |
| [`server/fleetctl.py`](server/fleetctl.py) | operator CLI: tokens, roster, dispatch, audit, watch |
| [`server/ui/`](server/ui/) | the dashboard: `index.html`, `app.mjs`, `map.mjs` (basemap + the Q5 transform), `mqtt.mjs` (a subscribe-only MQTT client) |
| [`server/mosquitto.conf`](server/mosquitto.conf), [`broker.sh`](server/broker.sh) | the broker, its WebSocket listener, and where its state goes |
| [`deploy/`](deploy/) | the deployed shape: an image for the API+UI, a compose file that runs it beside the broker, and `fleet-deploy.sh` (gated update, rollback, backup, restore) |

The server imports `mote_fleet.protocol` and `mote_bringup.bundle` from the
source tree by path (the `depth_server.py` pattern) and nothing else — no ROS,
no framework, no ament. `protocol` is the wire the robot and the server agree
on, and `bundle` is the *bundle format* they agree on, so the server validates
an uploaded map revision with the same code that wrote it rather than a second
implementation that agrees by convention (`fleet.md` Q4). `protocol` is
stdlib-only; `bundle` additionally imports PyYAML, which the image installs
beside paho — reading these files with anything other than the library that
writes them is exactly the second implementation the rule exists to avoid. Server state lives in `$MOTE_FLEET_HOME` (default
`~/.mote-fleet`), with the site bundles under `sites/`.

`http.server` rather than a web framework stays a floor, not an aspiration: a
dozen routes, no templating, no ORM, and one fewer dependency to solve on
whatever the fleet box turns out to be. M3 was where a framework was expected to
earn its keep and it did not — the UI is static files and there is nothing to
render server-side. The same goes for the browser: no bundler, no npm, and no
vendored MQTT library, because what the read path needs is five packet types of
a published wire format and a minified blob nobody can review is a worse
dependency than 200 lines that are tested.

**Dispatch is mediated by the server, and only by the server.** Every write to
`task/command` — from the dashboard or from `fleetctl` — is a POST that
authorizes an operator token and writes an audit row first. The read path is
unchanged and goes straight to the broker.

**Uploading a map is not publishing it.** A revision a robot uploads is a
candidate that changes nothing; an operator promotes one, which flips the
floor's `map` symlink and publishes the retained `…/current` topic agents pull
from. Two robots that map one floor leave two candidates, never a merge.

## Tests

```bash
pixi run test                 # colcon: everything except the broker tests
pixi run -e dev test-fleet    # + the real-broker end-to-end run
```

Four tiers, so the same files give full coverage wherever they run:

- **contract** (`test_protocol.py`, `test_fleet_server.py`, `test_map_registry.py`,
  `test_registry.py`) — the code, the JSON Schema files and the doc's field
  tables checked against each other, and every HTTP route over a real socket with
  an injected publisher, so a payload or status-code change that nobody described
  fails here rather than in a dashboard later. `api_harness.py` is the live
  server the last two share.
- **bridge** (`test_dispatch.py`, `test_agent.py`, `test_mapsync.py`) — the
  single-in-flight rule and the full agent against an injected fake MQTT client,
  so CI covers it on both architectures without a broker; and the robot's map
  staging and symlink flip against a real fleet server, with no ROS at all.
- **browser** (`test_ui.py` → `ui_test.mjs`) — the MQTT packet codec and the
  world→pixel transform under node, against the same `.mjs` files the browser
  loads. Skips where there is no node. `browser_check.mjs` is the other half —
  a real headless browser against a running stack, which needs more than CI has,
  so it is an operator's tool rather than a test.
- **end to end** (`test_e2e_fleet.py`, `test_e2e_map_registry.py`,
  `test_fleet_outage.py`) — a real
  mosquitto, the real fleet server, the `enroll` CLI, a real paho client, and
  the actual `mote_tasks` behaviour tree driving a mock Nav2; including a
  dispatch that goes out through the API. The second kills the broker under a
  live agent: the robot finishes its task anyway and the agent reconnects by
  itself, which is the claim the fleet server's update pipeline is allowed to
  have downtime on. The third publishes a map, promotes it with `fleetctl`, and
  starts a *second* robot's agent afterwards — so the only thing that can tell it
  about the map is a retained message handed over on connect. All skip where
  there is no broker, and share `fleet_harness.py`.
