# mote_fleet

The fleet control plane: the agent that runs **on** a robot, and the enrollment
and registry server that runs **off** it. One package for both halves, because
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
| **Operator runbook** | [`docs/fleet/README.md`](../docs/fleet/README.md) §6–9 |
| **What was measured** | [`m1-verification.md`](../docs/fleet/m1-verification.md) · [`m3-verification.md`](../docs/fleet/m3-verification.md) |
| **Design** | [`docs/design/fleet.md`](../docs/design/fleet.md) — M1 and M3, and Q1/Q2/Q3/Q5 |

## On the robot

```bash
pixi run enroll -- --server http://fleet-box:8080 --token <token>
pixi run agent
```

| Module | |
|---|---|
| [`agent.py`](mote_fleet/agent.py) | the node: presence/health/pose up, one task at a time down |
| [`dispatch.py`](mote_fleet/dispatch.py) | the single-in-flight rule; ROS-free and MQTT-free, so the awkward cases are plain function calls |
| [`protocol.py`](mote_fleet/protocol.py) | the wire contract — topics, payloads, states. Stdlib only |
| [`enroll.py`](mote_fleet/enroll.py) | the `enroll` CLI |
| [`facts.py`](mote_fleet/facts.py) | hardware facts and the fingerprint enrollment is idempotent on |
| [`fleet_config.py`](mote_fleet/fleet_config.py) | `$MOTE_HOME/fleet.yaml` — where this robot's fleet lives |

**The agent is a bridge and a reporter, never in the control loop.** Nav2, SLAM
and the behaviour tree run locally and keep running with the fleet server
unplugged; a dropped link means the agent stops reporting, not that the robot
stops. It is also the robot's *sole* egress — nothing off-box joins the ROS
graph — which is what makes the fleet-scale DDS story a non-problem rather than
a design (`fleet.md` Q1/Q3).

## Off the robot

```bash
pixi run -e fleet fleet-broker-ws                          # mosquitto + WebSockets
pixi run -e fleet fleet-server -- --broker-host fleet-box  # API + dashboard
pixi run -e fleet fleetctl -- dispatch mote-01 goto kitchen
```

| Script | |
|---|---|
| [`server/fleet_server.py`](server/fleet_server.py) | the fleet API: enrollment, roster, dispatch, audit, basemaps, and the UI — stdlib `http.server` |
| [`server/registry.py`](server/registry.py) | the SQLite row store: robots, enrollment tokens, operators, broker credentials, the audit log, transactional id allocation |
| [`server/credentials.py`](server/credentials.py) | who may connect to the broker and say what — mosquitto's `$7$` hash, and the `password_file`/`acl_file` generated from the registry (M7) |
| [`server/fleetctl.py`](server/fleetctl.py) | operator CLI: tokens, operators, broker sync, roster, dispatch, audit, watch |
| [`server/ui/`](server/ui/) | the dashboard: `index.html`, `app.mjs`, `map.mjs` (basemap + the Q5 transform), `mqtt.mjs` (a subscribe-only MQTT client) |
| [`server/mosquitto.conf`](server/mosquitto.conf), [`broker.sh`](server/broker.sh) | the broker, its WebSocket listener, its generated credentials, and where its state goes |

The server imports `mote_fleet.protocol` from the source tree by path (the
`depth_server.py` pattern) and nothing else — no ROS, no framework, no ament.
Server state lives in `$MOTE_FLEET_HOME` (default `~/.mote-fleet`).

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

## Tests

```bash
pixi run test                 # colcon: everything except the broker tests
pixi run -e dev test-fleet    # + the real-broker end-to-end run
```

Five tiers, so the same files give full coverage wherever they run:

- **contract** (`test_protocol.py`, `test_fleet_server.py`, `test_registry.py`,
  `test_credentials.py`)
  — the code, the JSON Schema files and the doc's field tables checked against
  each other, and every HTTP route over a real socket with an injected
  publisher, so a payload or status-code change that nobody described fails here
  rather than in a dashboard later.
- **bridge** (`test_dispatch.py`, `test_agent.py`) — the single-in-flight rule
  and the full agent against an injected fake MQTT client, so CI covers it on
  both architectures without a broker.
- **browser** (`test_ui.py` → `ui_test.mjs`) — the MQTT packet codec and the
  world→pixel transform under node, against the same `.mjs` files the browser
  loads. Skips where there is no node. `browser_check.mjs` is the other half —
  a real headless browser against a running stack, which needs more than CI has,
  so it is an operator's tool rather than a test.
- **authorization** (`test_broker_acl.py`) — a real mosquitto reading the real
  generated `password_file` and `acl_file`, asked to do the things M7 says it
  must refuse. Asserts on *delivery*, never on a return code: mosquitto denies
  silently, granting a forbidden subscription at SUBACK and simply never
  delivering, so a suite checking return codes would pass with no ACL at all.
  Skips where there is no broker.
- **end to end** (`test_e2e_fleet.py`) — a real mosquitto, the real fleet
  server, the `enroll` CLI, a real paho client, and the actual `mote_tasks`
  behaviour tree driving a mock Nav2; including a dispatch that goes out through
  the API. Since M7 the broker it runs against is **authenticated and ACL'd,
  starting from empty credential files**, so the chain only completes if
  enrollment issues a credential and the reload puts it in force. Skips where
  there is no broker.
