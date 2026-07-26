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
| **Interface contract** | [`docs/fleet/control-plane.md`](../docs/fleet/control-plane.md) — the versioned topic tree, payload schemas, and enrollment API |
| **Operator runbook** | [`docs/fleet/README.md`](../docs/fleet/README.md) §6–8 |
| **What was measured** | [`docs/fleet/m1-verification.md`](../docs/fleet/m1-verification.md) |
| **Design** | [`docs/design/fleet.md`](../docs/design/fleet.md) — M1, and Q1/Q2/Q3 |

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
pixi run -e fleet fleet-broker                     # mosquitto
pixi run fleet-server -- --broker-host fleet-box   # enrollment + registry
pixi run fleetctl -- dispatch mote-01 goto kitchen
```

| Script | |
|---|---|
| [`server/fleet_server.py`](server/fleet_server.py) | `POST /v1/enroll`, `GET /v1/robots`, `GET /healthz` — stdlib `http.server` |
| [`server/registry.py`](server/registry.py) | the SQLite row store: robots, tokens, transactional id allocation |
| [`server/fleetctl.py`](server/fleetctl.py) | operator CLI: tokens, roster, dispatch, watch |
| [`server/mosquitto.conf`](server/mosquitto.conf), [`broker.sh`](server/broker.sh) | the broker, and where its state goes |

The server imports `mote_fleet.protocol` from the source tree by path (the
`depth_server.py` pattern) and nothing else — no ROS, no framework, no ament.
Server state lives in `$MOTE_FLEET_HOME` (default `~/.mote-fleet`).

`http.server` rather than a web framework is a floor, not an aspiration: five
routes, no templating, no auth story yet, and one fewer dependency to solve on
whatever the fleet box turns out to be. M3 puts the dispatch API and the
operator UI on top, and that is where a framework earns its keep.

## Tests

```bash
pixi run test                 # colcon: everything except the broker tests
pixi run -e dev test-fleet    # + the real-broker end-to-end run
```

Three tiers, so the same files give full coverage wherever they run:

- **contract** (`test_protocol.py`) — the code, the JSON Schema files and the
  doc's field tables are checked against each other, so a payload change that
  nobody described fails here rather than in a dashboard later.
- **bridge** (`test_dispatch.py`, `test_agent.py`) — the single-in-flight rule
  and the full agent against an injected fake MQTT client, so CI covers it on
  both architectures without a broker.
- **end to end** (`test_e2e_fleet.py`) — a real mosquitto, the real enrollment
  endpoint, the `enroll` CLI, a real paho client, and the actual `mote_tasks`
  behaviour tree driving a mock Nav2. Skips where there is no broker.
