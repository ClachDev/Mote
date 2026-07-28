# Fleet control plane — interface contract v1

The wire between a robot and the fleet server: the MQTT topic tree, the payload
schemas, the task state machine, and the enrollment API. This is the versioned
contract [`fleet.md`](../design/fleet.md) requires each milestone to publish —
dashboards, tooling and future agents should build against *this*, not against
`mote_fleet`'s internals.

| | |
|---|---|
| **Contract version** | `v1` (topic root `mote/v1/…`, payload `schema: 1`) |
| **Authority** | [`mote_fleet/mote_fleet/protocol.py`](../../mote_fleet/mote_fleet/protocol.py) |
| **Machine-readable** | [`mote_fleet/schema/*.schema.json`](../../mote_fleet/schema/) (JSON Schema 2020-12) |
| **Kept honest by** | `mote_fleet/test/test_protocol.py` — this document, the code and the schema files fail CI if they disagree |
| **Milestone** | M1. Operator runbook: [`README.md`](README.md). Measurements: [`m1-verification.md`](m1-verification.md) |

## Versioning

Two things can change independently, so they are versioned separately.

**The topic root carries the major version.** A breaking change — a topic moves,
a field changes meaning, a state is removed — ships as `mote/v2/…`. Both trees
can be published at once while subscribers migrate, and a subscriber never has
to guess which contract a message came from.

**Every payload carries `schema`.** An integer tracking the payload shape within
a major version. Consumers **must ignore fields they do not recognise**; adding
an optional field is therefore not a breaking change and does not bump anything.
A payload whose `schema` a reader does not know is rejected, not guessed at
(`protocol.check`).

Removing a field, renaming one, or changing its type is a `v2` change.

---

## Topic tree

Robot topics are `mote/v1/<robot_id>/<leaf>`. `robot_id` is a lowercase DNS label
(`[a-z0-9]([a-z0-9-]{0,30}[a-z0-9])?`) because it is simultaneously a MagicDNS
hostname, a topic level and a directory name.

| Leaf | Direction | Retained | QoS | Payload |
|---|---|---|---|---|
| `presence` | robot → fleet | **yes** (and the LWT) | 1 | [presence](#presence) |
| `health` | robot → fleet | **yes** | 1 | [health](#health) |
| `pose` | robot → fleet | **yes** | 1 | [pose](#pose) |
| `task/command` | fleet → robot | **no** | 1 | [command](#command) |
| `task/status` | robot → fleet | **yes** | 1 | [status](#status) |

Two properties of that table are load-bearing:

**Everything except commands is retained.** An operator UI connecting at any
moment sees the current state of every robot immediately, with no polling and no
waiting for the next heartbeat. `mote/v1/+/health` is the roster.

**`task/command` is never retained.** A retained command would be redelivered
every time the robot reconnects, which turns a link flap into a re-dispatch. A
publisher that sets the retain flag on a command is a bug in the publisher.

QoS 1 (at-least-once) throughout: the broker may redeliver, and every consumer
here is idempotent — the state topics are snapshots, and a command is keyed by
its correlation `id` so a redelivery is recognised rather than re-run.

### The map registry's subtree (M4)

One subtree is about the fleet rather than about a robot:

| Topic | Direction | Retained | QoS | Payload |
|---|---|---|---|---|
| `mote/v1/registry/site/<site>/floor/<floor>/current` | server → robots | **yes** | 1 | [current](#current) |

**`registry` is a reserved first level.** No robot may be allocated that id: a
robot called `registry` would publish its health into the map registry's
subtree, and a consumer reading the first level as a robot id would invent a
fleet member out of a map announcement. `protocol.valid_id` refuses it and
`protocol.parse_topic` answers `None` for the subtree.

**Retained is the mechanism, not a detail.** A robot that was switched off
through an entire mapping session is handed its floor's canonical revision the
moment it reconnects — so map distribution has no polling and no
missed-update case. The subscription is `mote/v1/registry/site/+/floor/+/current`,
and an agent acts only on the floor it is on plus floors it already holds
(`mapsync.wants`): the registry is fleet-wide, one robot is not.

---

## Payloads

Every payload is a JSON object with `schema: 1`. Fields marked *nullable* are
always present and may be `null` — a consumer never has to test for absence.

### presence

Whether the agent is connected. Published retained on connect, and registered as
the MQTT **Last Will**, so a robot that loses power is marked offline by the
broker within the keepalive rather than whenever somebody notices the heartbeats
stopped.

| Field | Type | Notes |
|---|---|---|
| `schema` | int | `1` |
| `robot_id` | string | |
| `online` | bool | |
| `stamp` | string | RFC 3339, UTC, milliseconds |
| `version` | string | present when online: the software the agent is running |
| `reason` | string | present when offline: `last will` (the broker published it) or `agent stopped` (the agent did) |

```json
{"schema":1,"robot_id":"mote-01","online":true,"stamp":"2026-07-26T16:15:23.104Z","version":"ece90cc"}
```

### health

The robot's own roll-up, forwarded rather than recomputed: `state`, `summary`
and `subsystems` come from the health monitor's `/diagnostics_agg`
(`mote_bringup/health_monitor.py`), so the fleet sees exactly what the robot
sees.

| Field | Type | Notes |
|---|---|---|
| `schema` | int | `1` |
| `robot_id` | string | |
| `stamp` | string | |
| `state` | enum | `ok` \| `degraded` \| `fault` \| `stale` \| `unknown` |
| `summary` | string | one line, e.g. `DEGRADED: camera stale (3.1s > 2.0s)` |
| `subsystems` | array | `{name, state, message}`, one per watched subsystem |
| `task` | object, *nullable* | `{id, command, state}` while a task is in flight |
| `site` | string, *nullable* | the **active map bundle**, not identity's entitlement |
| `floor` | string, *nullable* | |
| `version` | string, *nullable* | |
| `uptime_s` | number, *nullable* | host uptime, from `/proc/uptime` |
| `battery` | object, *nullable* | **reserved; always null** |
| `map` | object, *nullable* | `{site, floor, revision}` — the map revision this robot is *running* |

`state: unknown` is a real answer, not a gap: the health monitor is a separate
service, and an agent whose diagnostics are missing or stale says so rather than
claiming a robot is fine.

`battery` is in the contract but unmeasurable — the power bank exposes no
telemetry (`fleet.md`). A field a dashboard renders as "unknown" from day one is
better than one bolted on later.

`map` is *reported*, not assumed. The registry says what a floor should be on;
only the robot can say what it is actually running, and the difference between
the two is the only way to see a robot that has not picked up a new map.
`revision` is `null` for a floor with no saved map. Added in M4 as an optional
field, which bumps no version — consumers ignore what they do not recognise.

```json
{"schema":1,"robot_id":"mote-01","stamp":"2026-07-26T16:15:24.001Z","state":"ok",
 "summary":"OK","subsystems":[{"name":"lidar","state":"ok","message":"ok"}],
 "task":null,"site":"home","floor":"ground","version":"ece90cc","uptime_s":48213.0,
 "battery":null,"map":{"site":"home","floor":"ground","revision":"20260727T101500"}}
```

### pose

| Field | Type | Notes |
|---|---|---|
| `schema` | int | `1` |
| `robot_id` | string | |
| `stamp` | string | |
| `frame_id` | string | normally `map` |
| `x`, `y` | number | metres, 3 dp |
| `yaw` | number | radians, 4 dp |
| `site`, `floor` | string, *nullable* | |

The site and floor travel with the coordinate because a map frame's origin is an
accident of where SLAM started: a pose from one floor means nothing on another,
so a consumer must know which basemap to draw it on. Converting to basemap
pixels needs that floor's `map.yaml` (`resolution`, `origin`) — the transform is
in [`fleet.md` Q5](../design/fleet.md#5-live-operations-ui--adopt-foxglove-for-depth-build-a-thin-fleet-roster).

Pose is **not published until the robot is localised** (there is no
`map`→`base_link` transform before that), so a robot may legitimately have a
retained health but no pose. The retained pose is a *last known* position —
check `stamp` before trusting it.

### command

| Field | Type | Notes |
|---|---|---|
| `schema` | int | `1` |
| `id` | string | correlation id; every status carries it back |
| `command` | string | the task-layer grammar, verbatim |
| `issued_at` | string | |
| `issued_by` | string | free text, for audit |

`command` is the existing `mote_tasks` grammar, unchanged and untranslated:
`fetch <target> <drop_zone>` or `goto <zone>` (`mote_tasks/task_server.py`). The
fleet layer deliberately adds no second grammar.

```json
{"schema":1,"id":"3e99cf44d1294ab5","command":"fetch lab kitchen",
 "issued_at":"2026-07-26T16:15:35.960Z","issued_by":"fleetctl"}
```

### status

| Field | Type | Notes |
|---|---|---|
| `schema` | int | `1` |
| `robot_id` | string | |
| `id` | string, *nullable* | the command's correlation id; `null` for a locally-issued task |
| `command` | string | |
| `state` | enum | `dispatched` \| `accepted` \| `rejected` \| `succeeded` \| `failed` |
| `detail` | string | why — the rejection reason, the failing tree node, the timeout |
| `source` | enum | `fleet` (this agent dispatched it) \| `local` (someone on the robot did) |
| `stamp` | string | |
| `terminal` | bool | true for `rejected`/`succeeded`/`failed` |

### current

The canonical map revision for one floor, published retained by the registry
when an operator promotes a candidate — and re-published for every floor when
the fleet server starts, so a promotion made while the broker was down, or a
broker that lost its retained state with its volume, repairs itself.

| Field | Type | Notes |
|---|---|---|
| `schema` | int | `1` |
| `site` | string | |
| `floor` | string | one floor is one SLAM session, i.e. one map frame |
| `revision` | string | the immutable revision id; also its directory name at both ends |
| `url` | string | path on the fleet server to fetch the packed revision from |
| `sha256` | string | `sha256:<hex>` of the packed bundle |
| `bytes` | int | packed size, so a robot can decide before downloading |
| `promoted_by` | string | the operator; empty for a startup re-announcement |
| `stamp` | string | |

`url` is **relative** so the same retained message stays correct however the
fleet box is reached — MagicDNS name, tailnet address or localhost.

`sha256` is checked by the puller before anything is staged. It is not a
security boundary (the tailnet is that until M7); it is there because a
transfer that silently truncated would otherwise become a map, and a wrong map
is worse than no map.

```json
{"schema":1,"site":"home","floor":"ground","revision":"20260727T101500",
 "url":"/v1/sites/home/floors/ground/revisions/20260727T101500/bundle.tar.gz",
 "sha256":"sha256:6f1c…","bytes":186349,"promoted_by":"michael",
 "stamp":"2026-07-27T10:22:04.118Z"}
```

---

## Task state machine

```
                         ┌─────────────┐
   command published ──▶ │ dispatched  │ ── task server rejects ──▶ rejected ─┐
                         └──────┬──────┘                                      │
                                │ task server accepts                         │ terminal
                                ▼                                             │
                         ┌─────────────┐ ── tree succeeds ──▶ succeeded ──────┤
                         │  accepted   │                                      │
                         └─────────────┘ ── tree fails ─────▶ failed ─────────┘
                                                                (also: no verdict
                                                                 within 20s)
```

**One in-flight command per robot.** This is the rule that makes retries safe
(`fleet.md` Q1), and it is enforced by the agent, not by the task layer:

- a command arriving while another is in flight is **rejected by the agent**,
  with `detail` naming the running command. The robot never sees two.
- a **redelivery of the same `id`** re-publishes the current status instead of
  re-dispatching. Publishing the same command twice is safe; publishing it with
  a fresh id twice is two commands.
- the first terminal status frees the slot.

**Why the agent owns this.** `task/command` is a bare `std_msgs/String` and
`task/status` answers with bare strings; the task layer has no notion of a
request id, and giving it one would mean changing a working interface for the
fleet's convenience. So the correlation id lives upstream of the ROS seam.
Attribution then has two handles: the single-in-flight rule, *and* the fact that
every status line echoes the command text it is about
(`accepted: goto kitchen`, `rejected: 'goto nowhere' (…)`,
`rejected: busy with 'fetch red_box dropoff'`). A status that matches neither is
reported as `source: "local"` — an operator should see a robot that is busy,
whoever asked it to be.

**`dispatched` is not an acknowledgement from the robot's task layer.** It means
the agent forwarded the command onto the ROS graph. If nothing answers within
`command_timeout` (default 20 s) the agent publishes `failed` with
`detail: "no verdict from the task server within 20s"` and frees the slot — that
is what a robot with no `task_server` running looks like. Once `accepted`, there
is no timeout at all: missions take as long as they take.

---

## Enrollment + registry API

HTTP/JSON on the fleet server (default port 8080). This is the endpoint that
supersedes M0's operator-set id: the server owns the id space.

| Route | Purpose |
|---|---|
| `GET /healthz` | liveness, contract version, robot count |
| `GET /v1/robots` | the roster |
| `GET /v1/robots/<robot_id>` | one row |
| `POST /v1/enroll` | allocate (or return) a robot id |

The map registry's routes — upload a candidate, pull a revision, promote one —
are the same server and are specified in [`fleet-api.md`](fleet-api.md), with
the retained `current` topic above as their only MQTT half.

### `POST /v1/enroll`

```json
{"schema":1,"token":"<enrollment token>","fingerprint":"machine_id:d25bff05…",
 "facts":{"machine_id":"…","serial":"…","mac":"…","hostname":"…","model":"Raspberry Pi 5"},
 "name":"Scout","site":"home","robot_id":""}
```

`name`, `site` and `robot_id` are optional. `robot_id` **requests** a specific
id — that is the M0 upgrade path: a robot with an operator-set id offers it, and
the server records it rather than renumbering the fleet.

```json
{"schema":1,"robot_id":"mote-01","name":"Scout","site":"home","created":true,
 "enrolled_at":"2026-07-26T16:12:46Z","broker":{"host":"fleet-box","port":1883},
 "contract":"mote/v1"}
```

`201` for a new robot, `200` for one that was already enrolled. The robot writes
`$MOTE_HOME/robot.yaml` (identity) and `$MOTE_HOME/fleet.yaml` (server +
broker) from that answer — it learns where its broker is from the same exchange
that gave it its id, so the two cannot disagree.

| Status | Meaning |
|---|---|
| `400` | malformed body, missing fingerprint, or an invalid requested id |
| `401` | missing, unknown, or already-used token |
| `409` | requested id belongs to another machine, or this machine is already enrolled under a different id |

**Enrollment is idempotent on the fingerprint.** The registry keys its row on a
stable hardware identifier (SoC serial, else `/etc/machine-id`, else MAC), so
re-running `enroll` — after a wiped `~/.mote`, after a failed first attempt —
returns the same id rather than minting a second fleet member.

**Allocation is transactional.** Ids are derived from the rows already present,
so concurrent enrollments run inside a `BEGIN IMMEDIATE` transaction; eight
robots enrolling at once get eight distinct ids.

---

## Security posture (and what M7 changes)

M1 is proportionate to the M0 substrate and no further. Stated plainly so it is
not mistaken for a finished story:

- **The broker is anonymous.** Any client that can reach it may publish or
  subscribe anywhere in the tree. WireGuard is the authentication boundary;
  nothing here is reachable from the public internet.
- **The fleet API has no auth** on its read routes. Enrollment tokens and, since
  M3, operator tokens are the only credentials in the system.
- **Dispatch is mediated, as of M3.** M1's `fleetctl` published straight to the
  broker; now it and the dashboard both POST to `/v1/robots/<id>/dispatch`,
  which authorizes an operator token and writes an audit row before publishing
  ([`fleet-api.md`](fleet-api.md)). As this section promised, **the topic tree
  did not change** — only who publishes to it. The browser holds no broker
  credential that can publish; making that structural on the broker side, with a
  subscribe-only credential, is still M7's.

M7 adds per-robot broker credentials (username = `robot_id`, publish confined to
its own prefix), operator auth on the API, and the Tailscale ACLs that stop
robots reaching each other. Until then: do not put the broker or the API on a
network the robots are not already trusted on.
