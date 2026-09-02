# Fleet control plane — interface contract v2

The wire between a robot and the fleet server: the MQTT topic tree, the payload
schemas, the mission lifecycle, and the enrollment API. This is the versioned
contract [`fleet.md`](../design/fleet.md) requires each milestone to publish —
dashboards, tooling and future agents should build against *this*, not against
`mote_fleet`'s internals.

| | |
|---|---|
| **Contract version** | `v2` (topic root `mote/v2/…`, payload `schema: 1`) |
| **Authority** | the topic tree and the telemetry payloads: [`mote_fleet/mote_fleet/protocol.py`](../../mote_fleet/mote_fleet/protocol.py). The mission and capability payloads: **the open specifications**, implemented in [`mote_bringup/mote_bringup/spec/`](../../mote_bringup/mote_bringup/spec/) |
| **Machine-readable** | telemetry: [`mote_fleet/schema/*.schema.json`](../../mote_fleet/schema/). Missions and capabilities: the specifications' own schemas, which Mote does **not** vendor |
| **Kept honest by** | `mote_fleet/test/test_protocol.py` for this document and the telemetry schemas; `mote_bringup/test/test_spec_conformance.py` validates real payloads against the specifications' schemas where a checkout of them is present |
| **Milestone** | M1, adopting mission/v0 + capability/v0. Operator runbook: [`README.md`](README.md). Measurements: [`m1-verification.md`](m1-verification.md) |

## What v2 changed, and why the root moved

v1 carried a **command string** — `fetch red_box dropoff` — and answered with a
**sentence**: `rejected: busy with 'goto kitchen'`. Both are now typed. A
command names a **capability key** and carries an **input object** validated
against a schema the robot advertises; a failure carries a **class** and a
**recoverability** a dispatcher can act on without parsing prose.

That is a change of meaning in an existing payload, which by the rule below is
exactly what a new topic root is for. The telemetry half (presence, health,
pose) and the map registry moved with it *unchanged*: a tree has one version,
not one per leaf.

**A v1 robot and a v2 fleet server do not interoperate.** Nothing translates
between them, deliberately — a shim would be a third definition of the wire.
Upgrade the agent and the server together; a robot enrolled before v2 keeps its
identity and its maps, since neither is on this tree.

## Versioning

Two things can change independently, so they are versioned separately.

**The topic root carries the major version.** A breaking change — a topic moves,
a field changes meaning, a state is removed — ships as the next root. Both trees
can be published at once while subscribers migrate, and a subscriber never has
to guess which contract a message came from.

**The mission payloads are not Mote's to version.** `mission/command`,
`mission/status` and `capabilities` carry mission/v0 and capability/v0
documents, and those specifications version independently of this tree: adding
an optional field there does not move `mote/v2`, and a `mission/v1` would. Mote
keeps no copy of their schemas, so there is nothing here to drift.

**Every payload carries `schema`.** An integer tracking the payload shape within
a major version. Consumers **must ignore fields they do not recognise**; adding
an optional field is therefore not a breaking change and does not bump anything.
A payload whose `schema` a reader does not know is rejected, not guessed at
(`protocol.check`).

Removing a field, renaming one, or changing its type is a `v2` change.

---

## Topic tree

Robot topics are `mote/v2/<robot_id>/<leaf>`. `robot_id` is a lowercase DNS label
(`[a-z0-9]([a-z0-9-]{0,30}[a-z0-9])?`) because it is simultaneously a MagicDNS
hostname, a topic level and a directory name.

| Leaf | Direction | Retained | QoS | Payload |
|---|---|---|---|---|
| `presence` | robot → fleet | **yes** (and the LWT) | 1 | [presence](#presence) |
| `health` | robot → fleet | **yes** | 1 | [health](#health) |
| `pose` | robot → fleet | **yes** | 1 | [pose](#pose) |
| `capabilities` | robot → fleet | **yes** | 1 | [capability set](#capabilities) |
| `mission/command` | fleet → robot | **no** | 1 | [mission command](#missioncommand) |
| `mission/status` | robot → fleet | **yes** | 1 | [mission status](#missionstatus) |

Two properties of that table are load-bearing:

**Everything except commands is retained.** An operator UI connecting at any
moment sees the current state of every robot immediately, with no polling and no
waiting for the next heartbeat. `mote/v2/+/health` is the roster, and
`mote/v2/+/capabilities` is what the whole fleet can be asked to do — which is
the question a dispatcher asks first and, before v2, could only answer by
sending something wrong and reading the refusal.

**`mission/command` is never retained.** A retained command would be redelivered
every time the robot reconnects, which turns a link flap into a re-dispatch. A
publisher that sets the retain flag on a command is a bug in the publisher.

QoS 1 (at-least-once) throughout: the broker may redeliver, and every consumer
here is idempotent — the state topics are snapshots, and a command is keyed by
its correlation `id` so a redelivery is recognised rather than re-run.

### The map registry's subtree (M4)

One subtree is about the fleet rather than about a robot:

| Topic | Direction | Retained | QoS | Payload |
|---|---|---|---|---|
| `mote/v2/registry/site/<site>/floor/<floor>/current` | server → robots | **yes** | 1 | [current](#current) |

**`registry` is a reserved first level.** No robot may be allocated that id: a
robot called `registry` would publish its health into the map registry's
subtree, and a consumer reading the first level as a robot id would invent a
fleet member out of a map announcement. `protocol.valid_id` refuses it and
`protocol.parse_topic` answers `None` for the subtree.

**Retained is the mechanism, not a detail.** A robot that was switched off
through an entire mapping session is handed its floor's canonical revision the
moment it reconnects — so map distribution has no polling and no
missed-update case. The subscription is `mote/v2/registry/site/+/floor/+/current`,
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
(`mote_health`), so the fleet sees exactly what the robot
sees.

| Field | Type | Notes |
|---|---|---|
| `schema` | int | `1` |
| `robot_id` | string | |
| `stamp` | string | |
| `state` | enum | `ok` \| `degraded` \| `fault` \| `stale` \| `unknown` |
| `summary` | string | one line, e.g. `DEGRADED: camera stale (3.1s > 2.0s)` |
| `subsystems` | array | `{name, state, message}`, one per watched subsystem |
| `mission` | object, *nullable* | `{id, capability, state, lane}` while a mission is in flight — a roster summary, not a status; the authority is the mission's own payload |
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
 "mission":null,"site":"home","floor":"ground","version":"ece90cc","uptime_s":48213.0,
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

### capabilities

A **capability/v0 capability set**: everything this robot can be asked to do, as
one document with one `revision`. Published retained, and *forwarded* by the
agent rather than authored by it — the task server publishes it on a latched ROS
topic, so a robot whose task server is down advertises nothing, which is true.

Mote advertises two keys, both from the specification's standard registry, so a
dispatcher that has never seen a Mote knows the input shapes:

| Key | Input | Notes |
|---|---|---|
| `goto` | `{target: <zone name>}` | `cancellable: false`, `max_duration_s: 600`, `idempotency: natural` |
| `fetch` | `{target: <zone name or object label>, destination: <zone name>}` | `cancellable: false`, `max_duration_s: 900`, `idempotency: none` |

`target` on `goto` and `destination` on `fetch` `$ref` the zone reference
schema, which is what makes the seam machine-readable: a tool reading the
document can tell *which inputs are places* and pre-check them against the zone
vocabulary at [`/v1/zones`](fleet-api.md) before sending anything.

Both declare `cancellable: false` because the task layer has no cancel — there
is a twist_mux pause lock an operator can assert, and that is a different thing.
Saying `true` would promise a message nothing handles. `mission/cancel` is
therefore a named topic with no publisher and no subscriber; it is reserved
rather than left to be invented.

The authority for the shape is
[`mote_tasks/mote_tasks/capabilities.py`](../../mote_tasks/mote_tasks/capabilities.py),
and for its rules the capability/v0 specification.

### mission/command

A **mission/v0 mission command**. The dispatcher chooses the `id`; that is the
load-bearing part, because a platform-assigned id cannot exist until the
platform has seen the command, so a dispatcher that timed out waiting for one
could only send another and hope.

| Field | Type | Notes |
|---|---|---|
| `schema` | int | `1` |
| `id` | string | correlation id, ≤64 of `[A-Za-z0-9_.:-]`. **Do not parse it.** |
| `platform_id` | string | the robot it is for; a robot ignores a command addressed elsewhere |
| `capability` | string | a key from that robot's capability set |
| `input` | object | validated against that capability's `input_schema` |
| `issued_at` | string | RFC 3339, UTC, milliseconds |
| `issued_by` | string | free text, for audit |
| `lane` | string | concurrency lane, default `default` |
| `capability_version` | string, *nullable* | pin; a different major version is rejected rather than reinterpreted |
| `parent_id` | string, *nullable* | for tracing a decomposed goal; implies no ordering |
| `deadline` | string, *nullable* | after which the mission is pointless |

```json
{"schema":1,"id":"3e99cf44d1294ab5","platform_id":"mote-01","capability":"fetch",
 "input":{"target":"red_box","destination":"dropoff"},"lane":"default",
 "capability_version":null,"parent_id":null,"deadline":null,
 "issued_at":"2026-07-26T16:15:35.960Z","issued_by":"fleetctl"}
```

### mission/status

A **mission/v0 mission status**: one transition, and a snapshot rather than a
delta — the latest status for an id is the whole truth about that mission.

| Field | Type | Notes |
|---|---|---|
| `schema` | int | `1` |
| `id` | string, *nullable* | the command's correlation id; `null` for a mission the robot started itself |
| `platform_id` | string | |
| `capability` | string | |
| `state` | enum | `dispatched` \| `accepted` \| `succeeded` \| `failed` \| `rejected`. `running`, `blocked` and `cancelled` are **optional in v0 and Mote does not emit them** — a consumer must not require them |
| `terminal` | bool | carried even though it is derivable, so a consumer stops watching without a table of final states |
| `source` | enum | `fleet` (the agent dispatched it) \| `local` (someone on the robot did) |
| `stamp` | string | |
| `lane` | string | |
| `detail` | string | human-readable; **never the only carrier of anything a caller must act on** |
| `warnings` | array | unmet *non-blocking* preconditions, on the `accepted` status |
| `failure` | object, *nullable* | required on `rejected` and `failed`, null everywhere else |
| `progress`, `result` | object, *nullable* | always null today |

The **failure** object is what replaced the `detail` sentence:

| Field | Type | Notes |
|---|---|---|
| `class` | enum | `unknown_capability` \| `invalid_input` \| `precondition` \| `busy` \| `unresolved_zone` \| `unreachable` \| `obstructed` \| `timeout` \| `hardware` \| `safety_stop` \| `internal` |
| `recoverable` | bool | whether re-dispatching the identical mission has a prospect of succeeding **without a human changing something first** |
| `detail` | string | the specific cause: the unmet precondition, the zone that would not resolve, the id of the mission holding the lane |
| `at` | enum, *nullable* | the state it failed from — the difference between "never started" and "died halfway" |
| `retry_after_s` | number, *nullable* | hint, meaningful only when `recoverable` |

`recoverable` is set **per failure**, not looked up from the class: a
`precondition` failure on localisation clears itself, one on a missing component
does not, and both arrive as `precondition`.

```json
{"schema":1,"id":"3e99cf44d1294ab5","platform_id":"mote-01","capability":"goto",
 "state":"rejected","terminal":true,"source":"fleet","lane":"default","detail":"",
 "warnings":[],"progress":null,"result":null,
 "failure":{"class":"unresolved_zone","recoverable":false,"at":"dispatched",
            "retry_after_s":null,
            "detail":"unknown_name: target 'nowhere' is not a place here; navigable zones are dropoff, home, pickup"}}
```

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
security boundary (the tailnet is that, while the broker is anonymous); it is
there because a
transfer that silently truncated would otherwise become a map, and a wrong map
is worse than no map.

```json
{"schema":1,"site":"home","floor":"ground","revision":"20260727T101500",
 "url":"/v1/sites/home/floors/ground/revisions/20260727T101500/bundle.tar.gz",
 "sha256":"sha256:6f1c…","bytes":186349,"promoted_by":"michael",
 "stamp":"2026-07-27T10:22:04.118Z"}
```

---

## Mission lifecycle

```
                         ┌─────────────┐
   command published ──▶ │ dispatched  │ ── executor rejects ──▶ rejected ────┐
                         └──────┬──────┘   (unknown_capability,               │
                                │           invalid_input, busy,              │
                                │           precondition, unresolved_zone)    │ terminal
             executor accepts   │                                             │
                                ▼                                             │
                         ┌─────────────┐ ── tree succeeds ──▶ succeeded ──────┤
                         │  accepted   │                                      │
                         └─────────────┘ ── tree fails ─────▶ failed ─────────┘
                                            (obstructed, unreachable,
                                             timeout, internal — and the
                                             agent's own timeout, below)
```

Exactly one terminal status per mission; nothing is published for that id
afterwards. `running`, `blocked` and `cancelled` are legal in the specification
and Mote emits none of them: the behaviour tree reports that it took a mission
and then that the mission ended, which is what makes this contract
implementable on an executor that reports a bare accept.

### Who enforces what

**One in-flight mission per (robot, lane)**, and from v2 the **executor** owns
that rule, not the agent. In v1 the agent had to: `task/command` was a bare
string with no correlation id, so the agent could not tell one refusal from
another and kept the robot from ever seeing two commands. A mission has an id
now, so the rule belongs to the thing that actually holds the lane — which also
sees missions issued locally on the robot, where the agent could only infer them.
A command arriving on a held lane is rejected with `failure.class: "busy"`, and
the detail names the mission holding it.

What the **agent** keeps is what is genuinely its own
(`mote_fleet/dispatch.py`):

- **deduplication** — a redelivered `id` re-publishes that mission's current
  status and does not re-execute it. QoS 1 is at-least-once, an operator may
  click twice, and a dispatcher that timed out may resend on purpose.
- **retention** — a terminal status is remembered for an hour, so a dispatcher
  that restarts can learn the outcome of what it sent. Within that window an id
  is not fresh: a redelivery of a *finished* mission's command returns the
  outcome rather than starting a second one.
- **the unanswered mission** — `dispatched` is not an acknowledgement. If
  nothing answers within `command_timeout` (default 20 s) the agent publishes
  `failed` with `failure.class: "timeout"`, `recoverable: true`. That is what a
  robot with no `task_server` running looks like, on purpose.
- **attribution** — `source`. On the robot, a command the agent forwarded and
  one a bench script published are the same message on the same topic; only the
  agent knows which ids it dispatched. A status for an id it does not know is
  republished as `source: "local"`, so an operator sees a robot that is busy
  whoever asked it to be.

Once `accepted` there is no agent-side timeout at all. The mission's own bound
is its capability's `max_duration_s`, enforced by the executor, which fails it
with class `timeout`.

### Preconditions

The executor evaluates every blocking precondition before accepting, and a
failure names which:

| Precondition | Holds when | On failure |
|---|---|---|
| `localized` | a `map`→`base_link` transform newer than 5 s | `precondition`, **recoverable** — localisation comes back on its own |
| `zone_known` | the named zone resolves on this robot | `unresolved_zone`, carrying zone/v0's own reason (`unknown_name`, `not_navigable`, …) |

An unmet **non-blocking** precondition is not a refusal: it is reported in
`warnings` on the `accepted` status, so a mission that started degraded is
visible rather than merely slow to fail. `fetch` declares one — the detector the
label branch needs may legitimately not be running.

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
 "contract":"mote/v2"}
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

## Security posture (and what is still owed)

Stated plainly so it is not mistaken for a finished story:

- **The broker is anonymous.** Any client that can reach it may publish or
  subscribe anywhere in the tree. WireGuard is the authentication boundary;
  nothing here is reachable from the public internet.
- **The fleet API is not.** Every `/v1` route needs an operator token, checked by
  one gate in front of routing ([`fleet-api.md`](fleet-api.md)); `/healthz`, the
  static UI, enrollment and the two robot-facing map routes are the carve-outs,
  each for a stated reason.
- **The tailnet has rules.** `mote_bringup/tailscale/policy.hujson` is the
  committed access policy: operators reach the fleet box and a robot's SSH and
  Foxglove ports, robots reach the fleet server and their inference box, and no
  robot reaches another. Its `tests` block asserts that last one, and Tailscale
  refuses to save a policy that fails it.
- **Dispatch is mediated, as of M3.** M1's `fleetctl` published straight to the
  broker; now it and the dashboard both POST to `/v1/robots/<id>/dispatch`,
  which authorizes an operator token and writes an audit row before publishing.
  As this section promised, **the topic tree did not change** — only who
  publishes to it. The browser holds no broker credential that can publish;
  making that structural on the broker side, with a subscribe-only credential,
  waits on the broker having credentials at all.

What is still owed is the broker half: per-robot credentials (username =
`robot_id`, publish confined to its own prefix), a subscribe-only operator
credential for the browser, and the ACL that keeps the three principals apart.
Until then, do not put the broker on a network the robots are not already
trusted on.
