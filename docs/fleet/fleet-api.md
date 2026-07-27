# Fleet API — interface contract v1

The HTTP wire: enrollment, the registry, **mediated dispatch**, the audit log,
and what the dashboard needs to bootstrap. This is the second of the fleet's two
contracts — [`control-plane.md`](control-plane.md) specifies the MQTT one — and
the versioned spec [`fleet.md`](../design/fleet.md) requires M3 to publish.

| | |
|---|---|
| **Contract version** | `v1` (routes under `/v1/…`, payload `schema: 1`) |
| **Authority** | [`mote_fleet/server/fleet_server.py`](../../mote_fleet/server/fleet_server.py) + [`bundle_store.py`](../../mote_fleet/server/bundle_store.py) |
| **Kept honest by** | `test_fleet_server.py`, `test_map_registry.py`, `test_mapsync.py`; `test_e2e_fleet.py` for dispatch end to end |
| **Milestone** | M3, extended by M4 (the registry routes). Operator runbook: [`README.md`](README.md) §6–9 and §11. Measurements: [`m3-verification.md`](m3-verification.md), [`m4-verification.md`](m4-verification.md) |

## Why there are two contracts

They carry opposite directions of the same loop and have opposite requirements.

**Reads ride MQTT.** Presence, health, pose and task status are retained on the
broker, so any subscriber — `fleetctl watch`, the dashboard, a future tool —
sees the whole fleet's current state the moment it connects, with no polling and
no service in the middle. The browser speaks the same protocol as everything
else, over WebSockets.

**Writes ride HTTP.** A command has to be attributed to somebody, recorded, and
refusable. Broker ACLs can express "may publish" but not "who did", so dispatch
is a request to this API, which authorizes the operator, writes the audit row,
and only then publishes to the topic tree it would otherwise have written to
directly. **The topic tree does not change** — `fleetctl` moved to this route in
M3 and no robot noticed.

## Versioning

The same two-axis rule as the control plane. **The path carries the major
version**: a breaking change ships as `/v2/…` and both can be served while
clients migrate. **Every payload carries `schema`**, an integer tracking the body
shape within a major version; consumers must ignore fields they do not
recognise, so adding an optional field bumps nothing. Removing a field, renaming
one, or changing its type is a `v2` change.

Status codes are part of the contract: a client may switch on them.

---

## Authentication

| Route | Credential |
|---|---|
| `POST /v1/enroll` | an **enrollment token** in the body (single-use by default) |
| `POST /v1/robots/<id>/dispatch`, `GET /v1/audit` | an **operator token** as `Authorization: Bearer <token>` |
| `POST …/revisions/<rev>/promote` | an **operator token** |
| `POST …/revisions/<rev>` (map upload) | none, but the `robot_id` must be enrolled — see [the registry](#the-map-registry-m4) |
| everything else | none — see the security note below |

Operator tokens are minted on the fleet box, against the registry file, never
over the network:

```bash
pixi run -e fleet fleetctl -- operator new --name michael
pixi run -e fleet fleetctl -- operator list
pixi run -e fleet fleetctl -- operator revoke --token <token>
```

The token's **name is what the audit log records**, which is why an unnamed one
is refused. Revocation keeps the row: who *had* access is part of the record.

Bearer header only — never a query parameter, which would put the credential in
every access log between here and the browser.

**Security posture, plainly.** The read routes are unauthenticated and the
broker is anonymous, exactly as M1 left them. M3 adds a credential on the
*write* path and a record of who used it, which is the milestone's brief; it is
proportionate only while the tailnet is the boundary. M7 adds operator auth on
the read routes, per-robot broker credentials, and the Tailscale ACLs. Until
then, do not expose this port to a network the robots are not already trusted
on.

---

## Routes

```
GET  /healthz                            liveness, contract, robot count
GET  /v1/config                          what the browser needs to bootstrap
GET  /v1/robots                          the roster
GET  /v1/robots/<robot_id>               one row
POST /v1/enroll                          allocate (or return) a robot id
POST /v1/robots/<robot_id>/dispatch      authorize, audit, publish a command
GET  /v1/audit[?limit=&robot_id=]        what was dispatched, by whom
GET  /v1/maps                            basemaps this server can serve
GET  /v1/maps/<site>/<floor>/map.json    resolution + origin + size
GET  /v1/maps/<site>/<floor>/map.png     the basemap image
GET  /v1/maps/<site>/<floor>/zones.json  the floor's taught zones
GET  /v1/sites                           the registry: every floor + its canonical revision
GET  /v1/sites/<site>/floors/<floor>     every revision, validated, with provenance
POST     …/revisions/<rev>               upload a candidate revision (a robot)
GET      …/revisions/<rev>/bundle.tar.gz pull a revision (a robot)
POST     …/revisions/<rev>/promote       make it canonical (an operator)
GET  /                                   the operator UI (static files)
```

`POST /v1/enroll` is specified in
[`control-plane.md`](control-plane.md#enrollment--registry-api) and unchanged by
M3; the rest are below.

### `POST /v1/robots/<robot_id>/dispatch`

Send one task to one robot.

```json
{"schema": 1, "command": "goto kitchen", "issued_by": "night shift"}
```

| Field | Type | Notes |
|---|---|---|
| `schema` | int | `1` |
| `command` | string | the task-layer grammar, verbatim; whitespace-collapsed, ≤200 chars |
| `issued_by` | string, *optional* | free text for audit; defaults to `ui:<operator name>` |

```json
{"schema":1,"robot_id":"mote-01","id":"3e99cf44d1294ab5","command":"goto kitchen",
 "issued_at":"2026-07-26T19:00:27.412Z","issued_by":"ui:michael","audit_id":7,
 "status_topic":"mote/v1/mote-01/task/status"}
```

**`202 Accepted` means published, not accepted by the robot.** The `id` is the
correlation id the agent will echo on every status; `status_topic` is where to
watch for them. Whether the robot takes the task is answered on that topic
within milliseconds — `accepted`, or `rejected` with a reason. An API that
reported "accepted" here would be inventing an answer only the robot can give.

| Status | Meaning |
|---|---|
| `202` | published to `task/command` and recorded |
| `400` | empty command, over-long command, or an invalid robot id |
| `401` | missing, unknown, or revoked operator token |
| `404` | no such robot in the registry |
| `503` | the broker could not be reached; nothing was published |

**The command grammar is not parsed here.** `fetch <target> <drop_zone>` and
`goto <zone>` belong to the robot's task layer, which validates against *its*
zones and answers `rejected:` with a reason an operator can act on. A parser in
the fleet server would be a second grammar to keep in step, and it would reject
commands a newer robot understands.

**One in-flight command per robot** is still enforced by the agent, not here
(`control-plane.md`). Two operators dispatching at once both get `202`; the
second robot-side answer is `rejected: busy with '…'`.

### `GET /v1/audit`

```json
{"schema":1,"audit":[
 {"id":7,"stamp":"2026-07-26T19:00:27Z","actor":"michael","action":"dispatch",
  "robot_id":"mote-01","command":"goto kitchen","command_id":"3e99cf44d1294ab5",
  "result":"published","detail":"","remote":"100.64.0.7"}]}
```

Newest first, `limit` defaults to 100 (max 1000), optional `robot_id` filter.

`result` is one of `published` · `error` (the broker refused or was unreachable,
with `detail`) · `rejected` (the API refused, e.g. no such robot) ·
`unauthorized` (no usable token — the actor is `anonymous`).

**Refused attempts are recorded too.** The row is written *before* the publish
and closed with the outcome after, so a command that reached the broker can
never be missing from the log; a log with an extra "we tried" line is the
failure mode to prefer.

### `GET /v1/config`

Everything the dashboard cannot work out from its own URL.

```json
{"schema":1,"contract":"mote/v1",
 "topics":{"root":"mote/v1","presence":"presence","health":"health","pose":"pose",
           "status":"task/status"},
 "broker":{"ws_host":null,"ws_port":9001,"host":"fleet-box","port":1883},
 "foxglove_url":"foxglove://open?ds=foxglove-websocket&ds.url=ws://{robot_id}:8765",
 "maps":true}
```

`broker.ws_host` is `null` unless `--broker-ws-host` was given, and the page then
falls back to the host it was loaded from — so reaching the fleet box by MagicDNS,
by tailnet address or over localhost all work with no per-deployment build.

### `GET /v1/maps`, `/v1/maps/<site>/<floor>/map.json|map.png`

The basemap a pose is meaningful on.

```json
{"schema":1,"site":"office_world","floor":"ground","image":"map.png",
 "resolution":0.05,"origin":[-10.935,-5.958,0.0],"negate":0,"mode":"trinary",
 "occupied_thresh":0.65,"free_thresh":0.196,"width":500,"height":300,
 "image_url":"/v1/maps/office_world/ground/map.png"}
```

`width`/`height` come from the PNG header, so a client can place a robot before
the image has decoded. The world→pixel transform is
[`fleet.md` Q5](../design/fleet.md#5-live-operations-ui--adopt-foxglove-for-depth-build-a-thin-fleet-roster):

```
px = (wx - origin_x) / resolution
py = height - (wy - origin_y) / resolution      # image y is top-down
```

`404` for a site/floor with no published map; `400` for a name that is not a
plain directory label (which is also the whole path-traversal story).

**These routes kept their shape across M4 and changed their source.** The bytes
now come from the registry below rather than from whatever an operator last
rsynced onto the box, `map.json` gained `revision`, and `/v1/maps` gained
`revision` + `candidates` per floor. Adding fields bumps nothing; a client that
ignores them is unaffected. The reader is no longer hand-rolled either: it is
`mote_bringup.bundle`, the same ROS-free module the robot writes revisions with
(fleet.md Q4) — which is what makes "the server validates what the robot saved"
a shared definition rather than two that agree by convention.

A floor that was seeded by `rsync` before M4 still works: `sites()` walks the
bundle layout, not a table, so such a floor appears with no upload history and
serves normally. It cannot be *promoted* onto until its `map/` directory is a
symlink into `maps/<rev>/` — the API says so with a `409` rather than
overwriting the directory.

### `GET /v1/maps/<site>/<floor>/zones.json`

The floor's taught places, in the same map frame as the basemap, so the
dashboard can draw them and an operator can see the `goto` targets they are
about to type.

```json
{"schema":1,"site":"home","floor":"ground","frame_id":"map","zones":[
 {"name":"kitchen","x":1.0,"y":2.0,"yaw":0.0,"radius":1.5},
 {"name":"ward","x":4.0,"y":1.0,"polygon":[[3,0],[5,0],[5,2],[3,2]]}]}
```

Read from the **canonical revision's** `zones.yaml`, falling back to the
floor-level file for a bundle seeded by rsync. `404` for a floor with no taught
zones — an empty list would claim the floor has none, which is a different
statement.

---

## The map registry (M4)

The fleet server is the source of truth for sites, floors and map revisions.
The shape of it is one rule:

> **Uploading is not publishing.** A revision that arrives is a *candidate*: it
> is validated, stored, recorded, and changes nothing. An operator promotes one,
> which flips the floor's `map` symlink and publishes the retained
> [`current`](control-plane.md#current) topic every agent pulls from.

That is also the conflict answer. Two robots that map the same floor produce two
candidates, both kept, neither merged — a map frame's origin is an accident of
where SLAM started, so merging two frames would break every taught zone
coordinate (fleet.md Q4). The loser is retained for audit.

**A revision is an immutable directory, and distribution is a copy plus one
atomic flip** — the model `sites.py` already used locally, unchanged. The wire
form is a flat gzipped tar of the revision's files with the floor's `zones.yaml`
packed in beside them, because zones are coordinates in that revision's frame
and must travel with it. Packing is deterministic, so a revision always packs to
the same bytes and the digest announced on the retained topic keeps matching
what the download route serves.

### `GET /v1/sites`

```json
{"schema":1,"sites":[{"site":"home","floor":"ground","canonical":"20260727T101500",
 "candidates":["20260728T090412"],"revisions":["20260727T101500","20260728T090412"]}]}
```

### `GET /v1/sites/<site>/floors/<floor>`

Every revision the floor holds, **re-validated on read** — a revision on disk
can rot (a restore, a half-copied backup) and "promotable" is a claim about now.

```json
{"schema":1,"site":"home","floor":"ground","canonical":"20260727T101500",
 "revisions":[{"revision":"20260727T101500","canonical":true,"ok":true,
   "errors":[],"warnings":[],"uploaded_at":"2026-07-27T10:21:44Z","robot_id":"mote-01",
   "bytes":186349,"sha256":"sha256:6f1c…","zones":["kitchen","ward"],
   "map":{"image":"map.png","resolution":0.05,"origin":[-2.9,-2.9,0.0],"width":438,"height":238},
   "occupancy":{"total":104244,"free":0.899,"occupied":0.05,"unknown":0.051},
   "url":"/v1/sites/home/floors/ground/revisions/20260727T101500/bundle.tar.gz"}]}
```

### `POST /v1/sites/<site>/floors/<floor>/revisions/<rev>?robot_id=<id>`

Body: the packed revision (`application/gzip`), ≤64 MB. `pixi run publish-map`
is the robot-side caller.

| Status | Meaning |
|---|---|
| `201` | stored as a candidate; the body says under which id |
| `400` | not a readable bundle, a bad name, or no `robot_id` |
| `404` | no such robot in this fleet |
| `413` | larger than the ceiling |
| `422` | a readable bundle that is **not a usable map revision**; `errors` says why |

```json
{"schema":1,"site":"home","floor":"ground","revision":"20260728T090412",
 "canonical":"20260727T101500","promoted":false,"warnings":[],
 "url":"/v1/sites/home/floors/ground/revisions/20260728T090412/bundle.tar.gz"}
```

**The stored id may not be the proposed one.** Revision ids are per-second
timestamps, so two robots mapping one floor in the same second collide; the
second is stored as `<rev>-2` rather than overwriting the first, and the
response says so. Re-uploading byte-identical content is a retry and mints
nothing.

**Server-side validation** is `mote_bringup.bundle` — the *same* module the
robot refused to save an incomplete revision with, run again because an upload
can truncate where a local save could not. It checks: every required file
present and non-empty; `map.yaml` parses with a positive resolution, a finite
origin, an image that is a plain file name, and thresholds the right way round;
the image is a PNG whose dimensions are sane and match `map_raw.png` if that is
present (they are the same frame); the posegraph is there, or mapping can never
be continued in this frame; `meta.yaml` provenance; and **the occupancy is not
degenerate** — a revision can have every file in place and still be a uniform
grey rectangle, which is what a mapping run that never got going looks like.

**Why this route has no credential.** Everything it can do is inert: a candidate
changes no floor, is bounded in size and count, and is recorded in the audit log
against the robot that sent it. The write that *does* change something —
promote — is the operator's. M7 replaces the `robot_id` check with a per-robot
credential.

### `POST /v1/sites/<site>/floors/<floor>/revisions/<rev>/promote`

Operator token required. Body `{"schema": 1}`.

```json
{"schema":1,"site":"home","floor":"ground","revision":"20260728T090412",
 "url":"…/bundle.tar.gz","sha256":"sha256:1a2b…","bytes":186349,
 "promoted_by":"michael","warnings":[],"announced":true,"detail":"",
 "topic":"mote/v1/registry/site/home/floor/ground/current","audit_id":12}
```

| Status | Meaning |
|---|---|
| `200` | the floor is on this revision — see `announced` |
| `401` | no usable operator token (recorded as an anonymous attempt) |
| `404` | no such revision |
| `409` | the floor's `map` is a plain directory, not a published revision |
| `422` | the revision is not promotable; `errors` says why |

**`announced` is separate from success on purpose.** The symlink flip is the
fact; the retained announcement is best effort. A broker that is down must not
leave a floor half-promoted, so the flip stands, the response says the fleet was
not told, and the server re-announces every floor at startup — which is what
repairs it.

### `GET …/revisions/<rev>/bundle.tar.gz`

The packed revision, with the digest in `X-Bundle-Sha256`. An agent pulls this
after the retained announcement, checks the digest, stages the whole revision in
a temporary directory, renames it into `maps/<rev>/`, and flips the local `map`
symlink — so a half-transferred revision is never visible and nothing has to be
undone if the transfer dies.

---

## What the browser is allowed to do

The dashboard holds an operator token for this API and **no broker credential at
all that can publish**. The read path connects to the broker's WebSocket
listener with a client that implements no PUBLISH packet
([`ui/mqtt.mjs`](../../mote_fleet/server/ui/mqtt.mjs)) — the split is enforced by
omission, not by intention. M7 makes that structural on the broker side too,
with a subscribe-only credential.
