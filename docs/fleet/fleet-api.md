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
GET  /v1/maps/<site>/<floor>/zones.json  the floor's zone *binding* (has coordinates)
GET  /v1/zones                           every floor's zone *vocabulary* (no coordinates)
GET  /v1/zones/<site>/<floor>            one floor's, as a zone/v0 document
GET  /v1/sites                           the registry: every floor + its canonical revision
GET  /v1/sites/<site>/floors/<floor>     every revision, validated, with provenance
POST     …/revisions/<rev>               upload a candidate revision (a robot)
GET      …/revisions/<rev>/bundle.tar.gz pull a revision (a robot)
GET      …/revisions/<rev>/map.json      that revision's own transform + size
GET      …/revisions/<rev>/map.png       that revision's own image
GET      …/revisions/<rev>/zones.json    that revision's own zone binding
POST     …/revisions/<rev>/promote       make it canonical (an operator)
POST /v1/sites/<site>/floors/<floor>/zones  edited zones of a revision ->
                                         a new candidate (an operator)
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

The floor's taught places **with their coordinates**, in the same map frame as
the basemap, so the dashboard can draw them and an operator can see the `goto`
targets they are about to type. This is the zone **binding**: it is served
beside the basemap, to a client that already has the basemap, and it is not
what a dispatcher should be given — see [the vocabulary](#the-zone-vocabulary)
below.

```json
{"schema":1,"site":"home","floor":"ground","frame_id":"map","zones":[
 {"name":"kitchen","x":1.0,"y":2.0,"yaw":0.0,"radius":1.5,"kind":"room",
  "display_name":"The Kitchen","aliases":["galley"],"navigable":true},
 {"name":"ward","x":4.0,"y":1.0,"polygon":[[3,0],[5,0],[5,2],[3,2]]}]}
```

Read from the **canonical revision's** `zones.yaml`, falling back to the
floor-level file for a bundle seeded by rsync. `404` for a floor with no taught
zones — an empty list would claim the floor has none, which is a different
statement — and `404` for a floor with no published map, because a coordinate
with no map frame to be in is not an answer.

---

## The zone vocabulary

**Names are shared; coordinates are not.** This is the half of a zone that is
portable between robots, served so that the question a dispatcher most needs to
ask — *what places can I name?* — has an answer in the API rather than out of
band. The shape is [zone/v0](https://spec.augereai.com/zone/v0/).

A zone's pose is a coordinate in one robot's map frame, and that frame's origin
is an accident of where its SLAM session happened to start. `(2.0, 3.5)` on
`mote-01` is a different physical point from `(2.0, 3.5)` on `mote-02`, and
there is no fleet-level transform that fixes it — the two are independent
estimates of the same building, drifting apart. The name, by contrast, is true
for both. So the vocabulary travels and the binding does not, and the split is
in the route: everything under `/v1/maps` is bound to a basemap, everything
under `/v1/zones` is bound to nothing.

A caller that must never be handed a map can be given `/v1/zones` and only
`/v1/zones`.

### `GET /v1/zones/<site>/<floor>`

```json
{"schema":1,"site":"home","floor":"ground","revision":4,"zones":[
 {"name":"kitchen","display_name":"The Kitchen","aliases":["galley"],
  "kind":"room","navigable":true,"parent":null,"tags":[],"description":""},
 {"name":"sluice","display_name":"","aliases":[],
  "kind":"keepout","navigable":false,"parent":null,"tags":[],"description":""}],
 "problems":[]}
```

| field | |
|---|---|
| `name` | The shared token. `^[a-z][a-z0-9_]*$`, unique within a **floor**, not within a site — two floors may each have a `reception`. |
| `display_name` | What an operator sees. Free text. Empty means "use the name". |
| `aliases` | The other things people call it, for natural-language dispatch. Matched case-insensitively and whitespace-normalised. |
| `kind` | One of `area room corridor doorway threshold elevator stair dock charger pickup dropoff staging home keepout slow`. `area` is the default and claims nothing. |
| `navigable` | Whether it is a legal destination. Always `false` for `keepout` and `slow`. |
| `parent` | An enclosing zone on the same floor, or `null`. |
| `revision` | Bumped every time a zone is taught, so a binding can record which vocabulary it was built against. |
| `problems` | Empty when the vocabulary is well-formed; see below. |

There are **no coordinates, no `frame_id` and no map reference**, by
construction: the payload is built from the fields a vocabulary may carry
rather than filtered of the ones it may not, so a geometry key added to
`zones.yaml` later cannot leak into it. `test_zone_vocabulary.py` asserts this
by walking the whole payload for geometry-shaped keys rather than checking the
ones it happens to know about.

Unlike the binding, this is **not** gated on a published map. A floor someone
has named but no robot has mapped still answers here — names are a fact about
the building and do not wait on a SLAM session. `404` only when the floor has
no `zones.yaml` at all.

### `GET /v1/zones`

Every floor's vocabulary in one call, for a dispatcher bootstrapping a fleet:
`{"schema":1,"vocabularies":[…]}`, each element the document above. Floors with
no zones yet are omitted rather than listed empty.

### `problems`

Reported, not enforced. Two things can be wrong with a vocabulary while the map
around it is perfectly good, so the server says so and still serves it — a
floor's basemap must not stop being served over a duplicated alias:

- **an ambiguous query** — two zones answering to one name or alias. A resolver
  must not pick between them, so the name is simply unusable until an operator
  fixes it. The robot's own loader *does* refuse such a file, because it would
  otherwise resolve `goto` by dictionary order.
- **a name a dispatcher cannot type** — e.g. a zone taught as `Café`. It is
  served verbatim rather than silently slugified to `cafe`: inventing a name is
  a rename nobody asked for. The fix is an operator's, and is to move the label
  into `display_name`.

A file that has no coherent reading at all — an unknown `kind`, a `keepout`
marked `navigable: true`, `aliases` that are not a list — is refused at the
parse instead, by the same shared validator (`mote_bringup/bundle.py`) that
`save-map` runs locally. Those are not ambiguities to report; they are
contradictions, and honouring the last one written would make the flag mean
whatever was typed most recently.

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

### `GET …/revisions/<rev>/map.json`, `…/map.png`, `…/zones.json`

The same three questions `/v1/maps/<site>/<floor>/…` answers, asked of a
revision that is **not** the floor's canonical one. Uploading is not publishing,
so an operator has a decision to make; these are what lets them see what they
are deciding about, and they are what the dashboard's review pane reads.

`map.json` is `/v1/maps`' payload with `revision` naming this revision and
`image_url` pointing at *this* route's `map.png`:

```json
{"schema":1,"site":"home","floor":"ground","revision":"20260802T145731",
 "resolution":0.05,"origin":[-2.927,-2.934,0.0],"width":438,"height":238,
 "image":"map.png",
 "image_url":"/v1/sites/home/floors/ground/revisions/20260802T145731/map.png"}
```

That URL is load-bearing. `/v1/maps/<site>/<floor>/map.png` serves whatever is
*published*, so a client that took the transform from here and the pixels from
there would draw the map the operator already has under the candidate's label —
which looks entirely convincing and is the exact failure this route removes.

`zones.json` carries one extra field over the canonical route:

```json
{"schema":1,"site":"home","floor":"ground","revision":"20260802T145731",
 "source":"floor","frame_id":"map","zones":[…]}
```

`source` is `revision` when the revision carries its own `zones.yaml` and
`floor` when it inherits the floor's. The difference matters and the coordinates
cannot express it: inherited zones were taught in a *previous* SLAM session's
frame, so they draw perfectly over this map and are wrong by however far the two
origins differ.

Unlike `read_zones` on the canonical route, this is **not gated on there being a
published map** — the review that matters most is the first candidate on a floor
with nothing published at all. That does not loosen the vocabulary/binding
split: these are still coordinates, still served under a path bound to a
basemap, and still never over `/v1/zones`. Naming a revision is naming a map
frame.

All three are reads, so like every other read route they take no operator token;
M7 changes that for all of them at once.

### `POST /v1/sites/<site>/floors/<floor>/zones`

Operator token required. The edited zone set, and the revision it was edited
against:

```json
{"schema": 1, "revision": "20260802T145731",
 "zones": {"kitchen": {"x": 1.0, "y": -3.0, "yaw": 0.0, "kind": "room",
                       "display_name": "The Kitchen", "aliases": ["galley"],
                       "polygon": [[0.0,-4.0],[2.0,-4.0],[2.0,-2.0],[0.0,-2.0]]}}}
```

`zones` is the `zones.yaml` shape — keyed by name, both halves of zone/v0 in one
entry — and it **replaces** that revision's set rather than patching it. An entry
may echo its key as `name`, which is dropped: the file keys by name and carries
no second copy.

```json
{"schema":1,"site":"home","floor":"ground","revision":"20260812T211029",
 "derived_from":"20260802T145731","promoted":false,"warnings":[],"audit_id":31}
```

| Status | Meaning |
|---|---|
| `201` | stored as a candidate — `revision` is the new one, `derived_from` the edited one |
| `400` | `zones` is not a mapping, or a name is not a directory name |
| `401` | no usable operator token (recorded as an anonymous attempt) |
| `404` | no such floor, or no such `revision` on it |
| `409` | no `revision` given and the floor has nothing published to edit |
| `422` | the edited set is not readable as a `zones.yaml`; `errors` says why |

**Editing is a derivation, not a mutation.** The named revision's map bytes are
re-packed with the submitted zones in place of its own and accepted as an
ordinary candidate — validated by the same code as a robot's upload, listed by
the floor route, promotable through the route above. Nothing already stored is
written: a promoted revision's bytes back a digest the fleet has been told, and a
candidate is immutable for the same reason an id is never reused. The response
carries `promoted: false` because that stays a separate, audited decision.

**`revision` is what makes an unpromoted map editable**, and that is the point
rather than a convenience. A fresh build arrives carrying `zone_01`..`zone_07`
from `segment-map`; without it the only thing an edit could derive from was the
canonical revision, so renaming those placeholders meant promoting them first —
publishing a map *because* it was wrong. It also keeps the coordinates in the
frame they were drawn in: the operator is looking at that revision's own map.
Omitted, the canonical revision is edited, which is the same thing for a floor
whose published map is what is on screen.

**The bar is the source's, not the upload's.** A revision with no posegraph is
one mapping cannot be continued from — an error for a robot's upload, where the
session can be re-run, and a *warning* on a stored revision, which navigates
perfectly and which `promote` will accept. A derivation is therefore validated
the way `promote` validates: holding an edit to a stricter bar than the revision
it derives from would put an `edit zones` button beside a `promotable` verdict
that could only ever fail.

Two things this route deliberately does not do. It does not slugify or otherwise
repair a name — a zone a dispatcher cannot type is stored with a warning, as
everywhere else in the vocabulary (`problems` are reported, not enforced; the
*robot's* loader is what refuses one). And it never publishes: the audit row
names the actor, the floor and the revision edited, and the floor's `map`
symlink is untouched.

---

## What the browser is allowed to do

The dashboard holds an operator token for this API and **no broker credential at
all that can publish**. The read path connects to the broker's WebSocket
listener with a client that implements no PUBLISH packet
([`ui/mqtt.mjs`](../../mote_fleet/server/ui/mqtt.mjs)) — the split is enforced by
omission, not by intention. M7 makes that structural on the broker side too,
with a subscribe-only credential.

**Reviewing a candidate is all GETs.** The review pane reads a revision's
`map.json`, `map.png` and `zones.json`; the two writes beside them are the
`promote` M4 already had and the zone edit above, both operator-authorized and
both audited. The edit is the only write in the fleet that produces a revision,
and it produces an inert one: a candidate nobody is running, on a floor that has
not moved.
