# Fleet API — interface contract v1

The HTTP wire: enrollment, the registry, **mediated dispatch**, the audit log,
and what the dashboard needs to bootstrap. One of the fleet's three contracts —
[`control-plane.md`](control-plane.md) specifies the MQTT wire and
[`security.md`](security.md) the authorization rules applied to both — and the
versioned spec [`fleet.md`](../design/fleet.md) requires M3 to publish.

| | |
|---|---|
| **Contract version** | `v1` (routes under `/v1/…`, payload `schema: 1`) |
| **Authority** | [`mote_fleet/server/fleet_server.py`](../../mote_fleet/server/fleet_server.py) |
| **Kept honest by** | `mote_fleet/test/test_fleet_server.py`, and `test_e2e_fleet.py` for the dispatch path end to end |
| **Milestone** | M3, with authorization from M7. Operator runbook: [`README.md`](README.md) §6–9. Measurements: [`m3-verification.md`](m3-verification.md), [`m7-verification.md`](m7-verification.md) |

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
| `GET /healthz` | none — a liveness probe that needs a secret is one nobody wires up |
| `GET /` and the static UI | none — the page must load before it can ask for a token |
| `POST /v1/enroll` | an **enrollment token** in the body (single-use by default) |
| **everything else under `/v1`** | an **operator token** as `Authorization: Bearer <token>` |

**Since M7 the read routes are authorized too**, not just dispatch — M3 left the
roster, the basemaps and the broker's address readable by anything that could
reach the port. The check sits in one gate in front of every `/v1` path rather
than in each handler, so a route added later is authenticated by default; a
`404` is never returned to an unauthenticated caller, because that would
disclose which routes are real.

Operator tokens are minted on the fleet box, against the registry file, never
over the network:

```bash
pixi run -e fleet fleetctl -- operator new --name michael
pixi run -e fleet fleetctl -- operator list
pixi run -e fleet fleetctl -- operator revoke --token <token>
```

The token's **name is what the audit log records**, which is why an unnamed one
is refused. Revocation keeps the row: who *had* access is part of the record —
and it also withdraws that operator's **broker** credential, because the two are
minted together and the broker's password file is regenerated from these rows
([`security.md`](security.md)).

Bearer header only — never a query parameter, which would put the credential in
every access log between here and the browser.

**Security posture, plainly.** Every route that discloses anything about the
fleet requires an operator; the broker requires a per-principal credential; and
the tailnet policy denies robot-to-robot traffic. What is still absent — token
expiry, mTLS, package signing — is listed with reasons in
[`security.md`](security.md#what-m7-does-not-do). The tailnet remains the outer
boundary: this port should still not be published to the internet.

---

## Routes

```
GET  /healthz                            liveness, contract, robot count  [open]
GET  /v1/config                          what the browser needs to bootstrap
GET  /v1/robots                          the roster
GET  /v1/robots/<robot_id>               one row
POST /v1/enroll                          allocate (or return) a robot id
POST /v1/robots/<robot_id>/dispatch      authorize, audit, publish a command
GET  /v1/audit[?limit=&robot_id=]        what was dispatched, by whom
GET  /v1/maps                            basemaps this server can serve
GET  /v1/maps/<site>/<floor>/map.json    resolution + origin + size
GET  /v1/maps/<site>/<floor>/map.png     the basemap image
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

Since M7 the `broker` object also carries **the calling operator's own**
subscribe-only MQTT credential:

```json
"broker": {"ws_host": null, "ws_port": 9001, "host": "fleet-box", "port": 1883,
           "username": "op_michael_3f9a", "password": "…"}
```

That is the reason this route needs a token: it hands out a credential, and the
one it hands out is the caller's. Two operators get two different logins, and a
revoked operator's next page load has nothing to connect with. The credential
grants `mote/v1/+/{presence,health,pose,task/status}` and **no write rule at
all** ([`security.md`](security.md)).

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

**This is a provisional source, not the registry.** The server reads **site
bundles exactly as `sites.py` writes them** — `<maps-dir>/<site>/floors/<floor>/map/`,
published symlink and all — from `--maps-dir` (default `$MOTE_FLEET_HOME/sites`),
which today an operator seeds with an `rsync` from a robot. **M4** makes the
fleet server the canonical registry with server-side validation and revision
promotion; when it does, these two routes keep their shape and change where the
bytes come from. That is the seam, and it is why the UI reads a bundle rather
than a bespoke map format.

---

## What the browser is allowed to do

The dashboard holds an operator token for this API and **no broker credential
that can publish**. That is now true twice over:

- its MQTT client implements no PUBLISH packet at all
  ([`ui/mqtt.mjs`](../../mote_fleet/server/ui/mqtt.mjs)) — enforced by omission;
- and since M7 the broker credential it is given has no write rule in the
  broker's ACL — enforced by the broker, so it holds for `curl`, a hand-rolled
  client, or anywhere else that credential is pasted.

The second is what makes the split structural rather than a property of our own
code, which is what M3 said it owed. Both credentials arrive together, from this
route and its token, and die together when the operator is revoked.
