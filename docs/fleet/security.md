# Fleet security — the authorization contract

Who may connect to what, what each of them may say, and how a credential is
issued, rotated and revoked. This is the third of the fleet's contracts —
[`control-plane.md`](control-plane.md) specifies the MQTT wire and
[`fleet-api.md`](fleet-api.md) the HTTP one; this specifies the **rules applied
to both**.

| | |
|---|---|
| **Contract version** | `v1` |
| **Authority** | [`mote_fleet/server/credentials.py`](../../mote_fleet/server/credentials.py) (policy), [`mote_bringup/tailscale/policy.hujson`](../../mote_bringup/tailscale/policy.hujson) (network) |
| **Kept honest by** | `mote_fleet/test/test_broker_acl.py` (a real broker), `test_credentials.py`, `test_fleet_server.py`, and the policy file's own `tests` block |
| **Milestone** | M7. Measurements: [`m7-verification.md`](m7-verification.md) |

## What changed, and why it is worth the words

Through M3 the fleet had **one credential on one path**: an operator token on
`POST /v1/robots/<id>/dispatch`. Everything else — the roster, the basemaps, the
broker's address, and the whole MQTT topic tree — was open to anything that
could reach the port. The justification was the tailnet: WireGuard authenticates
every peer, and nothing is exposed to the internet.

That justification is true and it is not enough, for one reason: **it makes
every robot as trusted as the fleet server.** A robot is a small computer that
drives around a building, is physically reachable, and runs the largest
dependency tree in the system. If one is compromised, a single boundary means
the attacker inherits the whole fleet — reads every robot's commands, forges
every robot's health, and dispatches to all of them.

M7 does not replace the tailnet. It stops the tailnet being the *only* thing
between a compromised robot and the rest of the fleet.

---

## The principals

Four kinds of thing hold a credential. Their namespaces are **disjoint by
construction**, not by a runtime check: a `robot_id` is a lowercase DNS label
(`protocol.ID_RE`), so it can never contain an underscore, and the other two
usernames always do.

| Principal | Broker username | HTTP credential |
|---|---|---|
| robot | its `robot_id` — `mote-01` | an enrollment token, once |
| operator | `op_<slug>_<4 hex>` | an operator token (bearer) |
| fleet server | `fleet_server` | — (it *is* the server) |
| anyone else | — | — |

### What each may do on the broker

Generated into `$MOTE_FLEET_HOME/broker/acl` from the registry. `allow_anonymous
false` is the other half: a client with no username never reaches the ACL.

| | publish | subscribe |
|---|---|---|
| **robot `mote-01`** | `mote/v1/mote-01/{presence,health,pose,task/status}` | `mote/v1/mote-01/task/command` |
| **operator** | *nothing, anywhere* | `mote/v1/+/{presence,health,pose,task/status}` |
| **fleet server** | `mote/v1/+/task/command` | `mote/v1/#` |

Three consequences worth stating outright:

- **A robot cannot read another robot's commands, or forge another robot's
  health.** That is the milestone's acceptance criterion, and
  `test_broker_acl.py` asserts it against a real mosquitto.
- **An operator cannot publish a command.** Through M3 this was a property of
  *our* browser client (`ui/mqtt.mjs` implements no PUBLISH packet). It is now
  also a rule of the broker's, so it holds for a hand-rolled client, `curl`, or
  anything else an operator's credential is pasted into. Dispatch goes through
  the API, where it is authorized and audited, because that is the only place it
  can be *attributed*.
- **An operator cannot read `task/command` either.** "Who dispatched what" is a
  question the audit log answers with a name attached; the broker cannot
  attribute a message to a person, so reading commands off it would be evidence
  that looks authoritative and is not.

### What each may do over HTTP

| Route | Credential |
|---|---|
| `GET /healthz` | none |
| `GET /` and the static UI | none |
| `POST /v1/enroll` | an enrollment token, in the body |
| **everything else under `/v1`** | an operator token, `Authorization: Bearer` |

Two routes stay open on purpose. `/healthz` because a liveness probe that needs
a secret is a liveness probe nobody wires up, and it discloses only that a fleet
server is running. The **static UI** because the page has to load before it can
ask for a token — it is public code and contains no fleet data until it has one.

The gate lives in one place (`do_GET`, in front of every `/v1` path) rather than
in each handler, so a route added later is authenticated by default and has to
opt *out* somewhere a reviewer looks.

---

## Credential lifecycle

### Robots — issued at enrollment, rotated by re-enrolling

`POST /v1/enroll` answers with the broker credential alongside the identity:

```json
{"schema":1,"robot_id":"mote-01","broker":{"host":"fleet-box","port":1883,
 "username":"mote-01","password":"…"}}
```

The robot writes it to `$MOTE_HOME/fleet.yaml`, mode `0600`. That is the **only
time the plaintext is ever sent**; the registry keeps a hash. So:

- **Rotation is `pixi run enroll`.** Enrollment is idempotent on the hardware
  fingerprint, so re-running it returns the same identity with a *new* password.
  There is no second mechanism to build, document, or forget.
- **A lost password cannot be looked up**, only replaced. If a robot's
  `~/.mote` is wiped, enrol it again.
- **An update cannot clobber it**, because `MOTE_HOME` is outside the package —
  the same property that protects identity, maps and calibration (M0).

### Operators — one act, two credentials, one revocation

`fleetctl operator new --name <you>` mints the HTTP token *and* the
subscribe-only broker login together. The browser fetches the broker half from
`/v1/config` using the HTTP half, so an operator only ever handles one secret.

`fleetctl operator revoke --token <token>` closes both, because the broker's
password file is *regenerated from the rows* — a revoked operator is absent from
the query and therefore absent from the file. The row itself is kept: who *had*
access is part of the record.

### The fleet server — generated once, kept

Stored in the registry's `settings` table. Regenerating it per restart would
lock the server out of its own broker for the window between starting and
reloading.

---

## How the broker learns about a credential

The `password_file` and `acl_file` are **generated, never hand-edited** — a
projection of the registry, rewritten whole:

```
registry.db  ──(credentials.render_*)──>  $MOTE_FLEET_HOME/broker/{passwd,acl}
                                                      │
                                          broker.sh reload  ──SIGHUP──> mosquitto
```

Regenerated and reloaded on every enrollment (by the fleet server) and on every
operator change (by `fleetctl`). `fleetctl broker sync` does it on demand, for
the two cases where nothing else would: a broker restarted with empty files, and
a registry restored from a backup.

**A failed reload is reported, never fatal.** The files are correct on disk
either way, and a broker started afterwards reads them; what must not happen is
a silent success while a robot is being refused.

Both files are written `0600` via a temporary file that is chmod-ed *before* the
rename, so a secret is never briefly world-readable. So is the registry, which
holds enrollment tokens, operator tokens and operator broker passwords in the
clear. **`$MOTE_FLEET_HOME` is a secret store** — back it up accordingly.

---

## The network boundary

[`mote_bringup/tailscale/policy.hujson`](../../mote_bringup/tailscale/policy.hujson)
is the source of truth for the tailnet policy; the admin console is a copy of
it. Paste it at `login.tailscale.com/admin/acls/file`.

It grants exactly what something in this repo dials, and it **denies robot to
robot on every port** — there is no rule granting `tag:robot` access to
`tag:robot`, because in v1 there is no robot-to-robot anything. That absence is
checked by the policy's own `tests` block, which Tailscale evaluates on save and
refuses to store a policy that fails.

The rule that matters most for something *other* than the fleet: robots reach
`tag:inference:5601,5602`, and nothing else does. The inference wire is
unauthenticated by design (`depth_wire.py`), so the network is the only thing
deciding who may speak it.

---

## Packages

Every dependency is pinned by sha256 in the committed `pixi.lock` (2274 hashes
at the time of writing), and provisioning runs `pixi install --locked`, which
**aborts** if the lockfile is not up to date with the manifest rather than
solving something new. A robot therefore installs the exact dependency set that
was tested; a silent re-solve on a machine nobody is watching is the thing this
forbids.

**Signing is not in place**, and honestly cannot be until there is something to
sign: the prefix.dev `mote` channel does not ship a robot package yet (that is
M5). The trust root today is: the lockfile's hashes, HTTPS to the channel, and
the tailnet. Signature verification is M5's to add, and the design doc's
verification ledger still carries it as **(verify)**.

---

## What M7 does *not* do

Stated so the posture is not read as more than it is.

- **No mTLS on the broker.** Username/password over the WireGuard tunnel, which
  is already end-to-end encrypted between peers. Client certificates would add a
  CA to operate and rotate for a fleet whose transport is already authenticated.
  The seam is unchanged: mosquitto reads `cafile`/`certfile` instead of
  `password_file`, and the ACL is keyed on the certificate CN rather than the
  username.
- **No expiry on operator tokens.** They are revocable but not
  self-expiring, and the browser keeps one in `localStorage`. Real sessions —
  OIDC or GitHub, per fleet.md Q7 — are what replaces this, and they replace the
  *minting*, not the checking: the API's gate does not change shape.
- **No Foxglove tokens**, because `foxglove_bridge` is M2 and does not exist
  yet. What M7 contributes is the tailnet rule that already scopes port 8765 to
  operators; the bridge's own token lands with the bridge.
- **No per-customer isolation.** One tailnet, one broker, one registry — Regime
  C in the design doc, designed for and not built.
- **No secret at rest encryption.** `$MOTE_FLEET_HOME` and `$MOTE_HOME` are
  `0600` files on a disk you trust.

---

## If something is refused

| Symptom | Cause | Fix |
|---|---|---|
| agent logs `broker refused this agent (Not authorized)` | the broker has no credential for this robot | `pixi run enroll` on the robot; check the server reloaded the broker |
| every robot refused at once, right after a restart | broker started before the fleet server ever wrote the files | `pixi run -e fleet fleetctl -- broker sync` |
| dashboard shows `broker refused: bad username or password` | the operator was revoked, or the broker has not reloaded | mint a new operator, or `fleetctl broker sync` |
| `fleetctl` says `401` on `robots`/`audit` | read routes need a token since M7 | `export MOTE_FLEET_TOKEN=<operator token>` |
| a robot publishes and nothing arrives | it published outside its own prefix — mosquitto drops it **silently** | check the topic against the ACL table above |
| `--advertise-tags` fails, "tags are invalid or not permitted" | the tag has no owner in the tailnet policy | apply `policy.hujson` first |
