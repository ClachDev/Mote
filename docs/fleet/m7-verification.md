# M7 verification ledger

What was measured for the security hardening, how, and what is still unverified.
The interface it verifies is [`security.md`](security.md); the two wires it
constrains are [`control-plane.md`](control-plane.md) and
[`fleet-api.md`](fleet-api.md), neither of which changed shape.

The milestone's acceptance criteria are **"a device off the tailnet reaches
nothing"** and **"a robot cannot read another robot's command topic."** §1
answers the second against a real broker; §6 explains why the first is asserted
by the Tailscale policy rather than measured here.

## 1. A robot cannot read another robot's command topic — **confirmed**

Against a real mosquitto 2.0.20, reading the real generated `password_file` and
`acl_file`, with two robots enrolled through the real HTTP endpoint. The whole
run is `mote_fleet/test/test_broker_acl.py` (13 tests); this is the same thing
driven by hand through the shipped `broker.sh` and `fleet_server.py`:

```console
=== 8. THE CRITERION: mote-01 cannot read mote-02's command topic
    connacks: mote-01=Success mote-02=Success server=Success operator=Success
    mote-02 (addressed) received: ['goto kitchen']
    mote-01 (eavesdropper)      : []
    operator                    : []
    operator saw on health      : ['genuine']   (FORGED absent)
    anonymous connack           : Not authorized
```

Five separate properties in that block, each of which was open before M7:

| | before | now |
|---|---|---|
| anonymous client | full read/write of the tree | `Not authorized` at CONNACK |
| `mote-01` reading `mote-02`'s commands | delivered | nothing delivered |
| `mote-01` publishing as `mote-02` | accepted | dropped; the operator sees only `genuine` |
| operator publishing a command | accepted | dropped |
| operator reading a command | delivered | nothing delivered |

### The finding that shapes every test above: **denial is silent**

mosquitto does not report an ACL denial to the client. A publish to a forbidden
topic is accepted at the socket and discarded; a subscribe to a forbidden filter
is **granted at SUBACK** and simply never delivers:

```
mote-01 SUBACK for mote/v1/mote-02/task/command: [1]     # granted QoS 1
mote-01 messages received:                       []      # ...and nothing arrives
```

So a test that asserted on a return code would pass against a broker with **no
ACL file at all**. Everything in `test_broker_acl.py` therefore asserts on
*delivery*, and one test (`test_the_denial_is_silent_…`) exists purely to pin
that measurement in place. It is also the first thing to check when a robot
"publishes and nothing happens": compare the topic against the ACL table in
[`security.md`](security.md), because nothing will have logged an error.

## 2. The credential files reach a running broker — **confirmed, via SIGHUP**

The ordering problem: a robot is issued a credential at enrollment, and the
broker has never heard of it. Measured with `broker.sh` started *first*, empty
files and all — which is exactly how a fleet box comes up:

```console
=== 1. broker starts before the server has ever run
      created an empty …/broker/passwd — nothing can connect until the fleet
      server writes it (start fleet-server, or run 'fleetctl broker sync')
      Opening ipv4 listen socket on port 43963.
      passwd bytes: 68   mode: 600

=== 2. nothing can connect yet (fail closed)
    Connection error: Connection Refused: not authorised.

=== 3. fleet server starts, generates credentials, reloads the broker
    mote-fleet on http://127.0.0.1:36211 … credentials=…/broker
    passwd now: 1 line(s)
```

and from the broker's own log, mid-run:

```
1785141364: Client auto-59A75FD4… disconnected, not authorised.
1785141364: Reloading config.
1785141365: New client connected … (p2, c1, k60, u'mote-01').
```

**An empty password file admits nobody**, which is the right state for a broker
whose fleet server has never run — as opposed to a broker that refuses to
*start*, which reads like a broken deployment. The bootstrap is therefore: start
the broker, start the server, and the server fills in and reloads it.

`fleetctl broker sync` covers the two cases nothing else would — a broker
restarted against a stale directory, and a registry restored from backup:

```console
=== 7. fleetctl broker show
    password_file …/broker/passwd
    acl_file      …/broker/acl
    principals    2 robots, 1 operators, 1 server
      fleet_server   mote-01   mote-02   op_michael_6e6c
```

## 3. Every API route is authorized — **confirmed**

```console
=== 5. the API refuses an anonymous read
    /v1/robots     401 an operator token is required (Authorization: Bearer)
    /v1/config     401 an operator token is required (Authorization: Bearer)
    /v1/maps       401 an operator token is required (Authorization: Bearer)
    /v1/audit      401 an operator token is required (Authorization: Bearer)
    /healthz       200
```

`/healthz` open is deliberate and is the only `/v1`-adjacent route that is; the
static UI is the other exception, because the page must load before it can ask
for a token. Parametrised over all seven read routes in
`test_fleet_server.py`, three ways each — no token, an unknown token, a revoked
token.

One choice worth recording: **an unauthenticated caller never gets a 404.** The
gate runs before routing, so `/v1/nothing` answers `401` rather than disclosing
which routes exist.

## 4. The whole suite — **212 tests, 0 failures**

```console
$ pixi run -e dev test-fleet
212 passed in 86.77s
```

(One of those 212 is `test_ui.py`, which runs the whole of `ui_test.mjs` under
node as a single pytest case — 16 node subtests, four of them M7's.)

That includes the four tiers from M3 plus M7's two new files, and — the part
that matters — **`test_e2e_fleet.py` now runs against an authenticated,
ACL-enforcing broker**. Its fixture starts mosquitto with `allow_anonymous
false` and *empty* credential files, so the enrollment → issue → reload → agent
connects → dispatch → status chain only completes if every link of M7 works. If
the credential plumbing broke, those four tests would fail rather than quietly
falling back to an anonymous connection.

The 13 broker-ACL tests and the real-broker e2e skip without mosquitto, so
`pixi run test` (the robot environment) still runs everything else.

## 5. The dashboard, in a real browser — **confirmed, including the revocation**

Driven with Playwright against the shipped stack: the container broker
(`eclipse-mosquitto:2`, 2.1.2) reading the generated credential files, the real
fleet server, two enrolled robots.

**Signed out**, which is a state that did not exist before M7:

```
operator:      "read-only — paste an operator token"
broker-state:  "not connected — no operator token"
roster:        "Paste an operator token to see the fleet. Mint one on the fleet
                box: fleetctl operator new --name <you>"
```

**After pasting the token**, with no page reload:

```
operator:      "operator token accepted"
broker-state:  "broker connected"
contract:      "mote/v1"
roster:        mote-01 · Scout   mote-02 · Rover
```

and from the broker's own log, which is what proves the WebSocket connection was
*authenticated* rather than merely accepted:

```
New client connected from 127.0.0.1:46886 as mote-ui-14f78c6b
    (p4, c1, k30, u'op_michael_4d54')
```

**Then the operator was revoked**, and one act closed both paths — the HTTP one
on the next request and the MQTT one *mid-session*, without the page doing
anything:

```
1785143413: Client mote-ui-14f78c6b disconnected.
1785143414: Client mote-ui-14f78c6b disconnected: not authorised.

GET /v1/robots  ->  401 {"error": "unknown or revoked operator token"}
broker-state:       "broker offline: connection closed"
```

That is the property the whole credential design is for, observed end to end.

### Two things this run found

- **`hidden` was not hiding anything.** `.dispatch { display: flex }` outranks
  the UA's `[hidden] { display: none }`, so the dispatch form and the Foxglove
  link were on screen whenever the code believed they were hidden — a defect
  since M3, invisible until M7 gave the page a signed-out state that shows a
  dispatch box you cannot use. Fixed with an explicit `[hidden]` rule in
  `style.css`.
- **A false positive worth recording.** The first browser run reported "broker
  connected" against a broker that was *not* the one under test: the workstation
  already had an M3-era mosquitto on 9001, and the page connects to
  `location.hostname:<ws_port>`. The measurement above uses a container broker on
  a non-default port, confirmed by its own connection log. If you are testing
  this on a box that already runs a fleet, check *which* broker answered before
  believing a green light.

## 6. The tailnet half — **asserted by the policy, not measured here**

"A device off the tailnet reaches nothing" is a property of
[`policy.hujson`](../../mote_bringup/tailscale/policy.hujson), which is applied
in a browser and enforced by Tailscale's coordination plane. Two things make
that better than an untested claim:

- **Tailscale evaluates the policy's own `tests` block on save** and refuses to
  store a policy that fails one. The block asserts `tag:robot` is denied
  `tag:robot:22`, `:8765` and `:1883` — the acceptance criterion, checked by the
  thing that enforces it.
- WireGuard denies by default and nothing in this repo publishes a port to the
  internet, which is the M0 property this milestone inherits rather than adds.

**Not yet applied to the live tailnet.** The policy file is committed and
reviewed; pasting it into the admin console is a one-time operator action, and
the M0 default (allow-all between devices) stands until then. Worth doing
alongside the next robot provisioning so the tag rules and a real enrollment are
exercised together.

## 7. Cost

| | measured |
|---|---|
| password hash (PBKDF2-SHA512, 101 iterations) | **0.034 ms** |
| ACL file, 10 robots + 2 operators | **3640 bytes, 108 lines** |
| regenerate + reload, per enrollment | one file write pair + one `kill -HUP` |

Nothing here is a scaling concern at any fleet size this design targets. The ACL
grows ~9 lines per robot, so a hundred robots is a ~30 KB file mosquitto reads
once per SIGHUP.

**On the iteration count.** 101 is mosquitto's own default and is low by
password-hashing standards — deliberately not raised, because it is not what the
security rests on: these are 24-byte `secrets.token_urlsafe` passwords (~192
bits of entropy), never human-chosen, so an offline attack on the hash is
infeasible regardless of the KDF cost. Raising it would slow every broker
connection to defend against a threat that does not apply. The count is encoded
*in* each hash, so it can be raised later without invalidating existing entries.

## 8. Interoperability checks

- **Our `$7$` hash is mosquitto's.** `test_credentials.py` generates a hash with
  the real `mosquitto_passwd` and verifies it with our code, so the
  reimplementation cannot drift from the broker that has to read it. (Format
  confirmed against mosquitto 2.0.20: `$7$101$<12-byte salt b64>$<64-byte key
  b64>`, PBKDF2-HMAC-SHA512.)
- **The browser's CONNECT is well-formed.** `ui_test.mjs` asserts the username
  and password flags and payload ordering under node, against the same file the
  browser loads. It also asserts the module still exports no `encodePublish` —
  if that ever appears, the "enforced by omission" half of the split is gone and
  the test is what should stop it.
- **An M3 registry upgrades in place.** `test_registry.py` builds an M1/M3-era
  SQLite file by hand, opens it with the M7 `Registry`, and checks the rows
  survive and the new columns appear — `CREATE TABLE IF NOT EXISTS` cannot alter
  an existing table, so without the migration every enrollment would fail on a
  missing column.

## 9. Not verified here

- **Nothing has run on real hardware.** No robot has been re-enrolled against an
  M7 server, and `mote-01`'s agent is still using an M1/M3 anonymous connection.
  The upgrade is one `pixi run enroll` per robot (§7 of the runbook) and it is
  the first thing to do on the next bench session. Until then, **an M7 broker
  will refuse the existing robot** — this is a breaking change for a deployed
  fleet, by design, and the agent's log says exactly what to run.
- **The tailnet policy is not applied** (§6).
- **`pixi run fleet-broker-ws` itself was not run.** §5 used a container
  mosquitto with the same image and the same generated credential files, but
  started by hand on non-default ports rather than through `broker.sh --docker`,
  because the workstation already had a broker on 1883/9001. What that leaves
  untested is the script's own `--network host` invocation and its
  `docker kill -s HUP` reload path — the *credentials* reaching a container
  broker is confirmed, the wrapper around it is not.
- **No adversarial testing.** Nothing here attempts to defeat the ACL, only to
  confirm it denies the specific things the design says it should.
