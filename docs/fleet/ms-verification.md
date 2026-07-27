# Ms verification ledger

What was measured for **Ms · server pipelines**, how, and what is still
unverified. The runbook it verifies is
[`server-pipelines.md`](server-pipelines.md).

The milestone's acceptance is two claims: *rebuild either server from scratch
via its documented pipeline*, and *robot autonomy is unaffected while the fleet
server is down*. Both are below, the first as a drill run with the shipped
scripts and the second as a test that runs in `pixi run -e dev test-fleet`.

Everything here was run on the workstation (docker 29.6.1, compose v5.2.0). The
GPU-specific half of the inference pipeline is the one gap, and §4 says exactly
what that leaves unmeasured.

---

## 1. Fleet server: from nothing, to a fleet, and back from a backup — **confirmed**

One drill, seven steps, using only `fleet-deploy.sh` and the shipped
`docker-compose.yml`. The image was built on the box (`up --build`) rather than
pulled, which is also the GHCR-unreachable path.

**Rebuild from scratch.** Volumes destroyed, then:

```console
$ ./fleet-deploy.sh up --build
== starting the fleet server (broker: 127.0.0.1)
 Volume mote-fleet_broker-data Created
 Volume mote-fleet_fleet-state Created
 Container mote-fleet-broker-1 Healthy
 Container mote-fleet-server-1 Started

== health gate
{"schema": 1, "ok": true, "service": "mote-fleet", "contract": "mote/v1", "version": "ms-v1", "robots": 0}
```

**A robot enrolls against it** — the real `pixi run enroll` on the host,
against the containerised server, and both credentials minted by the CLI
*inside* the container (dispatch refuses without an operator token):

```console
$ ./fleet-deploy.sh fleetctl operator new --name drill
mo-PnlRaiLrmDKPkb9J5Uj5IYrHvsAOqOic
$ ./fleet-deploy.sh fleetctl token new
mt-zCiw_sBb1D-Z3iYKO_0wq5HXMwABcZKa

$ pixi run enroll -- --server http://127.0.0.1:18080 --token mt-zCiw… --name Scout --site home
enrolled as mote-01 (new)
  broker:   mini-pc:18831

$ pixi run -e fleet fleetctl -- --server http://127.0.0.1:18080 robots
ID           NAME             SITE       ENROLLED              FINGERPRINT
mote-01      Scout            home       2026-07-26T21:25:44Z  machine_id:d25bff05e0f35ff718a875be6889edcc
```

**M3's surface works in the deployed stack** — the dashboard is served, the
browser's WebSocket path reaches the broker on the published port, and a
dispatch goes through the API, out to the broker, and into the audit log:

```console
dispatch -> 202 {'id': '7caa18da85e84b11'}
command on the broker: [{'schema': 1, 'id': '7caa18da85e84b11', 'command': 'goto kitchen',
                         'issued_by': 'ui:drill'}]
dashboard websocket path: ['mote/v1/mote-01/health']
audit rows: [('mote-01', 'goto kitchen', 'drill', 'published')]
dashboard: 200 text/html; charset=utf-8
```

That publish path is why `--publish-host`/`--publish-port` exist (§5): the first
run of this drill returned **503** on dispatch, because an API container told to
publish to the box's own name resolves it inside the container network — to
itself.

**An update, gated.** `robots: 1` in the answer is the state volume surviving
the redeploy:

```console
$ ./fleet-deploy.sh update mote-fleet:v2
{"schema": 1, "ok": true, ..., "version": "ms-v2", "robots": 1}
== updated to mote-fleet:v2  (rollback: ./fleet-deploy.sh rollback)
$ grep MOTE_FLEET_REF .env
MOTE_FLEET_REF=mote-fleet:v2
```

**An update that cannot serve, rolled back automatically.** The bad image is
the real one with its entrypoint replaced by a process that exits:

```console
$ ./fleet-deploy.sh update mote-fleet:broken
server-1  | boom: this build cannot serve
health gate timed out after 25s

== health gate failed -- rolling back to the previous image
error: rolled back; the new image was not deployed
$ echo $?
1
$ ./fleet-deploy.sh status
{"schema": 1, "ok": true, ..., "version": "ms-v2", "robots": 1}
```

**Backup, destruction, restore.** `docker compose down -v` is the box burning
down — both volumes gone:

```console
$ ./fleet-deploy.sh backup /…/evidence
== wrote /…/evidence/mote-fleet-20260726T191357Z.tgz     (933 bytes)

$ docker compose down -v && ./fleet-deploy.sh up
{"schema": 1, "ok": true, ..., "robots": 0}
$ pixi run fleetctl -- --server http://127.0.0.1:18080 robots
no robots enrolled

$ YES=1 ./fleet-deploy.sh restore …/mote-fleet-20260726T212638Z.tgz
{"schema": 1, "ok": true, ..., "robots": 1}
== restored from mote-fleet-20260726T212638Z.tgz
$ pixi run -e fleet fleetctl -- --server http://127.0.0.1:18080 robots
ID           NAME             SITE       ENROLLED              FINGERPRINT
mote-01      Scout            home       2026-07-26T21:25:44Z  machine_id:d25bff05e0f35ff718a875be6889edcc

$ curl -H "Authorization: Bearer mo-…" .../v1/audit
{"audit": [{"stamp": "2026-07-26T21:25:45Z", "actor": "drill", "action": "dispatch",
            "robot_id": "mote-01", "command": "goto kitchen", "result": "published"}]}
```

Same `enrolled_at`, same fingerprint, and the audit row from before the box was
destroyed: the rebuilt box is the same fleet, not a new one with the same name.

---

## 2. Robot autonomy during a fleet-server outage — **confirmed, and now a test**

[`mote_fleet/test/test_fleet_outage.py`](../../mote_fleet/test/test_fleet_outage.py),
3.7 s, in `pixi run -e dev test-fleet`. Real broker, real agent with a genuine
paho client, real `mote_tasks` behaviour tree, mock `navigate_to_pose`. The
broker process is killed while the agent is connected and restarted on the same
address — a fleet box being redeployed or rebooted, from the robot's point of
view.

What it pins down, in order:

1. the agent notices the broker is gone (LWT is the operator's side of this;
   this is the robot's);
2. a task issued **on the robot** while there is no broker in existence runs to
   completion — `succeeded`, and the Nav2 goal really was sent;
3. the agent survives as a node: its health and pose timers keep firing against
   a dead connection without raising, and `rclpy.ok()` throughout;
4. when the broker returns the agent reconnects **by itself** — no restart, no
   re-enrollment;
5. a *fresh* operator client (the dashboard opened after the redeploy) sees the
   robot immediately, from retained state;
6. and fleet dispatch works again end to end.

```console
$ pixi run -e dev pytest mote_fleet/test/test_fleet_outage.py -v
mote_fleet/test/test_fleet_outage.py::test_robot_keeps_working_while_the_fleet_server_is_down PASSED
1 passed in 3.83s
```

Whole suite after the change: `103 passed in 26.86s`.

**What it does not prove:** that a *physically driving* robot is unaffected.
Nav2 is mocked, so this is the seam (agent ↔ task layer ↔ action client), not
the wheels. The autonomy claim under it — Nav2, SLAM and the tree run locally —
is M0/M1 architecture, unchanged here.

---

## 3. Inference server: blue/green, probe-gated — **confirmed against stub images**

[`mote_perception/deploy/test/drill.sh`](../../mote_perception/deploy/test/drill.sh),
about a minute, `pixi run deploy-test`. It runs the shipped
`inference-deploy.sh` unmodified; what is stubbed is only the model, and the
stub speaks the real wire protocol while the image carries the real
`tools/probe.py`.

```console
== 1. first deploy
depth   OK   127.0.0.1:5601  cpu @ stub-v1  depth=[48, 64] median=1.5m  (0.0s)
detect  OK   127.0.0.1:5602  cpu @ stub-v1  detections=1  (0.0s)
  PASS  deployed and probed

== 2. update to a good build
== verifying candidate on shadow ports 15611/15612
depth   OK   ... @ stub-v2
== cutting over
== updated: stub-v2  (rollback target: sha256:2ae29236c35a)
  PASS  cut over to v2

== 3. update to a build that answers health but cannot infer
depth   FAIL 127.0.0.1:5601  frame rejected or not served
detect  FAIL 127.0.0.1:5602  frame rejected or not served
error: candidate failed its probe; mote-inference-drill is untouched and still serving
  PASS  rejected on the shadow port; v2 still serving

== 4. rollback
== rolling back to stub-v1 (sha256:2ae29236c35a)
  PASS  back on v1

all four checks passed
```

Step 3 is the one worth naming: the bad build **answers its health check** and
fails only on a real frame. A health-only gate would have deployed it. That is
why `probe.py` sends a synthetic image and why the container's `HEALTHCHECK`
(which must not load a model on an idle box) is explicitly the weaker check.

The drill also earned its keep immediately: it caught a bookkeeping bug where a
candidate rejected on the shadow port had already overwritten the rollback
pointer, so a later `rollback` would have restored the version already running.
Fixed — the pointer now moves at cutover and nowhere earlier.

---

## 4. What is **not** verified

- **The real inference image on a real GPU.** The drill proves the pipeline, not
  CUDA: stub containers, `GPUS=none`, no torch. Unmeasured on hardware: how long
  the candidate's first model load takes on the GPU box (the probe allows 300 s),
  whether two containers' worth of VRAM co-exist during the shadow-port check
  (~1 GB each for V2-Small + OWLv2, so expected to be fine on any card that runs
  the server at all), and the true length of the cutover gap. Run
  `./inference-deploy.sh update` on the GPU box and record it here.
- **A Pi as the fleet box.** The image is built `linux/arm64` as well, and it is
  stdlib python, but it has only been run on x86-64.
- **`restore` onto a box that never ran this stack.** The drill restores into a
  freshly created pair of volumes on the same machine; a genuinely new host
  (different docker, different uid mapping) is the untested variant. The archive
  contains plain files and the script chowns them to the image's uids, so no
  problem is expected.
- **Concurrent operators.** One `fleet-deploy.sh` at a time is assumed; nothing
  locks.
- **The dashboard in a real browser, against the deployed stack.** The drill
  proves the routes and the WebSocket path with a paho client using
  `transport="websockets"`; `mote_fleet/test/browser_check.mjs` is the tool for
  the rest, and it was not run here.
- **The websockets listener under a browser.** Verified with a paho client using
  `transport="websockets"` (publish, subscribe, and a late subscriber receiving
  the retained message on port 9001) — a real browser MQTT client is M3's to
  confirm.

---

## 5. Found while rebasing onto M3

- **`fleet-broker` and `fleet-broker-ws` are now one task.** They ran the same
  script with one flag between them, and `fleet-broker-ws` was itself declared
  twice (`[tasks]` and `[feature.fleet.tasks]`), so `pixi run fleet-broker-ws`
  hit the ambiguity below. `pixi run fleet-broker` is now the container — the
  one with websockets, and the one the deployed stack runs — and
  `pixi run -e fleet fleet-broker-local` is the conda binary for a box with no
  docker — in the `fleet` environment, because that is where the binary comes
  from, while the container needs docker rather than an environment. The conda
  package stays a dependency either way: the real-broker tests locate its
  binary.
- **`pixi run fleetctl` and `pixi run fleet-server` are ambiguous** on main
  since M3 defined both in `[feature.fleet.tasks]` as well as `[tasks]`; pixi
  refuses to choose, so the operator flow in `README.md` §7–8 errors as written.
  `pixi run -e fleet fleetctl -- …` works, and is what this ledger's transcripts
  use. Not fixed here — deleting either definition still leaves it ambiguous,
  because the `dev` environment includes the `fleet` feature, so the fix is a
  decision about where those two tasks live. Filed as its own task.

## 6. Notes for later milestones

- **The deployed broker is M3's broker.** `fleet-broker` already runs
  `eclipse-mosquitto` under Docker because conda-forge's build has no websockets
  (M1 §4); the compose service is that same container as a restarting service,
  mounting the *same* `server/mosquitto.conf`. Nothing about the listener set is
  deploy-specific, which is the point.
- **`/healthz` now reports `version`** (`MOTE_VERSION`, baked at image build,
  `unknown` from a checkout). The deploy gate reads it; the dashboard could.
- **`--publish-host`/`--publish-port` are new on the fleet server**, defaulting
  to `--broker-host`/`--broker-port` so nothing outside a container topology
  changes. They exist because those two flags had been doing two jobs — telling
  robots where the broker is, *and* telling the server where to publish
  mediated dispatches — which are the same address everywhere except inside a
  compose network, where the first run of §1 met it as a 503.
- **Tokens are prefixed — `mt-` for enrollment, `mo-` for operators.**
  `secrets.token_urlsafe` can start with `-`, which argparse reads as a flag, so
  the documented `enroll --token <token>` and `fleetctl --token <token>` lines
  failed on roughly one token in twenty. Tokens minted before this are stored
  verbatim and keep working.
