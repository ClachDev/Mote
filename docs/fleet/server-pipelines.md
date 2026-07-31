# Server pipelines — rebuilding the fleet server and the inference server

Milestone **Ms** of [`docs/design/fleet.md`](../design/fleet.md). Three machines
run Mote software and only one of them is a robot; this is the runbook for the
other two.

| | Fleet server | Inference server |
|---|---|---|
| What it is | MQTT broker + the fleet API, registry and dashboard | depth + open-vocabulary detection on a GPU |
| Ships as | two containers + a compose file (`mote_fleet/deploy/`) | one container (`mote_perception/deploy/`) |
| Holds state | **yes** — registry rows, site bundles, retained messages | no — code and model weights only |
| Update | recreate, health-gated, auto-rollback | blue/green: probe a candidate on a shadow port, then cut over |
| If it is down | fleet *management* stops; **robot autonomy does not** | perception degrades to lidar-only; navigation is unaffected |
| Host needs | docker | an NVIDIA driver and docker |

The two pipelines are deliberately different, and the difference is state. The
inference server can run two versions side by side and be *proven* on a real
frame before either serves a robot. The fleet server cannot — two processes on
one SQLite file is a correctness problem, not a capacity win — so its update is
a gated recreate, which is affordable precisely because its downtime does not
reach the robots. That claim is measured, not assumed:
[`test_fleet_outage.py`](../../mote_fleet/test/test_fleet_outage.py) kills the
broker under a live agent and watches the robot finish a task anyway.

Neither of these boxes is fleet-managed. The robot's OTA story (M5) drives
robots from the prefix.dev channel on the fleet server's schedule; a *server* is
ordinary infrastructure, versioned in this repo and deployed by its operator.

---

## 1. The fleet server

### From nothing to a running control plane

The box needs docker and two directories — `mote_fleet/deploy/` and
`mote_fleet/server/mosquitto.conf`, which the compose file mounts so the
deployed broker and `pixi run fleet-broker` cannot drift apart. No ROS, no
pixi, no rest-of-the-repo.

```bash
# on the fleet box: copy mote_fleet/ (deploy/ and server/mosquitto.conf)
cd mote_fleet/deploy
cp env.example .env && $EDITOR .env      # BROKER_HOST is the only required value
./fleet-deploy.sh up
```

`up` starts the broker and the API, then waits on a health gate that reads
`/healthz` over the *published* port — the one an enrolling robot actually
dials — before declaring success:

```console
== health gate
{"schema": 1, "ok": true, "service": "mote-fleet", "contract": "mote/v1", "version": "v0.1.0-63-g0a1b2c3", "robots": 0}
```

**`BROKER_HOST` is the one value you must get right.** It is handed to every
robot verbatim in its enrollment answer, so it has to be the address *robots*
can reach — the box's tailnet (MagicDNS) name, never `localhost` and never a
container name. The compose file refuses to start without it rather than
defaulting to something that would enroll robots into a broker they cannot
find.

Then mint the two credentials and enroll a robot, exactly as in
[`README.md` §7–8](README.md#7-enrolling-a-robot) — `fleetctl` runs inside the
container, because the registry file it writes lives in the container's volume:

```bash
./fleet-deploy.sh fleetctl operator new --name you    # dispatch refuses without one
./fleet-deploy.sh fleetctl token new                  # an enrollment token
pixi run enroll -- --server http://fleet-box:8080 --token mt-…   # on the robot
```

The dashboard is served by the same container, at `http://fleet-box:8080/`.

**Three clients reach the broker by three routes, so the compose file names it
three times.** Robots get `--broker-host`/`--broker-port` at enrollment: the
box's MagicDNS name and its published port. The browser gets `--broker-ws-port`,
again a published port, because its live feed is MQTT-over-WebSockets straight
to the broker. And the API itself — which publishes every mediated dispatch —
gets `--publish-host broker --publish-port 1883`, the service name on the
compose network. That last pair is new here: before it, an API container told to
publish to the box's own name would resolve that name to *itself* and every
dispatch would fail with a broker-unreachable 503. It defaults to
`--broker-host`, so nothing outside a container topology needs it.

If GHCR is unreachable, or you are deploying from a checkout, add `--build`:
the image is a python base plus three pure wheels (152 MB) and builds in
seconds on the box itself, including on a Pi.

### State, and what a backup has to contain

Two named volumes, and both matter:

- `fleet-state` → `/var/lib/mote-fleet`: `registry.db` (who is in the fleet,
  the enrollment tokens, the operator tokens and the audit log — the file that
  makes a rebuilt box the *same* fleet rather than a new one) and `sites/`, the
  site bundles the dashboard draws basemaps from until M4 gives the registry
  that job.
- `broker-data` → `/mosquitto/data`: retained messages and QoS-1 queues, so the
  roster is not blank until every robot's next heartbeat.

```bash
./fleet-deploy.sh backup /var/backups/mote     # -> mote-fleet-<UTC stamp>.tgz
./fleet-deploy.sh restore /var/backups/mote/mote-fleet-20260726T191357Z.tgz
```

The whole state root goes in, with the registry copied through sqlite3's online
backup API rather than `cp`, because the server is still writing to it and half
a page is not a registry.
Restore stops the stack first — mosquitto writes its persistence file on
shutdown, so restoring under a running broker would be overwritten seconds
later.

**The whole rebuild is therefore: copy the deploy directory, `up`, `restore`.**
Nothing else on that box is precious.

### Updating

```bash
./fleet-deploy.sh update                       # the ref pinned in .env
./fleet-deploy.sh update ghcr.io/clachdev/mote-fleet:v0.2.0
./fleet-deploy.sh rollback
```

`update` tags the running image locally as `:previous` *before* pulling —
`latest` will have moved by the time an update goes wrong, so a tag is not a
rollback target — then recreates the stack and runs the same health gate. If
the gate fails it puts the previous image back, checks *that* is healthy, and
exits non-zero having deployed nothing:

```console
== health gate failed -- rolling back to the previous image
error: rolled back; the new image was not deployed
```

`.env` is rewritten by both paths, so the file always states what is actually
deployed and a later bare `docker compose up -d` cannot silently undo the
decision. A rollback pins the previous image **by digest** when it has one; a
locally built image has no digest, so the local `:previous` tag stands in.

The state volumes are untouched by all of this — the drill in §3 updates,
rolls back, destroys the box and restores it, and the same robot is in the
registry at the end.

### The broker container, and the one config

The deployed broker mounts
[`../server/mosquitto.conf`](../../mote_fleet/server/mosquitto.conf) — the file
the workstation broker uses too. That is deliberate: M3 already had to run
mosquitto in a container, because conda-forge's build has no websockets
(m1-verification.md §4) and the dashboard subscribes from the browser. `pixi run
fleet-broker` is that container as a foreground command; this compose service
is the same thing as a restarting service with a volume for its retained state
and a healthcheck.

Keeping one config means the listener set cannot drift between the box you
develop on and the box you deploy. `working_dir: /mosquitto/data` is what puts
`persistence_file mosquitto.db` in the volume — the same reason `broker.sh` cds
to `$MOTE_FLEET_HOME`.

**The image tag is pinned in one place**, the compose file's broker service;
`broker.sh` reads that line rather than carrying a default of its own, so the
foreground broker and the deployed one cannot end up on different mosquittos.
It names a **minor** series (`2.1-alpine`) rather than the floating `:2`,
because the thing that must not move under us is how websockets is built in:
2.0 links libwebsockets, 2.1 implements it natively, and `:2` follows whatever
comes next. `-alpine` is not a variant choice — upstream publishes 2.1 only as
`-alpine` tags, and `:2`, `:2.1-alpine` and `:2.1.2-alpine` are one manifest
today. `MOTE_BROKER_IMAGE` overrides it for both.

**The healthcheck probes both listeners**, not just MQTT. Mosquitto opens the
MQTT listener and keeps running when the websockets one is refused, so an
MQTT-only probe reports a healthy stack whose dashboard is dead — and
`fleet-deploy.sh up` gates on that probe, so it would report a good deploy.
`mosquitto_sub` asks the broker for its own uptime on 1883, which is more than
a TCP connect proves; 9001 is a TCP probe, because the failure being caught is
"the listener never opened" and `mosquitto_sub` cannot speak websockets to do
better. `mote_fleet/test/test_deploy_config.py` holds all of the above without
needing docker.

### Security posture

Unchanged from M3, and stated here because a container makes it easy to expose
by accident: the broker is **anonymous** and the API's *read* routes are
**unauthenticated** (dispatch needs an operator token; that is one credential on
one path, not an auth story). It is proportionate only while the tailnet is the
boundary. `BIND_PREFIX` in `.env` is the lever — set it to the box's tailnet
address, with a trailing colon, and none of the three ports is published on the
LAN at all. Per-robot broker credentials and operator auth everywhere are M7.

---

## 2. The inference server

Deployment, health, the on-demand model loading and the fallback matrix are in
[`docs/inference-server.md`](../inference-server.md); this section is only the
*pipeline*.

### From nothing to a serving GPU box

```bash
curl -fsSLO https://raw.githubusercontent.com/ClachDev/Mote/main/mote_perception/deploy/inference-deploy.sh
chmod +x inference-deploy.sh
./inference-deploy.sh up
```

One file on the host, and it stays one file: everything the script needs to
check — the health sentinel, a real inference request — is *inside the image it
is deploying*, so the GPU box never grows a checkout, a python environment or a
set of scripts. That is the same rule the container itself follows.

### Blue/green, and what the gate actually proves

```bash
./inference-deploy.sh update       # pull latest, verify, cut over
./inference-deploy.sh rollback
./inference-deploy.sh status
```

`update` starts the candidate as a second container on **shadow ports**
(5611/5612) while the current one keeps serving the robots, and probes it with
[`tools/probe.py`](../../mote_perception/tools/probe.py) — health *and* a
synthetic frame through both services:

```console
== verifying candidate on shadow ports 15611/15612
depth   OK   127.0.0.1:5601  cuda @ v0.1.0-63-g0a1b2c3  depth=[480, 640] median=2.14m  (7.9s)
detect  OK   127.0.0.1:5602  cuda @ v0.1.0-63-g0a1b2c3  detections=1  (4.2s)
```

The frame is the point. A health check is answered before the model has ever
loaded, so it cannot see a broken weight download, a CUDA/driver mismatch, or a
torch build that faults on the first forward pass — the failures that actually
happen. Serving a frame forces the on-demand load
([`model_host.py`](../../mote_perception/tools/model_host.py)) and a full
inference. A candidate that fails is removed and the update aborts with the
current version still serving, untouched:

```console
depth   FAIL 127.0.0.1:5601  frame rejected or not served
error: candidate failed its probe; mote-inference is untouched and still serving
```

**The cutover is a stop-then-start, not a load-balancer flip.** It costs a few
seconds plus the new container's first model load. The design sketch
([fleet.md](../design/fleet.md#the-other-pipelines-fleet-server--inference-server))
imagined keeping both alive and pushing the new port to the robots; that is a
worse outage than the one it avoids — it means editing `perception.yaml` on
every robot and relaunching perception, and it leaves the fleet's config
transiently disagreeing about where inference lives. What blue/green is worth
here is the *gate*, not the instant flip: a bad build never touches the served
ports. The gap itself is a non-event, because the robot treats "no server" as
"skip this frame" and navigates on lidar (the fallback matrix in
[`inference-server.md`](../inference-server.md#fallback-matrix-server-present--absent)).

After the cutover the same probe runs against the live container; if *that*
fails, the previous image is restored automatically and the script exits
non-zero. The rollback pointer (`~/.mote-inference/previous`) moves only at
cutover, so an update abandoned on the shadow port leaves the rollback target
intact.

### Knobs

Configuration is environment variables — `IMAGE`, `TAG`, `NAME`, `GPUS`,
`BIND`, the four ports, `SERVER_ARGS` (e.g. `--idle-timeout 0`), `STATE_DIR`.
`./inference-deploy.sh --help` lists them with defaults. `BIND=100.x.y.z`
publishes only on the tailnet address; the sidecar pattern in
[`inference-server.md`](../inference-server.md#scaling-to-a-cloud-gpu) is the
stronger option for a rented cloud box.

---

## 3. Testing the pipeline without a GPU

[`mote_perception/deploy/test/drill.sh`](../../mote_perception/deploy/test/) runs
the real `inference-deploy.sh` against stub images that speak the real wire
protocol and carry the real probe:

```bash
pixi run deploy-test        # or ./mote_perception/deploy/test/drill.sh — docker only
```

Four checks, about a minute, on any machine: first deploy; a good update cuts
over; a build that answers health but rejects every frame is **caught on the
shadow port** with the old version still serving; and rollback restores the
previous version. Only the model is faked — the deployment machinery under test
is the one that runs on the GPU box. It has already earned its keep: it caught a
bookkeeping bug where a failed update overwrote the rollback pointer.

The fleet server's equivalent is `test_fleet_outage.py` (autonomy during an
outage) plus the manual drill recorded in
[`ms-verification.md`](ms-verification.md).
