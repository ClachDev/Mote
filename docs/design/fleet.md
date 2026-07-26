# Fleet architecture

How Mote robots are operated **completely remotely** — live map + location feed,
task dispatch, health, software updates — with **multiple robots assumed from
day one**, not retrofitted later.

This is a design document. No implementation lands with it; the milestone
breakdown at the end is sized so each milestone becomes one dispatchable task.

> **Scope note.** Everything here builds on seams that already exist in the repo
> — Sites (`mote_bringup/mote_bringup/sites.py`), the task layer
> (`mote_tasks/mote_tasks/task_server.py`), the inference wire
> (`mote_perception/mote_perception/depth_wire.py`), and the pending prefix.dev
> release channel (`pixi.toml:3`). Where a recommendation leans on one of those,
> the file and line are cited. Claims about candidate technologies link their
> docs; anything I could not verify is flagged **(verify)**.
>
> **Two horizons.** Mote is an example/portfolio project, so the design is
> written for the **homelab horizon** (1–10 robots, one operator, one site) —
> that is what v0/v1 build and what the recommendations optimise for. But every
> choice is also judged against a **platform horizon** (1k–10k robots across many
> sites and customers) so we can see its cost curve and its breaking point. The
> [Scaling](#scaling-one-robot--ten-thousand) section makes that explicit; each
> transport choice below carries a **Cost & tradeoffs** note so the downside is on
> the page next to the upside.

---

## Where we are today

A single robot, and the fleet layer is greenfield in the two places that matter:

- **One hardcoded robot.** `pixi run sync` rsyncs the source tree to
  `michael@auldbot:~/Mote/` (`pixi.toml:26`) and it is built on the Pi. The docs
  call the SSH host `mote` (`README.md`, `CLAUDE.md:22`) but the committed sync
  string says `auldbot` — the name is already drifting, and there is **no
  `robot_id`, no per-robot ROS namespace, and no per-robot DDS domain** anywhere
  in the tree. The only per-machine state is `~/.mote/active.yaml`
  (which site/floor the robot is on — `sites.py:4`), and that is *location*
  state, not identity.
- **One flat, unconfigured DDS graph.** `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
  is set once, globally, in the pixi activation env (`pixi.toml:219-221`). There
  is **no CycloneDDS XML** anywhere, `ROS_DOMAIN_ID` is never set (defaults to 0),
  and remote interaction today is "put the workstation on the same LAN and the
  same domain and let multicast discovery find everything"
  (`mote_perception/config/README.md:49-51`). That works on a bench. It does not
  cross a NAT, a firewall, or a second site, and it does not isolate two robots.

So the fleet layer is not a refactor of something — it is new surface. The job of
this design is to add that surface **without disturbing the parts of the robot
that already work offline**: Nav2, SLAM, and the `mote_tasks` behaviour tree all
run locally today and must keep running with the fleet server unplugged.

---

## The one-paragraph answer

Put every robot, the fleet server, the operator's machine, and the GPU inference
box on a **single WireGuard mesh (Tailscale)** so the LAN-vs-internet distinction
disappears and everything below rides an encrypted, NAT-traversing, identity-bearing
network for near-zero effort. On that mesh: each robot runs a small **`mote_agent`**
that is the *only* thing talking to the fleet server — it bridges the existing
`task/command` / `task/status` topics and reports health over **MQTT**, while the
robot's autonomy keeps running locally if the link drops. Adopt **Foxglove** (via
`foxglove_bridge`) as the deep single-robot console — live pose, camera peek,
teleop — and **build only a thin fleet dashboard** for the roster + dispatch view
Foxglove doesn't provide. The **fleet server is the central site/map registry**;
because Sites are already immutable file bundles distributed by an atomic symlink
flip (`sites.py:254-258`), map distribution is "copy a revision dir, flip a link."
Updates consume the pending **prefix.dev `mote` channel** (`pixi.toml:3`) —
install-alongside, health-check, keep the old env for rollback — so there is one
update mechanism, not two. Keep the **fleet server and the inference server as
separate roles** (small always-on vs big intermittent GPU); they may co-locate but
scale independently. The whole thing is deliberately the *cheap homelab shape*,
but the seams are chosen so the platform-scale swaps
([Scaling](#scaling-one-robot--ten-thousand)) are implementation changes behind the
same seams, not a redesign.

---

## Component diagram

```mermaid
graph TB
    subgraph tailnet["Tailscale mesh (WireGuard, encrypted, NAT-traversing)"]
        subgraph robot1["Robot: mote-01 (Raspberry Pi)"]
            ros1["ROS 2 graph (local, domain-isolated)<br/>Nav2 · SLAM · mote_tasks BT<br/>runs fully offline"]
            agent1["mote_agent (systemd)<br/>bridges task/* · reports health<br/>pulls map revisions · runs OTA"]
            fbridge1["foxglove_bridge"]
            ros1 -->|"task/status, /tf, /map"| agent1
            agent1 -->|"task/command"| ros1
            ros1 --- fbridge1
        end
        subgraph robotN["Robot: mote-02 … (same layout)"]
            dots["…"]
        end
        subgraph fleet["Fleet server (small, always-on: VPS / home box / Pi)"]
            broker["MQTT broker (Mosquitto → EMQX at scale)<br/>retained state · LWT · per-robot topics<br/>+ MQTT-over-WebSocket for the web UI"]
            registry["Site/map registry<br/>canonical site bundles + revisions"]
            api["Fleet API + thin web UI<br/>roster · health · dispatch · rollout"]
            ota["Update orchestrator<br/>ring rollout state"]
        end
        infer["Inference server (big, intermittent GPU)<br/>depth + detect over raw TCP<br/>(mote_perception/depth_wire.py)"]
        ops["Operator<br/>Foxglove (deep view) + fleet dashboard (roster)"]
    end

    prefix["prefix.dev &quot;mote&quot; channel<br/>versioned package releases<br/>(robot + inference-server packages)"]

    agent1 <-->|MQTT: health / dispatch / OTA state / map-notify| broker
    agent1 <-->|"HTTP/rsync: pull & publish revisions"| registry
    fbridge1 <-->|Foxglove WS: pose · camera · teleop| ops
    broker <-->|MQTT-over-WS: live roster + pose| ops
    api --- ops
    ros1 -.->|"raw TCP depth/detect (unauthenticated wire, now on tailnet)"| infer
    ota -.->|"target version per ring"| agent1
    agent1 -.->|pixi install pinned version| prefix
    infer -.->|pixi install (same channel, own env)| prefix
```

---

## The seven questions

### 1. Topology — per-robot agent + central fleet server

**Recommendation.** Each robot runs a lightweight **`mote_agent`** (a systemd
service alongside the existing ones — `mote_bringup/systemd/` already holds
`mote-bringup`, `mote-slam`, `mote-nav`, `mote-record`, so a `mote-agent.service`
is the established pattern). The agent is the *only* process that talks to the
fleet server. A central **fleet server** hosts the site/map registry, the task
dispatch + telemetry control plane, the update orchestrator, and the operator
API/UI.

**The agent is a bridge and reporter, never in the control loop.** This is the
load-bearing property. The robot's autonomy already runs entirely locally: the
`mote_tasks` behaviour tree ticks on a timer inside `task_server` and drives Nav2
directly (`task_server.py`), SLAM and localisation are local nodes, and the active
site/floor + map live on-disk in `~/.mote` (`sites.py:88-97`). None of that
depends on the fleet server. So when connectivity drops:

- the robot keeps executing its current mission and stays navigable;
- the agent re-publishes current state on reconnect (MQTT's retained + LWT
  semantics, below, make "went offline / came back" clean);
- inbound commands are safe to retry. `task/command` is a bare `std_msgs/String`
  with no correlation id (`task_server.py:1-15`), so the agent enforces the rule
  that makes retry safe: **one in-flight command per robot** — it holds a command
  until the tree returns a terminal `succeeded:`/`failed:`, attributes the next
  `accepted:`/`rejected:` to that command, and carries an MQTT correlation id
  upstream of the ROS seam. Without that convention the bare-String seam can't tell
  two in-flight commands apart.

**The agent being the sole egress is also the fleet-scale isolation story.**
Because DDS never leaves the robot (nothing subscribes to the robot's DDS graph
from off-box — the agent translates to MQTT/Foxglove instead), we can pin discovery
to the local host (Q3), so there is no fleet-wide DDS graph to partition, discovery
floods can't cross robots, and the `ROS_DOMAIN_ID` question disappears.

**Fleet server vs inference server: separate roles.** The task brief asks whether
they are the same box. They should be **distinct roles**, because their shape is
opposite:

| | Fleet server | Inference server |
|---|---|---|
| Uptime | always-on (availability-critical) | up only when a robot needs perception |
| Compute | tiny (broker, files, a web app) | heavy GPU (torch: OWLv2, Depth-Anything) |
| Hardware | VPS / home box / even a spare Pi | Windows/NVIDIA gaming PC |
| State | registry + retained broker state (back up) | stateless (models + code only) |
| Trust | holds fleet secrets, operator auth | runs an **unauthenticated** wire protocol |

That last row is decisive. The inference wire is explicitly *"unauthenticated;
run it on a trusted network"* (per the inference server; the node opens a plain
`AF_INET`/`SOCK_STREAM` connection to `inference_host:server_port`,
`depth_wire.py:88-91`), and it is deliberately **decoupled from ROS/DDS**
(`depth_wire.py:9-14`) with the host chosen by config so *"inference can move
machines without editing launch"* (`CLAUDE.md:120`, `perception.yaml:8`). Keeping
it a separate role means its box can be off half the day without touching fleet
availability, and its unauthenticated port is never something the fleet server has
to firewall around — the Tailscale substrate (Q2) makes "trusted network" true
across the internet for free. They *may* co-locate on one physical machine at home
today; the design just refuses to *assume* it. Provisioning + updates for both of
these non-robot roles are in
[The other pipelines](#the-other-pipelines-fleet-server--inference-server).

---

### 2. Transport — Tailscale substrate, Foxglove for rich data, MQTT for control

This is the crux, and the right answer is **layered**, not a single protocol.

#### Layer 0 — network substrate: Tailscale (WireGuard mesh) on everything

Put the robots, the fleet server, the operator machine, and the inference box on
one [Tailscale](https://tailscale.com/) tailnet. This is the single
highest-leverage decision in the whole design, because it dissolves the problem
every other transport option otherwise has to solve individually:

- **Off-LAN / NAT / firewall "just works."** WireGuard establishes direct
  encrypted tunnels between devices on different networks behind firewalls with
  *no port forwarding and no public exposure*
  ([Tailscale on Raspberry Pi](https://tailscale.com/learn/how-to-ssh-into-a-raspberry-pi)).
  The messy NAT question that Zenoh, Foxglove-remote, rosbridge, and MQTT each
  answer differently stops being N problems and becomes one.
- **Cheap on a Pi.** WireGuard is a kernel module; idle cost is negligible, which
  matters on the Pi that is already carrying Nav2 + SLAM + perception.
- **Stable identity + encryption for free.** Every device gets a stable MagicDNS
  name and an always-on encrypted link, and the free tier covers a homelab fleet
  (**verify** current free-tier limits at adoption — historically ~100 devices /
  3 users with ACLs, MagicDNS, subnet routing).
- **It directly fixes today's model.** The current "same LAN + same
  `ROS_DOMAIN_ID`" workflow (`mote_perception/config/README.md:49-51`) becomes
  "same tailnet."

**Cost & tradeoffs.**
- **Pricing shape bites at scale.** Tailscale charges *per user*, with unlimited
  *user* devices — but robots are **tagged devices** (not tied to a human), and
  tagged resources start at **50 per plan with $1/device/month overage**
  ([Tailscale pricing](https://tailscale.com/pricing)). So 200 robots ≈ $150/mo
  in tags; **10,000 robots ≈ ~$10k/mo** in tag overages alone, before per-user
  seats. Cheap at homelab scale, a real line item at platform scale.
- **Cloud-locked coordination plane.** The control/coordination server is
  Tailscale's SaaS; you can self-host DERP *relays* but not the coordinator. That
  is a vendor dependency and a availability/SPOF consideration for a commercial
  fleet.
- **Relay latency.** If two peers can't NAT-punch a direct path, traffic falls
  back to a DERP relay, adding latency and a bandwidth ceiling (relays are for
  connectivity, not throughput). Direct WireGuard is near-line-rate; relayed is
  not — matters for the inference link (Layer 3).
- **Single flat tailnet doesn't multi-tenant.** One tailnet mixing many customers'
  robots is wrong for isolation and blast radius.
- **Escape hatch: [Headscale](https://headscale.net/)** — an open-source, BSD-3
  self-hosted reimplementation of the coordination server (single-tailnet, embedded
  DERP). It removes the per-device cost and the vendor lock at the price of
  operating it yourself. This is the Regime-B/C answer, and it is a drop-in behind
  the same "every device has a stable overlay IP" seam.

#### Layer 1 — rich live data + teleop: Foxglove (adopt, don't build)

Run [`foxglove_bridge`](https://docs.foxglove.dev/docs/visualization/ros-foxglove-bridge)
on each robot (`ros-jazzy-foxglove-bridge` 3.3.0 is on robostack-jazzy with a
linux-aarch64 build, so the Pi is covered). It speaks the
[Foxglove WebSocket protocol](https://foxglove.dev/robotics/rosbridge) — like
rosbridge but with ROS 2 `.msg`/`.idl` schema support, parameters, and graph
introspection, and *faster* than rosbridge. Foxglove is the operator's **deep
single-robot console**: live pose on the floor map, `/tf`, camera peek
(the camera already publishes `/image_raw/compressed`, `CLAUDE.md`), raw topic
inspection, and **teleop** (publishing back to the robot is supported over the
WS protocol). Reach it over the tailnet directly
(`wss://<robot-magicdns>:8765`) or via Foxglove's device-token remote path
(`FOXGLOVE_DEVICE_TOKEN`).

**Is it a full RViz replacement? No — and that's fine.** Many teams move from RViz
to Foxglove for remote/scale workflows, but **RViz's interactive-marker primitives
remain RViz-only**, and teams keep a targeted RViz session for that
([RViz vs Foxglove vs Rerun](https://foxglove.dev/robotics/rviz-vs-foxglove-vs-rerun)).
Mote already ships RViz in the `dev` environment (`pixi run rviz`, `CLAUDE.md`), so
we keep RViz for local development and adopt Foxglove for *remote/fleet* viewing —
they don't compete.

**Alternatives considered.**
- **RViz** — local only, no remote/fleet story, no web client. Kept for dev, not a
  fleet console.
- **[Rerun](https://foxglove.dev/robotics/rerun-vs-foxglove)** — fully open source
  (MIT/Apache), excellent for logged/offline analysis, but **no live multi-user /
  cloud-hosted live sessions** — the exact thing a remote ops console needs. Wrong
  fit for live teleop; right fit if we later want open-source offline bag analysis.
- **rosbridge** — older JSON protocol, no schema, slower; Foxglove WS supersedes it
  ([What is ROSBridge?](https://foxglove.dev/robotics/rosbridge)). Compatibility
  fallback only.
- **Build our own ROS web viewer** — large project, no payoff over Foxglove's free
  tier.

**Cost & tradeoffs.** Free tier = **3 users / 5 devices / 10 GB**
([Foxglove pricing](https://foxglove.dev/pricing)) — covers v0/v1, and the **5
connected-device cap is the first wall we hit** (Regime B). Paid pricing is
per-user + storage + usage (Enterprise is quote-only). The tradeoff is a **vendor
dependency for the ops console** and a per-seat/per-device cost curve; the mitigation
is that Foxglove is *adopted at a seam* (it consumes the robot's ROS graph and
nothing else depends on it), so it can be swapped for Rerun-offline + a custom live
panel later without touching robot code. Note also that streaming camera/pointcloud
to Foxglove is **bandwidth-heavy** — throttle/scope topics per connection, and over
a DERP-relayed link expect it to be the first thing that saturates.

#### Layer 2 — fleet control plane: MQTT (`mote_agent` ↔ fleet server broker)

The structured control plane — enrollment, task dispatch, health heartbeats,
update state, map-change notifications — is a different shape from live
visualisation: many robots, small messages, and semantics that want *durability*.
Use **MQTT** (Mosquitto broker on the fleet server; both
[mosquitto](https://anaconda.org/conda-forge/mosquitto) and
[paho-mqtt](https://anaconda.org/conda-forge/paho-mqtt) are on conda-forge, so the
agent stays inside the pixi solve). MQTT fits because its native features *are* the
fleet semantics we'd otherwise hand-build:

- **Last Will & Testament** → a robot that drops off is detected *instantly*.
- **Retained messages** → the last-known health/pose of every robot is available
  the moment a client connects; no "wait for the next heartbeat."
- **Per-robot topic namespacing** → `mote/<robot_id>/health`,
  `mote/<robot_id>/task/command`, `mote/<robot_id>/task/status`,
  `mote/<robot_id>/ota/state`, `mote/<robot_id>/pose`. Multi-robot falls out of the
  topic tree.
- **Cheap fan-out at the broker**, and cheap on the Pi.

The bridge is trivial because the dispatch seam is already string-shaped:
`task/command` and `task/status` are both `std_msgs/String`
(`task_server.py:67-68`), grammar `fetch <target> <drop_zone>` in and status
strings (`accepted:`/`rejected:`/`succeeded:`/`failed:`) out
(`task_server.py:1-15`). The agent JSON-wraps those onto MQTT — no new ROS message
types, no changes to `task_server`.

**Can a web UI connect to the broker directly?** Yes. Browsers speak **MQTT over
WebSockets** via [MQTT.js](https://www.emqx.com/en/blog/mqtt-vs-websocket); the
broker (Mosquitto, EMQX, HiveMQ all expose a WS listener) routes messages to the
browser exactly as to any client. So the fleet dashboard (Q5) subscribes to
`mote/+/pose` and `mote/+/health` over WS and renders live positions with no
polling and no extra service in between. This is why the ops UI stays thin.

**Cost & tradeoffs / limitations.**
- **Pub/sub only — no native request/response.** Command→ack is modelled with a
  reply topic + correlation id; fine, but it's a convention we implement, not a
  built-in.
- **No message history/replay.** Unlike Kafka, MQTT keeps only the *last retained*
  value per topic. Good for "current pose/health"; if we later want a telemetry
  time-series we add a subscriber that writes to a DB (or bridge to Kafka — EMQX
  Enterprise has a Kafka data bridge). Out of scope for v1.
- **Payloads should stay small.** MQTT is for control/telemetry, not for shipping
  maps or camera frames — those go over HTTP (registry) and Foxglove respectively.
- **Broker is a scaling knob, not a wall.** Mosquitto (single process) is right for
  a homelab; it does *not* cluster. At Regime B/C swap it for **EMQX/HiveMQ**, which
  are benchmarked into the **millions of concurrent connections** and cluster
  horizontally ([EMQX WebSocket scaling](https://www.emqx.com/en/blog/a-deep-dive-into-emqx-s-websocket-performance)).
  Same protocol, same topic tree, same agent code — a broker swap, not a redesign.
- **Auth is broker-config, not free.** Anonymous by default; we must configure
  per-robot credentials / TLS (Q7).
- **Fallback if we didn't want a broker at all:** a single persistent
  WebSocket/HTTP from agent to fleet server — but then we reimplement LWT and
  retained state by hand. A Mosquitto broker is a ~10-minute setup that gives both
  for free, so MQTT is the recommendation.

#### Layer 3 — inference: unchanged, now on the tailnet

The depth/detect link stays exactly as built — raw length-prefixed TCP on ports
5601/5602 (`depth_wire.py:16-31`, `detect_wire.py:27-29`), `inference_host` from
`perception.yaml` (`perception.yaml:8`). The only change is that `inference_host`
points at the GPU box's **MagicDNS name**, so "run it on a trusted network" holds
across the internet. Nothing in the perception code changes.

**Does the tailnet affect latency? Yes, and it dictates placement.** WireGuard adds
a small, roughly fixed per-packet crypto overhead; on a shared LAN that is
negligible next to the torch inference time itself. But **if the robot and the GPU
box are on different physical networks, the depth/detect round-trip now pays the
WAN RTT** (tens to hundreds of ms), and if the two can't NAT-punch a direct path
the DERP relay adds more. Perception is latency-sensitive (it feeds Nav2's
obstacle layer and the fetch detector), so the design rule is: **keep the inference
server on the same LAN/site as the robots it serves.** `inference_host` per-robot
config (already `~/.mote`-overridable, `perception.yaml:5-6`) means each robot
points at its *local* GPU box; the tailnet is what makes an *occasional* cross-site
fallback possible, not the normal path. At platform scale this becomes "one
inference node per site," which the per-robot `inference_host` already expresses.

#### Rejected / deferred: rmw_zenoh and the Zenoh bridge

- **rmw_zenoh (swap the RMW).** *Not production-ready for Jazzy* — maintainers call
  it "mostly feature-complete" but still changing, first supported release targeted
  at **Kilted** (the distro after Jazzy)
  ([rmw_zenoh binaries announcement](https://discourse.openrobotics.org/t/rmw-zenoh-binaries-for-rolling-jazzy-and-humble/41395)).
- **What would Zenoh actually buy us, and would it simplify other decisions?**
  Zenoh is a mesh data-plane that natively routes across WAN/NAT and scales discovery
  far better than DDS, with per-robot namespace prefixing and TLS/ACLs at the Zenoh
  layer
  ([zenoh-plugin-ros2dds](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds),
  [Zenoh access control](https://zenoh.io/docs/manual/access-control/)). Notably the
  bridge is *"tested with `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`"* — exactly Mote's
  RMW. **If adopted, it could in principle absorb Layer 0 *and* the transport for
  Foxglove/inference** (one mesh instead of Tailscale + per-app transports), because
  it gives WAN routing + identity + ACLs in one layer. That is the theoretical
  simplification.
- **Why still defer.** The simplification is real but the maturity and blast radius
  are not worth it now: it either means swapping the robot's working RMW (waits for
  Kilted — and *"I would need a good reason"* to jump ROS versions, which this alone
  isn't), or running the `zenoh-plugin-ros2dds` bridge as an *addition* — which is
  more moving parts than Tailscale + MQTT + Foxglove for a homelab, without removing
  any of them until we commit to Zenoh end-to-end. **Recommendation:** keep it as the
  documented consolidation path — the seams (agent-as-egress, robot_id namespacing)
  are Zenoh-compatible — and re-evaluate at Kilted, when the "good reason" would be:
  we've hit Tailscale's per-device cost wall *and* want ROS-graph federation between
  robots. Until both are true, don't.

---

### 3. Identity & provisioning — a `robot_id`, per-robot state stays in `~/.mote`

**Recommendation.** Introduce a stable **`robot_id`** as the fleet's primary key,
decoupled from the hostname (already ambiguous — `auldbot` in `pixi.toml:26` vs
`mote` in the docs). Cache it in a new **`~/.mote/robot.yaml`**, alongside the
`active.yaml` that already lives there (`sites.py:4`) and honouring the same
`MOTE_HOME` override (`sites.py:63-64`):

```yaml
# ~/.mote/robot.yaml   (per-robot cache; the fleet server is the source of truth)
id: mote-01            # assigned by the server at enroll; keys every topic + registry row
name: "Front desk"     # human label, editable in the ops UI
site: acme-hq          # which site's bundles this robot is entitled to
```

**DDS stays on the robot — which dissolves the domain-ID problem entirely.** The
cleaner answer to "how do we isolate robots' DDS" is not to assign domains but to
**stop DDS from leaving the machine at all**. ROS 2 Jazzy does this with one
environment variable — `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` — which confines
discovery to the local host (it replaces the deprecated `ROS_LOCALHOST_ONLY`)
([Improved Dynamic Discovery](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Improved-Dynamic-Discovery.html)).
This is a natural fit here because the agent and `foxglove_bridge` run **on the same
Pi** as the ROS graph (Q1: the agent is the sole egress), so nothing off-box ever
needs to join the robot's DDS graph. With discovery pinned to localhost:

- two robots on the same physical LAN (a charging room) simply cannot see each
  other's graph, regardless of `ROS_DOMAIN_ID` — so **there is no domain to
  allocate or manage**, and `domain_id` drops out of `robot.yaml`;
- multicast discovery floods stay on-box, which is also better for a shared LAN;
- the one thing it changes is the *bench* workflow where a workstation shares the
  LAN graph (e.g. camera calibration, `mote_perception/config/README.md:49-51`) —
  that's a dev-time convenience, so the setting is the robot default and developers
  flip discovery back to `SUBNET` when they want it. Fleet ops never needs it,
  because Foxglove/MQTT/inference are the off-box paths.

(This makes the `ROS_DOMAIN_ID` range limit — 0–232, safe 0–101, a per-machine
concept, [docs](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Domain-ID.html)
— a non-issue rather than something to engineer around.)

> **(verify)** In localhost-peer mode rmw_cyclonedds defaults
> `MaxAutoParticipantIndex` to ~32 — roughly 32 discoverable ROS processes per host.
> A full stack (bringup + Nav2 + SLAM + perception + task server + agent +
> foxglove_bridge) could approach that. Count participants on the robot and, if
> needed, raise it via a `CYCLONEDDS_URI` config — since this was written, the
> robot's systemd units load `mote_bringup/config/cyclonedds.xml` (single-interface
> pin + SPDP-only multicast), so that is a setting to add, not a file to create.
> Q3's answer leans on this setting.

**Identity is server-allocated.** The robot cannot know its `id` before it
registers, so the server owns the id space: `mote enroll` presents a bootstrap token
+ hardware facts (MAC, serial), the server allocates `id`/`name` and records the row,
and the agent writes `robot.yaml` as a cache that re-enrolling can rebuild.
(Sequencing: server allocation needs the enrollment endpoint + registry, which land
with the control plane in **M1** — so **M0** bootstraps a single robot with an
operator-set `id` written straight into `robot.yaml`/cloud-init, and the allocation
flow replaces that once M1 exists. See Milestones.)

**Per-robot vs shared config — formalise the split that already exists.** Shared
*code + config* ships identically via the prefix.dev package (Q6). Per-robot
*state* stays in `~/.mote` — `active.yaml`, `camera_calibration.yaml`, the
uncommitted `perception.yaml` override (`perception.yaml:5-6`), and now
`robot.yaml`. The clean consequence: **an update can never touch identity**,
because updates replace the package and `~/.mote` is outside it.

#### Provisioning a new robot — one flow, built on the tools we already use

This is the single authoritative enrollment sequence. It resolves the ordering
trap ("if the robot needs a Tailscale key to reach the network, how does it get the
key from the server?") by making the Tailscale key a **provisioning-time secret
baked into the image, not a runtime fetch** — the robot is on the tailnet *before*
it ever talks to the fleet server.

Mote already images Pis with **Raspberry Pi Imager** and keeps **Raspberry Pi
Connect** for break-glass access; this flow slots into both rather than replacing
them. Imager's OS-customization now writes **cloud-init `user-data`** to the boot
partition (Imager 2.0+), which can create users, join wifi, install packages, and
run first-boot commands headlessly
([cloud-init on Raspberry Pi OS](https://www.raspberrypi.com/news/cloud-init-on-raspberry-pi-os/)) —
so the "golden image" is just **our stock Raspberry Pi OS image + a `user-data`
template**, not a bespoke image to maintain.

At imaging time, the operator (or a small provisioning script that holds the
tailnet API credential and mints the secrets) fills the `user-data` template with a
**single-use, pre-authorised, tagged (`tag:robot`) Tailscale auth key**
([auth keys](https://tailscale.com/kb/1085/auth-keys)) and a **short-lived
enrollment token**. First boot then runs, with no interactive steps:

1. cloud-init installs pixi and `pixi install`s the robot package from the
   prefix.dev channel (the ROS software), and runs the one-time `pixi run setup`
   (udev, wifi-powersave, systemd units — `CLAUDE.md`,
   `mote_bringup/systemd/install.sh`); this is the OS-level setup the channel does
   *not* carry.
2. `tailscale up` with the baked-in pre-auth key → **the robot joins the tailnet**
   and gets a MagicDNS name. (The key was minted out-of-band and written to the
   image; it is never fetched over the network the robot can't yet reach.)
3. `mote enroll` — now reachable over the tailnet — presents the bootstrap token to
   the fleet server, which **allocates `id`/`name`, records the robot, and pushes
   the entitled site bundle** (Q4). The agent writes `robot.yaml`.
4. The robot appears in the roster.

**Raspberry Pi Connect stays as the independent break-glass path** — remote shell/
screen over Raspberry Pi's own relay, not dependent on our tailnet or fleet server,
so a robot with a botched Tailscale key or a broken agent is still recoverable
without a physical keyboard. Tailscale is the fleet data plane; Pi Connect is the
"peace of mind" fallback, exactly as today.

MagicDNS handles reachability; `robot_id` handles identity; the legacy `auldbot`
hostname becomes irrelevant to the fleet layer.

---

### 4. Site/map registry — fleet server is source of truth, distribute by revision

**Recommendation.** The **fleet server is the canonical registry** of sites,
floors, and map revisions. This is barely new work, because Sites were *designed*
for it: the module docstring states the whole bundle is *"plain files + YAML so it
can be zipped, synced, or served by a web API without translation"*
(`sites.py:33-35`).

**Distribution = copy an immutable revision dir + one atomic flip.** A map revision
is a timestamped directory `floors/<floor>/maps/<rev>/`, *immutable once published*,
and the live map is a symlink flipped by an atomic `os.replace` (`sites.py:254-258`):

```python
tmp = fdir / f".map-{os.getpid()}"
os.symlink(os.path.join("maps", rev), tmp)
os.replace(tmp, fdir / "map")          # atomic publish
```

A robot pulling a new map = "download the `maps/<rev>/` dir, then flip the local
`map` link" — the existing revision model guarantees a **half-transferred revision
is never visible**, and rollback is a flip to an older rev (the exact semantics of
`site use-map <rev>`, `sites.py:432-441`). `KEEP_REVISIONS = 3` (`sites.py:60`)
bounds local disk.

**How does a robot know there's a new map? Event-driven, not polling.** The
registry publishes a **retained** MQTT topic per floor,
`mote/registry/site/<site>/floor/<floor>/current = <rev-id>`. The agent subscribes
to the floors its robot is entitled to; when the retained value changes it pulls
that revision and flips. Retained means a robot that was offline gets the current
value the instant it reconnects — no polling, no missed updates. (A robot on that
floor also gets a targeted `mote/<robot_id>/…` nudge so an active navigator can
reload cleanly rather than mid-goal.)

**Who may publish, and what validation runs.** A robot finishes mapping and runs
`save-map`, which already produces a complete local revision: `map.yaml`+`map.png`,
the raw PNG, the slam_toolbox posegraph, and a `meta.yaml` provenance record, and
**locally refuses to publish a revision unless all four artifacts are present**
(`sites.py:396-406`). The agent uploads that revision; the **fleet server
re-validates before making it canonical** — a small validation step in the registry
component, checking:

- all required files present and non-empty (server-side repeat of the local
  `sites.py:396-406` invariant, since the upload could truncate);
- `map.yaml` parses and its `resolution`/`origin`/frame are sane, image dimensions
  match, occupancy isn't degenerate (all-unknown / all-occupied);
- `meta.yaml` provenance is present (who/when/which bag).

If it passes, the server assigns it as the floor's **canonical** revision and
flips the retained topic; if not, it's rejected with a reason surfaced in the UI.
Publishing is **server-gated: one writer wins.** (Implementation note: the fleet
server is a container with no ROS, so this invariant should be an **extracted
torch/ROS-free validation module** shared with `sites.py` rather than logic
duplicated in two places — one small refactor to factor out at M4.)

**Conflict: two robots map the same floor.** Do **not** auto-merge. A map frame's
origin is *"an accident of where SLAM started, so zones/map/posegraph must live and
travel together"* (`sites.py` docstring); silently merging two frames breaks every
taught zone coordinate. Instead the server keeps both as **candidate revisions**,
and an operator **promotes** one to canonical — the same `site use-map` semantics,
centralised. Nothing is lost, nothing is silently merged, and the loser is retained
for audit (matching the raw-map-retention ethos already in `save-map`).

**`active.yaml` stays per-robot.** The *registry* of sites/floors/revisions is
central; which floor a given robot is on is local state (`sites.py:88-104`).

**Where the registry actually stores bytes (scaling).** At homelab scale it is the
fleet server's filesystem serving the bundle dirs over HTTP/rsync — literally the
on-disk layout `sites.py` already writes. At Regime B/C the same immutable-revision
model maps directly onto **object storage (S3/R2) + a metadata DB + a CDN**:
revisions are content-addressed dirs (already), the "flip" becomes updating a
pointer row, robots pull from the CDN. Because the bundle is "plain files + YAML"
by design, that swap is a storage-backend change behind the same registry API.

---

### 5. Live operations UI — adopt Foxglove for depth, build a thin fleet roster

**Recommendation: hybrid.** Adopt Foxglove for deep single-robot inspection;
**build only the fleet-level view Foxglove doesn't provide.**

- **Adopt Foxglove** (Q2) for the rich per-robot view: live pose on the floor map,
  camera peek, teleop, raw topic inspection.
- **Build a thin fleet dashboard** — the "minimum lovable" operator view — for the
  one thing Foxglove is not: the **fleet roster**. The browser is **read-only on the
  broker**: it subscribes over **MQTT-over-WebSockets** (Q2) — `mote/+/health` → the
  roster with online/current-task (and *battery* as a schema field, **future** —
  nothing in the tree publishes battery state yet), `mote/+/pose` → live positions —
  plus a **deep-link into Foxglove** per robot.
- **Dispatch goes through the fleet API, not straight to the broker.** The dispatch
  box POSTs to the fleet API, which authorizes the operator, writes an **audit
  record**, and *then* publishes `mote/<robot_id>/task/command`. Broker ACLs alone
  can't attribute or gate per-operator dispatch, and there's no audit trail if the
  command never passes through a mediating service — so the read path stays on the
  broker (cheap, live, no service in the middle) while the *write* path is mediated.
  The operator's broker credential is therefore subscribe-only (Q7).

**The fleet map is a proper interactive 2D map, not a static thumbnail.** The floor
PNG is only the *basemap*; the view on top of it is a pannable, zoomable 2D canvas
(Canvas2D at small scale, WebGL — e.g. deck.gl/PixiJS — once there are many markers)
with **pan, zoom, and click-to-follow-a-robot** as table stakes. A robot pose in the
map frame (metres) becomes a basemap pixel via the `map.yaml` `resolution` (m/px)
and `origin`:

```
px = (wx - origin_x) / resolution
py = height_px - (wy - origin_y) / resolution     # image y is top-down
```

and the render transform (pan/zoom) maps basemap pixels to screen. The same
coordinate transform places **zones** (areas/poses from the floor's `zones.yaml`),
**heading arrows** (from the pose quaternion), and robot markers. Marker *positions*
come off `mote/+/pose` over MQTT-over-WS at a modest rate; because those payloads are
tiny, **hundreds of robots is a rendering problem, not a data problem** — handled the
way every map UI handles it: marker **clustering** when zoomed out, viewport culling,
and WebGL instancing so thousands of markers stay smooth.

**Large sites** need the basemap itself to scale: a big floor's PNG is downsampled
for the zoomed-out view and **tiled** (or served at a couple of pyramid levels) so
the browser isn't decoding a huge image — the registry already stores immutable
revision dirs (Q4), so pre-rendering tiles/pyramid levels per revision is a natural
add. Multi-floor sites get a floor switcher (the Sites model is already
floor-scoped, `sites.py`).

**What still deep-links to Foxglove — and why that line is here, not further out.**
The fleet map owns the *2D fleet picture*: where every robot is, what it's doing,
follow one around, dispatch to it. It deliberately does **not** try to render 3D,
point clouds, live costmap/laser layers, or in-panel teleop — those are exactly what
Foxglove already does per-robot, so the map's per-robot "inspect" button deep-links
there. The split is "fleet-wide 2D situational awareness" (build it, and build it
well — pan/zoom/follow/cluster) vs "single-robot deep sensor view + teleop" (adopt
Foxglove).

The dashboard is thus a thin client over the MQTT control plane + the registry's
PNG maps. It is **not** a rebuild of Foxglove, and Foxglove is **not** asked to be
the fleet roster (it is per-connection, not a fleet database). Each tool does the
half it is good at.

---

### 6. Updates — one mechanism, the prefix.dev channel, install-alongside + rollback

**Recommendation.** Consume the **versioned prefix.dev `mote` channel** — already a
listed channel (`pixi.toml:3`), with the parallel Voro task productionizing
`pixi-build-ros` → that channel (`README.md`). The robot's software *is* a pixi
environment resolved from that channel, so an update is *"resolve a new pinned
version and re-activate"* — **one update mechanism**, satisfying the brief. (Today
there is no update mechanism at all — rsync-then-build-on-Pi, `pixi.toml:26`,
`mote-bringup.service`; this replaces it.)

**"Install-alongside" means two envs on *disk*, never two stacks *running*.** The Pi
does not have the CPU for two ROS stacks at once, and the hardware is exclusive
anyway (one process set holds the servo/lidar/camera serial ports — `mote_hardware`
opens the port in `on_activate`). So the new version is only *installed and staged*
while the old one runs; the cutover **stops the old stack, then starts the new one**.
That brief downtime is why updates are **scheduled when the robot is idle/charging,
not mid-mission** (the orchestrator holds the update until the agent reports idle).
"Keep the old env active" means kept
*installed on disk* for rollback, not kept *running*.

**Flow (per robot, driven by the agent):**

1. Agent reports **current version** on `mote/<robot_id>/ota/state`.
2. Operator sets a **target version for a rollout ring** (canary → stable).
3. **Stage (old stack still running):** agent installs the new version into a
   *second* pixi prefix pinned by the lockfile — disk only, no new processes, so no
   CPU/hardware contention with the running robot.
4. **Cutover (robot idle):** when the agent reports the robot idle, it **stops the
   old stack, flips the `current` pointer** to the new prefix (same
   install-beside-old-then-flip pattern as the map registry, `sites.py:254-258`),
   and **starts the new stack**, which now takes the hardware ports.
5. **Health-gate:** run a post-update health check on the new stack (does it launch?
   do controllers come up? a `sim-test`-style smoke gate). If it fails **or the robot
   doesn't check back in**, **stop the new stack and restart the old prefix** —
   rollback is starting the env we kept on disk.
6. Agent reports each transition — `idle → downloading → staged → stopping-old →
   activating → healthy | rolled-back`.

**Disk & network cost — real, and minimisable.** Two envs' worth of ROS + deps is
**gigabytes**, which is a genuine constraint on a Pi's SD/SSD, and pulling a full
env over WAN per robot is a lot of bytes.

- **Disk:** keep exactly **two slots** (current + one rollback), not N. And
  conda/pixi **hard-links identical packages across environments**, so a new env
  only costs disk for the packages that actually *changed* between versions — a
  point release is megabytes, not gigabytes.
- **Network:** conda fetches **per-package**, not whole-env, so an update downloads
  only changed packages (the lockfile diff), not the world. At Regime B/C, put a
  **channel mirror/CDN per site** so N robots don't each pull from prefix.dev over
  WAN — robots resolve against the local mirror.
- **Staging safety:** because staging is a separate prefix, a failed/half download
  never touches the running robot.

**Staged rollout.** Robots belong to a **ring** (canary/stable, and at multi-customer
scale, per-customer rings). The server holds a ring until the canary reports
`healthy` for N minutes, then advances — bookkeeping on the server keyed by
`robot_id`, not new robot code.

**Identity is safe across updates.** `~/.mote` is outside the package (Q3), so an
update *cannot* clobber identity, site selection, or maps — a free property of the
existing repo-vs-`~/.mote` split.

> **(verify)** prefix.dev package **signing** support. If available, verify
> signatures on-robot as the code trust root; if not, the trust root is
> account-scoped channel access + pinned lockfile hashes + the tailnet.

---

### 7. Security — proportionate, and mostly free from the substrate

**Recommendation.** Lean on the Tailscale substrate for the baseline and add
per-channel auth on top; do **not** build a PKI or a custom auth server for v1.

- **Transport encryption is free (WireGuard).** Every robot↔server, robot↔
  inference, operator↔robot link rides an encrypted WireGuard tunnel, and **nothing
  robot-side is exposed to the public internet** — no port forwarding. This deletes
  a whole class of risk and is what makes the **unauthenticated** inference wire
  (`depth_wire.py`) safe across the internet.
- **Authorization via Tailscale ACLs.** Tag robots `tag:robot`, servers
  `tag:fleet`; ACL so operators + fleet server reach a robot's Foxglove/MQTT ports,
  robots reach the broker + their local inference box, and **robots can't reach each
  other**. Free-tier ACLs cover this.
- **Per-channel auth on top of the tunnel:** MQTT → per-robot credentials
  (username = `robot_id`, publish only under its own prefix) or mTLS; the **operator
  broker credential** → **subscribe-only** ACL on `mote/+/health|pose` (the browser
  never publishes; dispatch is mediated by the fleet API, Q5); **dispatch authz +
  audit** → the Fleet API, which authorizes the operator and records the command
  before publishing `task/command`; Foxglove remote → device token; Fleet API/UI →
  operator auth (start with a token or GitHub/OIDC); Updates → pinned exact versions
  via the lockfile (+ signing if available, Q6).
- **Multi-tenant note (Regime C).** One flat tailnet does not isolate customers.
  The platform-scale answer is **per-customer tenancy** — separate Headscale
  tailnets or broker vhosts + topic-prefix ACLs per customer — so one customer's
  operator can never see another's robots. Designed-for, not built in v1.
- **Proportionality.** Small trusted fleet, not a public product: mTLS-everywhere
  and a custom PKI wait until the fleet runs *outside* the trusted overlay. Note
  Zenoh's TLS is *hop-by-hop, not end-to-end*
  ([Zenoh security analysis](https://census-labs.com/news/2025/03/17/zenoh-protocol-security-analysis/))
  — a reason the encryption baseline lives at the WireGuard layer (end-to-end
  between peers) rather than in an app-layer bridge.

---

## The other pipelines: fleet server & inference server

The robot pipeline is only one of three. The non-robot roles need their own
provisioning + update story, deliberately *different* from the robot OTA because they
are server software, not fleet-managed robots.

**Inference server (the GPU box) — a managed compute node.** Treat it as a **managed
node the fleet server drives with the same machinery as a robot, minus the ROS
bits**: it runs a tiny agent (the same `mote_agent` in a "compute node" mode) that
reports its version/health and executes staged updates on command, so it shows up in
the roster and its updates are orchestrated, gated, and reported — not done by hand
over SSH.
- *Provisioning:* install pixi, join the tailnet (tagged `tag:inference`), run the
  perception servers as a service — `pixi run inference` (Linux systemd) or the
  equivalent Windows service (this box is the Windows/NVIDIA machine the parallel
  inference-server Voro task productionizes; `pixi.toml` `inference`/`inference-rocm`
  envs).
- *Updates (automated blue/green):* **the same prefix.dev channel**, a *different
  package + env* (the torch inference server, no ROS). Because it is **stateless**
  (code + model weights only) it is *easier* to automate than the robot: the agent
  stands up the new version on a **second port**, runs a **probe request** as the
  health gate, and on success flips the served port (and pushes the new
  `inference_host` port to the robots that use it) — a true zero-downtime blue/green,
  with automatic rollback to the old port if the probe fails. No hardware exclusivity
  to work around (unlike the robot, Q6), so both versions really can run side by
  side during the check. Model-weight artifacts are versioned alongside the code.
- *Failure posture:* **not availability-critical** — the perception nodes idle
  harmlessly when the server is unreachable (`depth_wire.py:70-76`,
  `perception.yaml:10-12`), so a robot degrades (no depth-obstacle layer / no
  open-vocab detect) rather than failing. That is the safety net *under* the
  automation, not a substitute for it.

**Fleet server (broker + registry + API/UI).**
- *Provisioning:* infrastructure-as-code — a container-compose (Mosquitto/EMQX +
  registry service + web app) or a VM image. This is ordinary server ops, versioned
  in the repo, **not** driven through the robot OTA channel.
- *Updates:* standard container image deploy, ideally blue/green; the broker's
  retained state + the registry (files → object storage/DB) are the state to
  preserve and back up.
- *Failure posture:* it is the **availability-critical** component for *operations*
  (dispatch, roster, map publish), but **robot autonomy survives its downtime** —
  a robot keeps executing its current mission and navigating locally (Q1). So "fleet
  server down" degrades fleet *management*, not robot *safety*. HA becomes worth it
  at Regime B/C, not at homelab scale.

Net: **three pipelines, one release channel for the two ROS-software roles (robot,
inference) and normal server-deploy for the fleet server**, each with a failure
posture that keeps robot autonomy independent of fleet infrastructure.

---

## Scaling: one robot → ten thousand

The recommendations are tuned for the **homelab horizon**; this section is the cost
curve and the breaking points, and how the *seams* let us cross each regime without
a redesign.

| Component | A · Homelab (1–10, one site) | B · Small fleet (10s–100s, few sites, one org) | C · Platform (1k–10k, many sites & customers) |
|---|---|---|---|
| **Overlay** | Tailscale free/Starter | Tailscale Premium **or** self-hosted **Headscale** (per-device cost & vendor lock bite) | **Drop the full mesh.** Robots keep an **outbound TLS connection to a broker**; mesh/relay only for on-demand Foxglove/SSH; per-customer tailnets/Headscale |
| **Control plane** | Mosquitto (single node) | Mosquitto or single **EMQX** node | **EMQX/HiveMQ cluster** (millions of conns), per-region, per-customer vhosts |
| **Ops console** | Foxglove free (5-device cap = first wall) | Foxglove Pro (per-seat) | Foxglove Enterprise **or** Rerun-offline + custom live panel; multi-tenant fleet UI |
| **Registry** | Fleet-server filesystem over HTTP/rsync | Same, on a proper VM | **Object storage (S3/R2) + metadata DB + CDN** |
| **Fleet server** | A Pi / small VPS | A VM, backups | HA, multi-region, per-customer isolation |
| **Inference** | One shared GPU box | One GPU box per site (`inference_host` per robot) | One+ per site; autoscale; still LAN-local to robots |
| **Updates** | Pull from prefix.dev directly | Per-site channel mirror | CDN-mirrored channel, per-customer rings, delta packages |
| **Est. overlay cost** | ~free | ~$150/mo tags @200 robots (or run Headscale) | **~$10k/mo Tailscale tags @10k** → Headscale/broker-only makes the case |

**The three things that actually break at scale, and the pivot each forces:**

1. **Tailscale's per-tagged-device cost** (~$1/device/mo past 50). Fine to
   hundreds; ~$10k/mo at 10k. *Pivot:* self-host **Headscale**, or — better at
   platform scale — stop requiring a full mesh and have robots hold only an
   **outbound broker connection** (MQTT/TLS scales to millions and needs no mesh),
   using the relay only for interactive sessions.
2. **Mosquitto doesn't cluster** and **Foxglove's free device cap is 5**. *Pivot:*
   EMQX/HiveMQ cluster; Foxglove Pro/Enterprise or a custom multi-tenant console.
   Both are **swaps behind the same seams** (same MQTT topic tree, same ROS graph),
   not rewrites.
3. **`ROS_DOMAIN_ID` never scales as isolation** (safe range ~101, per-machine). It
   was never asked to — pinning discovery to localhost (Q3) means there is no
   cross-robot DDS graph at any scale, so this is a non-problem by construction.

**Why the pivots are cheap:** the v0/v1 seams were chosen for exactly this —
`robot_id` keys everything (not hostname), the topic tree is already per-robot,
site bundles are already object-storable immutable dirs, the agent is already the
sole egress, and both ROS roles already release through one channel. Crossing a
regime is replacing an *implementation* (Mosquitto→EMQX, filesystem→object storage,
Tailscale→Headscale/broker-only) behind a seam that doesn't move. That is the whole
reason the homelab design is allowed to be simple.

---

## Non-goals for v1

Explicitly **out of scope** for the first fleet, to keep each milestone shippable:

- **No multi-robot traffic coordination.** No centralised path planning, no
  deconfliction, no fleet traffic manager. Each robot navigates independently.
  (A large capability in its own right; deliberately deferred.)
- **No automatic map merging.** Two mappers → two candidate revisions → an operator
  promotes one (Q4). No auto-merge, ever.
- **No cross-robot task allocation/optimization.** An operator (or a trivial queue)
  assigns a task to a *specific* robot. No auctioning/bin-packing.
- **No telemetry time-series / cloud data lake.** MQTT carries *current* state; bags
  stay local (`~/.mote/bags/`, `sites.py`); Foxglove covers spot inspection.
- **No public-internet-exposed services, no multi-tenant isolation, no HA.**
  Everything on one tailnet, one org, single-node servers. (Regime B/C designed-for,
  not built.)
- **No safety-rated remote e-stop or high-rate teleop SLA over WAN.** Teleop is
  best-effort; all safety behaviour stays local on the robot.
- **No RMW swap.** `rmw_cyclonedds_cpp` stays; rmw_zenoh revisited at Kilted (Q2).

---

## Milestones

Each milestone is sized to be **one dispatchable Voro task**. Dependencies and
parallelism are called out so several can be dispatched at once. **Any milestone
that creates a surface others consume — a topic tree, a health payload, the dispatch
API, the bundle/registry API — must ship that interface as a documented, versioned
schema/spec, not an incidental implementation detail**, so dashboards, tooling, and
future agents build against the contract rather than the internals.

**Dependency graph** (→ = must-precede; siblings under one parent can run in
parallel):

```
M0 (overlay + identity) ─┬─> M1 (agent + control plane) ─┬─> M3 (fleet UI)
                         │                                └─> M4 (map registry)
                         ├─> M2 (Foxglove)      [parallel with M1]
                         └─> Ms (server pipelines) [parallel with M1/M2]
M1 ──> M5 (OTA)   [also needs the prefix.dev release task]
{M1,M3,M4} ──> M6 (second robot)
M7 (security hardening) : cross-cutting, folds into each; can start after M0
```

### v0 — one robot, fully remotely operable off-LAN

- **M0 · Overlay + identity foundation.** Tailscale on robot + workstation + fleet
  box; `~/.mote/robot.yaml` with an **operator-set `id`/`name`** (no server yet — the
  cloud-init provisioning path of Q3); pin DDS to localhost
  (`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` — already the default for sims and
  benchmarks; the robot is still LAN-discoverable and only pinned to its one
  interface by `config/cyclonedds.xml`, because nothing on-robot replaces an
  operator's RViz yet — M2 is what makes the localhost pin safe there); formalise
  per-robot (`~/.mote`) vs shared (package) config.
  *Accept:* clean-Pi → reachable by MagicDNS off-LAN; `robot_id` stable across
  reboots. *Depends on:* nothing. *Blocks:* M1, M2, Ms.
  *Seams:* `sites.py:63-104`, `pixi.toml:219-221`.

- **M1 · `mote_agent` + control plane + enrollment.** `mote-agent.service`;
  Mosquitto; the **enrollment endpoint + registry row store** and the server-first
  `mote enroll` allocation flow (Q3) that supersedes M0's operator-set id; agent
  publishes health/pose (LWT + retained, **defined health schema**) and bridges
  `task/command`↔`task/status` to `mote/<robot_id>/…` with the single-in-flight ack
  rule (Q1).
  *Accept:* enroll a robot and dispatch a `fetch`, observing status transitions,
  off-LAN, over MQTT; the **MQTT topic tree + health payload are published as a
  versioned schema**. *Depends on:* M0. *Blocks:* M3, M4, M5, M6. *Parallel with:*
  M2, Ms. *Seams:* `task_server.py:67-68`, `:1-15`; `mote_bringup/systemd/`.

- **M2 · Foxglove observability + teleop.** `foxglove_bridge` on the robot; a
  Foxglove layout for pose-on-PNG-map + camera peek + teleop.
  *Accept:* operator drives + watches one robot remotely.
  *Depends on:* M0 only. **Parallel with M1** (independent transport).
  *Seams:* PNG maps (`sites.py`), `/image_raw/compressed` (`CLAUDE.md`).

- **M3 · Thin fleet UI + dispatch API.** Web app: roster + health and per-robot
  map+pose overlay from the broker (**read-only**, MQTT-over-WS; coordinate transform,
  Q5); **dispatch through the fleet API** (authorizes the operator, writes an audit
  record, then publishes `task/command` — Q5/Q7); Foxglove deep-link.
  *Accept:* the "minimum lovable" operator view works for one robot, fully off-LAN;
  the **dispatch API is documented as a versioned spec** and the browser holds no
  broker publish rights. *Depends on:* M1. **← end of v0.**

### v1 — second robot enrolled, plus shared infrastructure

- **M4 · Central site/map registry + distribution.** Registry stores canonical
  bundles; agent pulls revisions (stage + atomic flip) on the retained
  `…/current` MQTT signal and uploads new revisions from `save-map`; **server-side
  validation** (Q4); operator promotes a floor's active revision.
  *Accept:* map a floor, publish, robot re-pulls canonical; a second candidate is
  retained, not merged; the **bundle/registry API is documented as a versioned
  schema**. *Depends on:* M1. *Parallel with:* M2, M5, Ms.
  *Seams:* `sites.py:254-258`, `:348-410`, `:396-406`, `:432-441`.

- **M5 · OTA updates via prefix.dev.** Agent reports version; server drives ring
  rollout; install-alongside (two slots, hardlink-dedup) + health-check + rollback;
  report update state.
  *Accept:* push a version to a canary; auto-rollback on failed health check; stable
  ring advances only after canary healthy. *Depends on:* M1 **and** the parallel
  prefix.dev release task producing versioned packages — specifically a
  **linux-aarch64** robot build for the Pi. *Parallel with:* M4.
  *Seams:* `pixi.toml:3`, the `~/.mote`-vs-package split (Q3).

- **Ms · Server pipelines.** Provisioning + update story for the inference server
  (same channel, own env, stateless blue/green) and the fleet server (IaC container
  deploy). *Accept:* rebuild either server from scratch via its documented pipeline;
  robot autonomy unaffected while the fleet server is down. *Depends on:* M0.
  **Parallel with M1/M2/M4.** *Seams:* `pixi.toml` inference envs; `depth_wire.py:70-76`.

- **M6 · Second robot enrollment (multi-robot hardening).** Enroll `mote-02`;
  verify per-robot MQTT namespacing, localhost-pinned DDS (two robots on one LAN
  don't cross-talk), registry sharing; fleet UI shows two; exercise the two-mapper
  conflict/promote flow.
  *Accept:* two robots dispatched independently from one UI. *Depends on:* M1, M3,
  M4. **← v1 complete.**

- **M7 · Security hardening (cross-cutting).** Tailscale ACL tags; per-robot broker
  credentials / mTLS; Foxglove tokens; operator auth on the fleet API; pinned (+
  signed if available) packages. *Accept:* a device off the tailnet reaches nothing;
  a robot can't read another robot's command topic. *Depends on:* M0; folds into
  each milestone as it lands.

---

## Appendix — verification ledger

Claims grounded in the repo (cited inline) are verified. External claims flagged
**(verify)** before building on them:

- **CycloneDDS participant cap under localhost discovery** — `MaxAutoParticipantIndex`
  defaults to ~32 discoverable processes/host; a full stack may approach it. Count
  participants on the robot; bump via `CYCLONEDDS_URI` if needed (Q3). *Q3's whole
  answer leans on this.*
- Tailscale free-tier limits + **tagged-device pricing** ($1/device/mo past 50) —
  confirm current at adoption; it drives the Regime-C cost pivot.
- Foxglove free-tier caps (3 users / 5 devices / 10 GB) and teleop-over-WS specifics.
- EMQX/HiveMQ cluster scale figures (millions of connections) — vendor benchmarks.
- prefix.dev package **signing** support — determines the OTA code trust root
  (Q6/M5).
- conda/pixi cross-env **hardlink dedup + per-package fetch** behaviour — the basis
  of the OTA disk/network minimisation (M5); confirm on the Pi's filesystem.
- rmw_zenoh Jazzy production status — re-check at Kilted before reconsidering the
  RMW swap.

Resolved during review (struck from the list above): `ros-jazzy-foxglove-bridge`
3.3.0 exists on robostack-jazzy with a linux-aarch64 build; `ROS_AUTOMATIC_DISCOVERY_RANGE`
is supported by rmw_cyclonedds ([rmw_cyclonedds#429](https://github.com/ros2/rmw_cyclonedds/pull/429),
in Iron/Jazzy).
