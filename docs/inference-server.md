# The inference server

Mote keeps torch off the robot. The heavy vision models (monocular depth today,
open-vocabulary detection now, SfM/policy inference later) run in a separate
process reached over a plain TCP socket, so the robot's ROS environment stays
light and the compute can live wherever the GPU is. This document is about
running that process as a **first-class role on a dedicated machine** that the
whole robot talks to, and that the robot degrades gracefully without.

The mechanism (the two-process split, the `depth_wire`/`detect_wire` protocol,
the on-robot nodes) is described in [`mote_perception/README.md`](../mote_perception/README.md).
This document adds the *deployment*: how it ships, how it starts, how it updates,
health, the multi-service pattern, the fallback matrix, and how to measure it.

---

## It ships as a container

The inference machine is **not a development environment**. It is a Windows
gaming PC, or a Linux box, or a rented cloud GPU — a machine whose owner should
not have to install a toolchain, clone a repo, or run a setup script to lend the
robot its GPU. So the server ships as a container image and the host needs
exactly two things: an NVIDIA driver and a container runtime.

That also makes the role portable in a way the alternatives were not. The same
image is the answer for a contributor with no Windows PC who wants to rent a
cloud GPU for an afternoon, and for a permanently-installed box on the LAN.

**Host options considered:**

| | Container (chosen) | Native Windows + pixi | WSL2 + pixi |
|---|---|---|---|
| **What the host installs** | ✅ a container runtime | ❌ pixi, a git checkout, PowerShell scripts | ❌ pixi, a checkout, plus a WSL distro |
| **GPU access** | ✅ `--gpus all` (Docker Desktop bundles the NVIDIA toolkit; on Windows this runs over WSL2, but as an installer detail you never touch) | ✅ direct | ✅ via the WSL2 CUDA driver |
| **Reaching it from the robot** | ✅ `-p 5601:5601` publishes on the host interface | ✅ binds the LAN directly | ⚠️ NAT'd — needs `portproxy` or mirrored networking |
| **Works on a cloud GPU** | ✅ same artifact, no changes | ❌ Windows-only | ❌ awkward |
| **Update** | ✅ `docker pull` | ❌ `git pull` + re-solve on the host | ❌ same |
| **Reproducibility** | ✅ a frozen, versioned image | ⚠️ `pixi.lock` pins deps, but the host supplies the rest | ⚠️ same |
| **Cost** | ⚠️ a multi-GB image to build in CI and pull once | ✅ none beyond pixi | ✅ none beyond pixi |

The native-pixi paths were implemented first and then removed: they required the
inference machine to become a small dev environment, which is exactly what the
owner of a gaming PC does not want. The container's one real cost — a few GB of
image, built in CI and pulled once — buys a host that stays a gaming PC.

pixi is still how inference runs **for development** on Linux (`pixi run
inference`, `pixi run inference-rocm`). The container is how it runs as a
*deployed role*. Those are different jobs and it is fine that they use different
tools; nothing on the robot changes either way.

---

## Setup: one command

On the inference machine, install a container runtime with GPU support — on
Windows that is [Docker Desktop](https://docs.docker.com/desktop/) (free for
personal use; tick *Start Docker Desktop when you log in*), on Linux Docker plus
the NVIDIA Container Toolkit. Then, once:

```
docker run -d --gpus all --restart unless-stopped \
  -p 5601:5601 -p 5602:5602 \
  --name mote-inference \
  ghcr.io/clachdev/mote-inference:latest
```

That is the entire deployment. No repo, no scripts, no files on the host.
`--restart unless-stopped` means it comes back whenever the Docker daemon starts,
so it survives reboots and crashes on its own.

Confirm from the robot:

```bash
pixi run inference-health --host <inference-machine>
```

Expect `depth UP` and `detect UP` with the GPU name and the image's version.

> **Windows: boot vs login.** Docker Desktop is a desktop application — it starts
> when you *sign in*, not at boot. After a reboot the container comes back once
> someone logs into the PC. For a personal machine that is normally fine; if you
> ever need it up before login, that needs Docker Engine inside WSL2 started by a
> scheduled task, which puts a setup script back on the host.

---

## On-demand GPU: the model is not resident when idle

A gaming PC should not have ~1 GB of VRAM pinned by a robot that is parked, so the
servers load their model on the **first request** and release it after
`--idle-timeout` seconds (default 300) with no traffic. Serving continuously
never unloads; serving after an idle period pays a few seconds for the reload.

This lives in the server ([`tools/model_host.py`](../mote_perception/tools/model_host.py)),
not in the deployment, so it behaves identically on a gaming PC, a Linux box, and
a cloud instance — no socket activation, wake proxy, or orchestration. The health
blob reports `loaded`, so "up but cold" is a visible, normal state rather than
something that looks like a hang.

To keep a truly dedicated box always-resident, pass `--idle-timeout 0`:

```
docker run ... ghcr.io/clachdev/mote-inference:latest --idle-timeout 0
```

Arguments after the image name pass through to both servers.

---

## Updating

```
docker pull ghcr.io/clachdev/mote-inference:latest
docker rm -f mote-inference
docker run -d --gpus all --restart unless-stopped -p 5601:5601 -p 5602:5602 \
  --name mote-inference ghcr.io/clachdev/mote-inference:latest
```

Two commands and a re-run, or the equivalent buttons in Docker Desktop's GUI. CI
([`inference-image.yml`](../.github/workflows/inference-image.yml)) builds and
pushes to GHCR when the server files change on `main`, on a tag, or on demand;
`latest` tracks main and `sha-` / version tags let a host pin a known-good build
and roll back.

**Version skew is visible, not silent.** The robot and the server share the wire
protocol modules, so they must move together. CI bakes the build's revision into
the image as `MOTE_VERSION`, each server reports it in its health blob, and
`pixi run inference-health` compares it against the robot's own:

```
inference host: mote-gpu   (this machine: a1b2c3d)
  depth   UP   ...  @ v0.1.0-42-g6aa919f
  detect  UP   ...  @ v0.1.0-42-g6aa919f

WARNING: version skew — this machine is at a1b2c3d, but depth is at ...
```

In practice the protocol changes rarely and additively — the health request was
added without breaking existing clients, since an unknown leading `uint32` could
never be confused with an image length — but when it does change, this turns a
confusing protocol error into a one-line diagnosis.

---

## Scaling to a cloud GPU

The image runs unchanged on a cloud GPU instance, which is the practical option
for anyone without an NVIDIA machine. Two things change, and both matter:

**1. The wire protocol is unauthenticated and unencrypted.** That was a
deliberate choice for one hop on a trusted LAN, and it is documented as such in
`depth_wire.py`. On a public cloud instance, publishing 5601/5602 means anyone
who finds the port can use your GPU and read the images your robot sends. The
tailnet is what makes "trusted network" true again off-LAN — but on a rented box
the safest shape is to **put the container itself on the tailnet and publish
nothing on the host**, using a Tailscale sidecar:

```yaml
# docker-compose.yml — the inference container has no ports of its own; it shares
# the sidecar's network namespace, so it is reachable *only* over the tailnet.
services:
  tailscale:
    image: tailscale/tailscale:latest
    environment:
      TS_AUTHKEY: ${TS_AUTHKEY}          # tagged, e.g. tag:inference
      TS_HOSTNAME: mote-inference        # becomes the MagicDNS name
      TS_STATE_DIR: /var/lib/tailscale   # persist, or every restart makes a new node
      TS_USERSPACE: "true"
    volumes: [ts-state:/var/lib/tailscale]
    restart: unless-stopped
  inference:
    image: ghcr.io/clachdev/mote-inference:latest
    network_mode: service:tailscale      # no `ports:` — nothing on the host
    deploy:
      resources: {reservations: {devices: [{driver: nvidia, count: all, capabilities: [gpu]}]}}
    restart: unless-stopped
volumes: {ts-state: {}}
```

This is stronger than binding to the tailnet address by hand: there is no
host-published port to get wrong, no bind-ordering race at boot, and the instance
never has to be permanently enrolled — the *container* is the tagged
`tag:inference` node with its own MagicDNS name, which is exactly the identity
model M0 defines. Set `inference_host` to `TS_HOSTNAME` and the robot is
unchanged. (The same pattern works on a Linux GPU box; on a personal Windows PC
the simpler host-Tailscale route above is usually the better trade.)

Adding a shared-secret token to the handshake would be the next step if the
socket ever needed to face a hostile network directly; it is deliberately not
built, because the tailnet solves the problem without complicating a protocol
whose value is that you can debug it with a hexdump.

**2. Bandwidth, not latency, is the limit.** Each frame sends ~50 KB up but the
depth reply is `640×480×4` = **1.2 MB down**. On a LAN that is nothing; over
broadband it dominates the round trip. Note that the pipeline stamps clouds at
*image-capture* time and Nav2 places them via tf, so added latency costs
**rate**, not correctness — obstacles land in the right place, just less often.
If cloud use becomes routine, sending float16 depth would halve the payload for
no meaningful precision loss at these ranges; `pixi run inference-bench` measures
exactly this, so the decision can be made on numbers.

This is why `docs/design/fleet.md` sets the placement rule: **keep the inference
server on the same LAN as the robots it serves**, and treat the tailnet as what
makes an *occasional* cross-site fallback possible rather than the normal path.
Per-robot `inference_host` already expresses that — each robot points at its
local GPU box.

---

## Multi-service pattern (adding the next tenant)

Depth and detect are already two tenants, and a third (SfM, a policy server, …)
is a config exercise, not a redesign. The seam is
[`inference_server.py`](../mote_perception/tools/inference_server.py), the
supervisor the container runs:

```python
SERVICES = [
    ("depth",  "depth_server.py",  []),
    ("detect", "detect_server.py", []),
]
```

To add a tenant:

1. **Write the server** in `mote_perception/tools/`, following `depth_server.py`:
   load the model through a `ModelHost` (so it inherits on-demand loading),
   `listen(1)`, and in the per-connection loop read the leading `uint32` — if it
   equals `HEALTH_MAGIC`, `send_health(conn, info)` and continue; otherwise read
   your request and reply.
2. **Give it a wire module** (`mycompute_wire.py`) with a `DEFAULT_PORT` (next
   free port, e.g. 5603), the request/reply framing, and a `Client(WireClient)`
   subclass — it inherits `connect`/`close`/**`health`**/reconnect for free.
3. **Add one row to `SERVICES`**, plus its port to the `EXPOSE` line and the
   documented `docker run`. It now inherits binding, supervision, restart,
   on-demand loading, and the health probe.
4. **On the robot**, add its node to `perception_launch.py` and a
   `mycompute: { enabled, server_port }` block to `perception.yaml`, exactly like
   `depth`/`detect`.

Port allocation is manual and documented here (5601 depth, 5602 detect, 5603+
next) — a fixed small map beats a discovery protocol for a handful of services.

---

## Pointing the robot at the server

The single deployment knob is **`inference_host`** in
[`mote_perception/config/perception.yaml`](../mote_perception/config/perception.yaml):

```yaml
inference_host: mote-gpu   # the inference machine
depth:  { enabled: true, server_port: 5601 }
detect: { enabled: true, server_port: 5602 }
```

Override per-robot in `$MOTE_HOME/perception.yaml` (`~/.mote` by default), the
per-robot state root M0 formalised — same precedence as the camera calibration,
resolved through `mote_bringup.mote_home.override()` so an update can never
clobber it. `pixi run inference-health` resolves the same way, so the probe always
reads the file the nodes were launched with. This lives here, not in `robot.yaml`:
`robot.yaml` is shared hardware description, `perception.yaml` is perception
runtime (and `$MOTE_HOME/robot.yaml` is a third thing again — this robot's
identity). No discovery
protocol is invented — a stable hostname is the contract.

**Use the machine's MagicDNS name.** Since M0 every robot, workstation and GPU box
joins one Tailscale tailnet (`docs/fleet/README.md`), so `inference_host` should be
the inference machine's tailnet name rather than a LAN IP — that is what
`docs/design/fleet.md` specifies, and it means the name stays correct whether the
robot reaches the box over the LAN or from another site. It also removes the need
for a DHCP reservation or a hosts entry.

## The tailnet and the container

Tailscale runs on the **host**; the container does not need to know it exists.
Docker publishes the ports onto the host's interfaces, Tailscale gives the host a
MagicDNS name, and the robot connects to that name — nothing in the image, the
protocol, or the perception code changes. Three practical points:

**1. `pixi run tailnet` is Linux-only.** `mote_bringup/tailscale/install.sh` is
`curl | sh` plus `systemctl`, so it covers robots, a Linux GPU box and the fleet
server — not a Windows PC. There, install the Tailscale Windows app and sign in;
the result is the same tailnet node.

**2. Leave a personal PC an untagged workstation.** The roles script offers
`--role inference` (`tag:inference`), but advertising a tag *transfers the node
from your account to the tailnet* — the M0 runbook already makes this call for a
co-located dev machine, and it applies at least as strongly to a gaming PC you
own. Nothing functional is lost: robots reach it by MagicDNS either way and
`inference_host` is just a name. What you defer is M7's ACLs keying on
`tag:inference` rather than your user. Tag it when the box outlives your account.

**3. `-p 5601:5601` publishes on *every* interface, not just the tailnet.** On a
home LAN behind a router that is the same trusted-network posture the wire
protocol already assumes, so it is fine. To restrict it to the tailnet, bind the
published port to the Tailscale address:

```
docker run -d --gpus all --restart unless-stopped \
  -p 100.x.y.z:5601:5601 -p 100.x.y.z:5602:5602 ...
```

Note the ordering hazard: after a reboot Docker can start before Tailscale has
assigned that address, and the bind fails. `--restart unless-stopped` retries
with backoff so it recovers on its own, but if that bothers you, the sidecar
pattern below avoids the problem entirely by never publishing on the host.

---

## Fallback matrix (server present / absent)

The robot must keep working when the inference machine is off, asleep, or
unreachable. It does: the depth/detect nodes are torch-free and treat "no server"
as "skip this frame", never as a fatal error. Navigation runs on lidar; the
camera obstacle layer is an *additive* near-band voxel layer, so losing it
degrades obstacle coverage but never stops nav.

| Situation | What runs the model | `/camera_obstacles` | Navigation |
|---|---|---|---|
| **Inference machine up** | NVIDIA CUDA in the container — the fast path | published normally | full: lidar + camera near-band |
| **Machine down / unreachable / not logged in** | nothing — node warns (throttled 2 s) and skips each frame; publisher stays alive | silent (no points) | **unaffected** — runs on lidar alone |
| **Model idle-released** | reloads on the next frame (a few seconds) | brief gap, then normal | unaffected |
| **No GPU box at all** | dev fallback: `pixi run inference-rocm` (AMD iGPU) or `pixi run inference` (CPU) on a Linux machine | published (slower) | full, at reduced depth rate |

The "down" case is **warn-and-skip, not disable**: `DepthClient.infer` /
`DetectClient.infer` return `None` on any socket failure and the node returns
early, so the topic goes quiet and resumes automatically when the server returns
— no relaunch. Verified by `test_depth_client_unreachable_returns_none_and_warns`
and the reconnect test.

To *intentionally* disable a service, set `depth.enabled: false` /
`detect.enabled: false` in `perception.yaml` and relaunch `pixi run perception`.
Leaving one enabled with no server is harmless.

---

## Measuring it

Two committed harnesses; results live under
[`mote_perception/benchmarks/`](../mote_perception/benchmarks).

- **`pixi run inference-bench`** — client-side, torch-free, run from the robot or
  any machine that can reach the server. Times the full round trip the node pays
  — compress → send → infer → receive — so the number includes GPU time *and*
  the network hop:

  ```bash
  pixi run inference-bench --host mote-gpu --image sample.jpg --frames 200 \
      --out mote_perception/benchmarks/depth_cuda_lan.json
  ```

  Prints a percentile table (min/p50/mean/p90/p99/max ms + fps) and writes the raw
  samples as JSON. `--service detect --labels "red box"` for the detector. Note
  the first timed frame after an idle release includes the model load — use
  `--warmup` (default 5) to exclude it, or `--idle-timeout 0` on the server.

- **The server's per-frame log** (`served WxH in N ms`) isolates model time, so
  the bench round trip minus that is transport overhead.

See [`benchmarks/README.md`](../mote_perception/benchmarks/README.md) for the
baselines and how to fill in results.
