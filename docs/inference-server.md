# The inference server

Mote keeps torch off the robot. The heavy vision models (monocular depth today,
open-vocabulary detection now, SfM/policy inference later) run in a separate
process reached over a plain TCP socket, so the robot's ROS environment stays
light and the compute can live wherever the GPU is. This document is about
running that process as a **first-class role on a dedicated machine** — a Windows
gaming PC with an NVIDIA GPU — that the whole robot talks to, and that the robot
degrades gracefully without.

The mechanism (the two-process split, the `depth_wire`/`detect_wire` protocol,
the on-robot nodes) is described in [`mote_perception/README.md`](../mote_perception/README.md).
This document adds the *deployment*: host choice, the CUDA env, auto-start,
health, the multi-service pattern, the fallback matrix, and how to measure it.

---

## Host decision: native Windows, not WSL2

The target is a Windows gaming PC with an NVIDIA GPU that also stays a gaming PC.
Two ways to host a Linux-flavoured torch stack there — native Windows or WSL2 —
were weighed against the things that actually matter for this role:

| Criterion | Native Windows (chosen) | WSL2 |
|---|---|---|
| **CUDA torch in a pixi env** | ✅ `win-64` feature, torch `cu128` wheel from pytorch.org solves cleanly (verified: `torch-2.11.0+cu128-cp312-cp312-win_amd64.whl`) | ✅ `linux-64`, torch `cu128` linux wheel |
| **LAN reachability from the robot** | ✅ server binds `0.0.0.0` straight onto the PC's LAN IP; robot connects directly | ⚠️ WSL2 is NAT'd behind the host — needs `netsh interface portproxy` or mirrored-networking mode to expose the socket to the LAN |
| **Auto-start on boot** | ✅ Task Scheduler "at startup" runs the runner in session 0; CUDA compute needs no desktop | ⚠️ needs WSL auto-launch + systemd-in-WSL + the host waking the distro; more moving parts |
| **Stays a gaming PC** | ✅ a background scheduled task; no VM | ✅ but a running WSL VM alongside |
| **Operational simplicity** | ✅ one pixi env, one task, logs in `%LOCALAPPDATA%` | ⚠️ two OSes to reason about when something breaks |

Native Windows wins on the two that bite in practice — the robot reaching the
socket, and the servers coming back after a reboot — without giving up the easy
CUDA torch path. WSL2's only real edge (matching the existing Linux tooling) is
outweighed by its NAT layer sitting between the robot and the server. Dual-boot
Linux was out of scope and unnecessary since both options solved.

**So: the inference PC runs native Windows, pixi's `inference-cuda` env, torch
from the CUDA wheel index.** The pixi feature declares `platforms = ["win-64"]`
in its own environment, so this never perturbs the robot/dev/sim solves (the
aarch64 robot env is byte-for-byte unchanged — verified in `pixi.lock`).

If the win-64 torch solve ever breaks (a torch release skips Windows wheels, say),
the fallback is WSL2 with the `inference-rocm`-style pattern retargeted to a linux
`cu128` index, plus a `portproxy` rule — but that is the contingency, not the plan.

---

## Setup guide (run once at the PC)

Everything below runs in PowerShell on the gaming PC. The scripts live in
[`mote_perception/deploy/windows/`](../mote_perception/deploy/windows) and are
parameterised, so a PC rebuild is: clone the repo, run these, done.

1. **Get the repo.** Install git if needed, then clone Mote somewhere stable,
   e.g. `C:\mote`. (Only the repo is needed; no ROS on this machine.)

2. **Set up the env + models** — a normal (non-admin) PowerShell:

   ```powershell
   cd C:\mote
   .\mote_perception\deploy\windows\setup.ps1
   ```

   This installs pixi if missing, solves and installs the `inference-cuda` env
   (first run downloads the ~2–3 GB torch CUDA wheel — be patient), prints a GPU
   check (`cuda_available True`, the device name), and pre-fetches the depth +
   detect model weights into the HuggingFace cache so the first request doesn't
   block on a download.

3. **Smoke-test by hand** (optional but recommended):

   ```powershell
   pixi run inference-cuda
   ```

   You should see `using GPU: <your card>` for depth and detect, and both
   "listening on 0.0.0.0:5601 / :5602". Leave it running and, from the robot:

   ```bash
   pixi run inference-health --host <gaming-pc-hostname-or-ip>
   ```

   Expect `depth  UP ... on cuda (<card>)` and `detect UP ... on cuda (<card>)`.
   Ctrl+C the server when satisfied.

4. **Install boot auto-start** — an **elevated** (Administrator) PowerShell:

   ```powershell
   .\mote_perception\deploy\windows\install_service.ps1
   ```

   It registers a `MoteInference` scheduled task that runs the servers at every
   boot (whether or not anyone logs in) and prompts for the account password so
   Task Scheduler can run it unattended. Remove it later with
   `uninstall_service.ps1`.

5. **Point the robot at the PC** — see the next section.

That's it. Reboot the PC and confirm `pixi run inference-health --host <pc>` from
the robot answers with no manual step on the PC.

---

## Pointing the robot at the server

The single deployment knob is **`inference_host`** in
[`mote_perception/config/perception.yaml`](../mote_perception/config/perception.yaml).
Set it to the gaming PC's stable hostname (or LAN IP):

```yaml
inference_host: mote-gpu   # the gaming PC on the LAN
depth:  { enabled: true, server_port: 5601 }
detect: { enabled: true, server_port: 5602 }
```

Override per-robot without editing the committed file by dropping the same keys in
`~/.mote/perception.yaml` (same precedence as the camera calibration). This config
lives here, **not** in `robot.yaml`: `robot.yaml` is hardware/description (wheels,
servos, sensor device paths) and `perception.yaml` is perception *runtime* — the
two are deliberately separate config surfaces. No discovery protocol is invented;
a stable hostname is the contract. Give the PC a DHCP reservation or a hosts entry
so its name is stable.

Ports are per-service and fixed (depth 5601, detect 5602); the robot node for each
service carries its own port, so services are independent.

---

## Production behaviors

- **Auto-start on boot** — the `MoteInference` scheduled task (step 4) launches
  [`run_inference.ps1`](../mote_perception/deploy/windows/run_inference.ps1),
  which runs `pixi run inference-cuda` in a supervise loop and restarts it a few
  seconds after any exit. So a server crash *or* a reboot both recover with no
  human at the PC.

- **Reconnect across restarts, both ends** — the robot nodes use a persistent
  socket that reconnects lazily: any failure (server down, connection dropped,
  server restarted) tears the socket down and the next frame reconnects, warning
  once (throttled) meanwhile. Nothing on the robot needs restarting when the
  server bounces. This is `WireClient` in `depth_wire.py` and is covered by
  `test_depth_wire.py::test_depth_client_reconnects_after_server_drop` and the
  health round-trip tests. On the server side, each connection is independent and
  a bad frame never kills the server.

- **Health / version check from the robot** — `pixi run inference-health [--host H]`
  probes each service over the same socket (the `HEALTH_MAGIC` request in the wire
  protocol) and prints the model, device, GPU, and torch version the server is
  actually running, or `DOWN` if it can't be reached. Exit status is non-zero if
  any service is down, so it doubles as a scriptable gate. `--json` for machine
  output.

- **Logs somewhere findable** — the runner tees everything (its own lifecycle
  plus both servers' per-frame lines) to `%LOCALAPPDATA%\mote\logs\inference-<date>.log`
  on the PC. Each server also prints `health check` when probed, so you can see
  the robot reaching it.

---

## Multi-service pattern (adding the next tenant)

Depth and detect are already two tenants of this role, and a third (SfM, a policy
server, …) is a config exercise, not a redesign. The seam is
[`inference_server.py`](../mote_perception/tools/inference_server.py), the
cross-platform supervisor the `inference*` tasks run:

```python
SERVICES = [
    ("depth",  "depth_server.py",  []),
    ("detect", "detect_server.py", []),
]
```

To add a tenant:

1. **Write the server** in `mote_perception/tools/`, following `depth_server.py`:
   load the model, `listen(1)`, and in the per-connection loop read the leading
   `uint32` — if it equals `HEALTH_MAGIC`, `send_health(conn, info)` and continue;
   otherwise read your request and reply. Reuse the framing helpers in a
   `*_wire.py` module.
2. **Give it a wire module** (`mycompute_wire.py`) with a `DEFAULT_PORT` (next
   free port, e.g. 5603), the request/reply framing, and a `Client(WireClient)`
   subclass — it inherits `connect`/`close`/**`health`**/reconnect for free.
3. **Add one row to `SERVICES`** above. It now inherits `0.0.0.0` binding,
   supervision (if it dies, the others are torn down so the failure is visible),
   teardown, boot auto-start, and the health probe automatically.
4. **On the robot**, add its node to `perception_launch.py` and a
   `mycompute: { enabled, server_port }` block to `perception.yaml`, exactly like
   `depth`/`detect`.

Port allocation is manual and documented here (5601 depth, 5602 detect, 5603+ for
new services) — a fixed small map beats a discovery protocol for a handful of
services on one box. Health and reconnect are free because they live in the shared
`WireClient`/`send_health`, not per-service.

---

## Fallback matrix (server present / absent)

The robot must keep working when the inference PC is off, asleep, or unreachable.
It does, because the depth/detect nodes are torch-free and treat "no server" as
"skip this frame", never as a fatal error. Navigation runs on lidar; the camera
obstacle layer is an *additive* near-band voxel layer, so losing it degrades
obstacle coverage but never stops nav.

| Situation | What runs the depth model | `/camera_obstacles` | Navigation |
|---|---|---|---|
| **Gaming PC up** (`inference_host: mote-gpu`) | NVIDIA CUDA — the fast path | published normally | full: lidar + camera near-band |
| **Gaming PC down / unreachable** | nothing — node warns (throttled 2 s) and skips each frame; publisher stays alive, publishes nothing | silent (no points) | **unaffected** — runs on lidar alone |
| **No GPU box, fall back to the dev machine** (`inference_host: <dev>`, `pixi run inference-rocm` or `pixi run inference`) | AMD ROCm iGPU, or CPU | published (slower) | full, at reduced depth rate |

The current code's behavior in the "down" case is **warn-and-skip, not disable**:
`DepthClient.infer` / `DetectClient.infer` return `None` on any socket failure and
the node returns early (`depth_obstacle_node._on_image`), so the topic simply goes
quiet and resumes automatically when the server returns — no relaunch. Verified by
`test_depth_client_unreachable_returns_none_and_warns` and the reconnect test.

To *intentionally* disable a service (e.g. the PC is gone for a while and you want
the logs quiet), set `depth.enabled: false` / `detect.enabled: false` in
`perception.yaml` and relaunch `pixi run perception` — the node isn't created at
all. Leaving it enabled with no server is harmless (it idles, only working when a
subscriber is present).

---

## Measuring it

Two committed harnesses; results live under
[`mote_perception/benchmarks/`](../mote_perception/benchmarks) so they can be
compared across machines and over time.

- **`pixi run inference-bench`** — client-side, torch-free, run from the robot (or
  any LAN machine). Times the full round trip the node pays — compress → send →
  server infer → receive — so the number includes GPU time *and* the network hop,
  comparable to the on-robot pipeline. Example:

  ```bash
  pixi run inference-bench --host mote-gpu --image sample.jpg --frames 200 \
      --out mote_perception/benchmarks/depth_cuda_lan.json
  ```

  It prints a percentile table (min/p50/mean/p90/p99/max ms + fps) and writes the
  raw samples as JSON. Use `--service detect --labels "red box"` for the detector.

- **The server's own per-frame log** (`served WxH in N ms`) isolates pure model
  time on the GPU, so `inference-bench`'s round-trip minus the server's reported
  time is the LAN + transport overhead.

See [`benchmarks/README.md`](../mote_perception/benchmarks/README.md) for the
baseline numbers (#152 CPU / ROCm iGPU) and how to fill in the gaming-PC CUDA
results.
