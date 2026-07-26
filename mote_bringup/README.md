# mote_bringup

Launch files, config, udev rules, NetworkManager drop-ins, and systemd services
for the robot. The package layout and launch hierarchy are documented in the
top-level `CLAUDE.md`; this README covers the **on-robot reliability stack** —
how the robot survives unattended operation.

## Starting the robot

**By hand (the normal way):** `pixi run robot` (nav) or `pixi run mapping`. Both
include the health monitor, so `/health` and `/diagnostics_agg` are published on
any manual run — one command, nothing else to start.

**Unattended:** the systemd units below. They are installed by `pixi run setup`
but **not enabled**, because starting the drive stack and recorder on every boot
drains the battery of a robot that is just sitting on a desk, and the recorder's
pruner trims older bags whenever it runs. Opt in per robot:

```bash
sudo systemctl enable --now mote-bringup mote-health   # autostart at boot
sudo systemctl disable mote-bringup mote-health        # back to manual
```

## systemd services

Installed by `pixi run setup` (→ `systemd/install.sh`), which fills in the
invoking user/home/repo. Boot order once enabled:

```
mote-bringup  →  mote-slam  →  mote-nav
  │  (self-check runs here      ↘  mote-record
  │   as ExecStartPre)          ↘  mote-health
```

`mote-bringup` runs the self-check as its `ExecStartPre` gate — there is no
separate self-check service; gating bringup gates everything downstream.

| Service          | Runs                | Notes |
|------------------|---------------------|-------|
| `mote-bringup`   | `pixi run launch health:=false` | Hardware base. `ExecStartPre` runs the self-check gate first; `health:=false` because `mote-health` runs the monitor separately here. |
| `mote-slam`      | `pixi run slam`     | `BindsTo`/`PartOf` bringup — restarts with it. |
| `mote-nav`       | `pixi run nav`      | `BindsTo`/`PartOf` slam. |
| `mote-record`    | `pixi run record`   | `After`/`PartOf` bringup (no `Wants` — see below); a recorder crash never takes down the drive stack. |
| `mote-health`    | `pixi run health`   | Health monitor; `Type=notify` + `WatchdogSec` watchdog. `After=` only, so it keeps observing across a bringup restart. |

**Hardening** (all services):

- `Restart=always` with backoff (`RestartSec=2`, `RestartSteps=5`,
  `RestartMaxDelaySec=30`): a crashed service restarts, backing off from 2 s to a
  30 s ceiling instead of hammering.
- `StartLimitIntervalSec=0`: never permanently give up. An unattended robot must
  self-heal when reconnected hardware reappears; the backoff prevents a busy
  loop, so there is no reason to latch into a failed state.
- **Device ordering**: the udev rules tag the servo/lidar devices
  `TAG+="systemd"`, so systemd synthesises `dev-mote_servos.device` /
  `dev-mote_lidar.device`. `mote-bringup` orders `After=` / `Wants=` them, so
  bringup waits for the hardware to enumerate but a mid-run flap does not
  force-kill the stack (the self-check and health monitor own that).
- **journald sizing**: `systemd/journald-mote.conf` bounds the persistent
  journal (`SystemMaxUse=500M`, `SystemKeepFree=1G`, `MaxRetentionSec=2week`) so
  always-restarting services can never fill the SD card.
- **No `Wants=` on the dependents.** `mote-health` and `mote-record` order
  `After=` bringup but never *pull* it. A `Wants=` dependent that is itself
  restart-looping fires a start request for bringup every cycle, and such a
  request **bypasses bringup's own `RestartSec` backoff** — systemd logs
  "Scheduled restart job immediately on client request". Measured on the robot:
  bringup hammered every ~3.5 s while systemd's `RestartUSecNext` already said
  30 s. Each unit is started at boot by its own `WantedBy=multi-user.target`.
  `mote-slam`/`mote-nav` keep `Requires=`/`BindsTo=` because they are genuinely
  meaningless without bringup, and `BindsTo` holds them stopped (rather than
  looping) while it is down.
- **Units run from the checkout they were installed from**, via `@REPO@`
  substituted by `install.sh` from its own location — not a hardcoded `~/Mote`.
  Installing from a second checkout otherwise yields units pointing at a tree
  that may not contain the tasks they invoke (`status=127`, permanent loop).

**Three layers of process recovery** (see also `test/chaos/`):

1. **Node crash** → the launch system relaunches just that node
   (`respawn=True` on the drivers in `mote_launch.py` and the nav2 servers in
   `nav2_launch.py`; the nav2 lifecycle managers reconnect the respawned node).
2. **Launch/process crash** → systemd restarts the whole service.
3. **Health-monitor hang** → the `WatchdogSec` watchdog restarts `mote-health`.

## Startup self-check — `self_check.py`

Runs as `mote-bringup`'s `ExecStartPre` (and by hand: `pixi run self-check`).
Fast, static pre-flight checks — no ROS graph — that gate the launch:

- **servos**: bus device present *and* an SCServo ping answers (`servo_ping`, the
  drive IDs from `robot.yaml`) — CRITICAL.
- **lidar**: device present and openable — CRITICAL.
- **camera**: device present and openable — advisory.
- **disk**: free space on `MOTE_HOME` (< 500 MB CRITICAL, < 2 GB warn).
- **clock**: system time looks NTP-synced (an RTC-less Pi boots at the epoch) —
  advisory.
- **config**: `robot.yaml` parses (CRITICAL); an active site is resolved
  (advisory — mapping runs without one).

Any failed **CRITICAL** check → non-zero exit → the `ExecStartPre` fails →
bringup does not start (robot stays in safe idle) and systemd retries with
backoff, so replugging the lidar recovers on its own. The verdict is written to
`$MOTE_HOME/self_check_status.yaml` and printed to journald.

Runtime data-flow liveness (is `/scan` *publishing*?) is deliberately **not**
checked here — the drivers are not up yet. That is the health monitor's job.

## Health monitor — `health_monitor.py`

Runs as `mote-health.service` (or `pixi run health`). Watches subsystem liveness
and publishes, every second:

- **`/diagnostics_agg`** (`diagnostic_msgs/DiagnosticArray`) — one
  `DiagnosticStatus` per subsystem (scan, filtered scan, joint states, camera,
  odom TF, localisation TF), the host status folded in from `system_monitor`'s
  `/diagnostics`, the last self-check verdict, and a rolled-up `mote` status. The
  standard form the fleet layer can lift later.
- **`/health`** (`std_msgs/String`) — a single human-readable summary line:
  `OK` / `DEGRADED: camera stale` / `FAULT: scan stale (…)`. Easy to eyeball:

  ```bash
  pixi run -- ros2 topic echo /health
  ```

**Severity → roll-up**, set per subsystem in `config/health.yaml`
(overridable per-robot at `~/.mote/health.yaml`):

| `severity` | Missing/stale means | Used for |
|-----------|---------------------|----------|
| `critical` | **FAULT** | scan, filtered scan, joint states, odom TF — cannot drive without them |
| `degraded` | **DEGRADED** | camera — capability lost, driving still safe |
| `info` | reported, never degrades | `map→odom`, which exists only once a mission localises |

A fresh-but-slow subsystem degrades one step at most (never above its own
severity). `info` exists because the hardware base alone legitimately has no map
frame: scoring that as DEGRADED made a healthy idle robot report DEGRADED
forever. Mission localisation health belongs to the nav2 lifecycle, not here.

Two things learned measuring this on the robot:

- The `joint_states` 5 Hz floor is a *control-loop-overrun* check, not a rate
  spec (`controller_manager` runs at 50 Hz). When the servo bus stopped
  answering, each `read()` blocked ~200 ms per servo and the loop collapsed to
  ~1.6 Hz — the floor caught a real fault the driver only logged as warnings.
- `/diagnostics` is a **shared** topic: `controller_manager` publishes its own
  loop-jitter status there. The host status is therefore matched by exact name
  (`system`), or a third party's ERROR gets misattributed to the host and drives
  a spurious robot-level FAULT.

The monitor is also the systemd watchdog feeder: it sends `READY=1` once up and
pets the watchdog on every publish (`sd_notify.py`, a dependency-free
`$NOTIFY_SOCKET` client that no-ops outside systemd).

## Known gap: battery voltage

The USB-C power bank exposes **no state-of-charge or voltage telemetry**, so the
robot cannot see its own battery in software. The only power signal available is
the Raspberry Pi firmware's `get_throttled` under-voltage bitfield, which
`system_monitor` already reports (a brown-out shows as `DEGRADED`). True battery
sensing needs a hardware change (a fuel-gauge / INA-class sensor on the power
rail) and is tracked as a follow-up — see the reliability follow-up task.
