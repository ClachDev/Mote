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

## Drive path — who gets the wheels

`DiffDriveController` has exactly one publisher: `twist_mux`, started with the
base by `twist_mux_launch.py`. Everything that wants to move the robot publishes
an input instead, and the mux forwards the highest-priority source that has
spoken recently:

| Input | Published by | Priority | Timeout |
|---|---|---|---|
| `/cmd_vel_nav` | Nav2's `controller_server` and `behavior_server` | 10 | 0.5 s |
| `/cmd_vel_teleop_stamped` | `twist_relay` (Foxglove panel), `pixi run teleop`, the RViz teleop panel | 100 | 1.0 s |

Priorities and timeouts live in `config/twist_mux.yaml`; the output is
`/diff_drive_controller/cmd_vel`, unchanged, so bags, the benchmark and the sim
smoke test still watch the command the wheels actually got.

Before this, teleop and Nav2 both wrote the controller's topic and it simply took
whichever arrived last — so taking over by hand during a goal meant two writers
at 20 Hz and a robot tracking neither, and the documented remedy was "cancel the
task first", which is the wrong instruction for someone grabbing control of a run
that is going wrong.

**Teleop overrides Nav2; it does not cancel it.** The mux is a drive-path
component, and cancelling a goal from it would wire velocity arbitration into the
action layer — a nudge to straighten the robot in a doorway would destroy a fetch
mission halfway through. So a takeover suppresses Nav2 for as long as the
operator is driving, and the goal is still there afterwards.

**Letting go stops the robot before Nav2 gets it back.** That is what the teleop
input's 1.0 s timeout buys, against the controller's `cmd_vel_timeout` of 0.5 s
(`controllers.yaml`): after the operator's last command the wheels halt at 0.5 s
and Nav2 only regains the topic at 1.0 s, so there is always a stopped robot in
between rather than a handback mid-motion. Invert the two numbers and that
property is gone silently, so `test_twist_mux.py` holds the two files together
and `test_twist_mux_arbitration.py` measures the gap against a real mux
(1.00–1.05 s over five takeovers; pre-emption itself lands within one 20 Hz
publish period, ~50 ms).

**The deadman is unchanged.** `twist_mux` publishes from an input callback and
only when that input holds priority — no timer, no stored last command — so when
every source stops the mux stops and `cmd_vel_timeout` halts the wheels, exactly
as when the sources wrote the controller directly. A mux that re-published would
have turned "the operator's link dropped" into "the robot keeps going"; that it
does not is asserted, not assumed.

**To hold autonomy off entirely**, publish `std_msgs/Bool` on `/pause_navigation`
— `true` masks every source below priority 50, which is navigation and not
teleop, and `false` hands it back. The shipped Foxglove layout has a Publish
panel for it. The lock is state rather than a heartbeat (timeout 0.0), so it does
not engage when its publisher goes away and a restarted mux starts unlocked. Note
what a long pause does to the *mission*: Nav2's `SimpleProgressChecker` gives the
robot `movement_time_allowance` (10 s) to move `required_movement_radius`, so a
goal held off the wheels while the robot sits still aborts itself, and the task
reports failed. Driving under teleop keeps it alive, since the checker watches
the robot's pose and not who commanded it.

Cost is one process and **one DDS participant** (measured with
`pixi run dds-check`), putting the full robot stack at ~26 of 33.

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
| `mote-bringup`   | `pixi run launch health:=false foxglove:=false` | Hardware base. `ExecStartPre` runs the self-check gate first; the two `false`s are because `mote-health` and `mote-foxglove` run those separately here. |
| `mote-slam`      | `pixi run slam`     | `BindsTo`/`PartOf` bringup — restarts with it. |
| `mote-nav`       | `pixi run nav`      | `BindsTo`/`PartOf` slam. |
| `mote-record`    | `pixi run record`   | `After`/`PartOf` bringup (no `Wants` — see below); a recorder crash never takes down the drive stack. |
| `mote-health`    | `pixi run health`   | Health monitor; `Type=notify` + `WatchdogSec` watchdog. `After=` only, so it keeps observing across a bringup restart. |
| `mote-foxglove`  | `pixi run foxglove` | The operator's remote view + teleop (`docs/fleet/README.md` §10). `After=` only, for the same reason as the monitor: a crash-looping mission is when someone needs to look at it. |
| `mote-agent`     | `pixi run agent`    | Fleet bridge (`docs/fleet/README.md` §7). `After=` only. |

All of these also pin DDS to the robot
(`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`), so a robot running under systemd is
not visible to a workstation's ROS graph — `mote-foxglove` is the replacement.
An interactive `pixi run` keeps stock discovery.

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
  bringup then restarts at the dependent's rate rather than its own backoff.
  Each unit is started at boot by its own `WantedBy=multi-user.target`.
  `mote-slam`/`mote-nav` keep `Requires=`/`BindsTo=` because they are genuinely
  meaningless without bringup, and `BindsTo` holds them stopped (rather than
  looping) while it is down.
- **Units run from the checkout they were installed from**, via `@REPO@`
  substituted by `install.sh` from its own location — not a hardcoded `~/Mote`.
  Installing from a second checkout otherwise yields units pointing at a tree
  that may not contain the tasks they invoke (`status=127`, permanent loop).

**Three layers of process recovery** (see also `test/chaos/`):

1. **Process crash** → the launch system relaunches just that process
   (`respawn=True` on the drivers in `mote_launch.py` and on the Nav2 container
   in `nav2_launch.py`). Nav2 is composed, so its granularity is the whole stack
   rather than the individual server: the container respawns, the launch file
   reloads the components into it, and the lifecycle managers — components
   themselves — re-activate everything on the way back up.
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

## Host monitor — `system_monitor.py`

Started by `mote_launch.py`, so every mission bag carries the compute context:
CPU busy/load, memory, SoC temperature, fan RPM, and the Pi firmware's
`get_throttled` bitfield, published on `/diagnostics` as the `system` status.

**Throttle flags come from `vcgencmd get_throttled`, not sysfs.** The Pi 4
device-tree node (`/sys/devices/platform/soc/soc:firmware/get_throttled`) does
not exist on a Pi 5, so the sysfs read this monitor used to do was dead code —
the robot spent a whole nav mission at 85 °C without the ERROR ever firing. The
binary needs the invoking user in the `video` group (the `mote-*` service user
is). Off a Pi, `shutil.which` finds nothing and throttle reporting is skipped.

Four conditions are reported, each as a `_now` and a latched `_ever` key
(firmware bits 0–3 and their has-occurred latches at 16–19):
`undervoltage`, `freq_capped`, `throttled`, `soft_temp_limit`. Any `_now` bit
raises **ERROR** with a `power: …` message naming the conditions; the `_ever`
keys and the raw `throttled_flags` hex are informational. `soft_temp_limit` is
the sustained-85 °C case specifically — hard throttling (bit 2) is not always
asserted at the instant you sample, so keying only off it misses the event.

`fan_rpm` is the Active Cooler's tachometer, found by scanning
`/sys/class/hwmon/*/name` for `pwmfan` (hwmon indices are not stable across
boots). With no cooler fitted the key is simply absent. Measured on `auldbot`:
idle 48 °C / 0 RPM, 4-core load 61 °C / ~4900 RPM with no throttle bits set.

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
(overridable per-robot at `$MOTE_HOME/health.yaml`, resolved through `mote_home`):

| `severity` | Missing/stale means | Used for |
|-----------|---------------------|----------|
| `critical` | **FAULT** | scan, filtered scan, joint states, odom TF — cannot drive without them |
| `degraded` | **DEGRADED** | camera — capability lost, driving still safe |
| `info` | reported, never degrades | `map→odom`, which exists only once a mission localises |

A fresh-but-slow subsystem degrades one step at most (never above its own
severity). `info` exists because the hardware base alone legitimately has no map
frame: scoring that as DEGRADED made a healthy idle robot report DEGRADED
forever. Mission localisation health belongs to the nav2 lifecycle, not here.

Two things worth knowing about these thresholds:

- The `joint_states` 5 Hz floor detects a control loop that is overrunning; it
  is not a rate spec (`controller_manager` runs at 50 Hz). An unresponsive servo
  bus blocks each `read()` ~200 ms per servo and collapses the loop to ~1.6 Hz,
  which the driver itself reports only as warnings.
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
