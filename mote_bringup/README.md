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

## Mapping a space autonomously

`pixi run explore` drives autonomous coverage against a live mapping mission:
left-wall following for dense boundary tracing, a Nav2 frontier relocation when
the map stops growing, and a stuck-escape (back off, turn away, blacklist the
spot) for obstacles the 2D lidar cannot see — rug edges, cables, low clutter.
It exits when no reachable frontier remains, then the map is saved like any
other session. The sim builds its world sites with the same tool (`pixi run
sim-map-world`, which passes `--sim-time`).

It publishes the drive mux's *teleop* input (see the drive path below) — it
stands in for a human driver, so its wall-follow out-ranks the Nav2 goals it
hands off during relocation. A human on the Foxglove stick shares that input
with it, last writer wins — stop the explorer before driving by hand.

**Run everything on the Pi**, in tmux, so losing wifi only loses your view of
the mission — never the mission:

```bash
ssh <robot> tmux new -s map
pixi run mapping      # window 1
pixi run explore      # window 2 — watch progress via Foxglove
pixi run save-map     # when it reports covered
```

The default thresholds suit corridor-scale spaces. Domestic layouts (~0.75 m
doorways) want the geometry tightened, e.g.
`pixi run explore -- --cruise 0.2 --obstacle 0.4 --desired-left 0.6 --follow-band 1.0 --blacklist-radius 1.0`.

If the scan stream goes stale (wedged graph, dead lidar) the explorer stops
and waits rather than driving blind. The graph itself cannot be stalled by
wifi: DDS transport is **loopback-only by default** (`config/cyclonedds.xml`,
loaded through `CYCLONEDDS_URI` by pixi activation and the systemd units
alike), because Cyclone otherwise prefers a radio interface's locators even
between processes on the same board, and a wifi flap then freezes same-host
scan delivery. Foxglove still works (it is a WebSocket server, not a DDS
peer); RViz-over-LAN does not — Foxglove is the supported window. One gotcha:
a stale `ros2` daemon from a different environment will show an empty graph
until `pkill -9 -f '[_]ros2_daemon'`.

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

## Wifi roaming

The robot changes access point as it drives between rooms because the Broadcom
firmware's roaming engine is on: `options brcmfmac roamoff=0`, the driver's own
default, which Raspberry Pi OS overrides. Neither iwd nor wpa_supplicant will
take that decision on this card, for reasons in their source rather than their
configuration — `wifi/README.md` quotes both.

```bash
pixi run wifi-check     # what takes the roam decision (read-only)
pixi run wifi-roaming   # write the modprobe option; takes effect at next boot
pixi run wifi-roamlog   # log BSSID/signal/RTT during an acceptance walk
```

The trigger is -75 dBm with a 20 dB delta, compiled into the driver and not
tunable. What to do if that is wrong for a site, and the measurements behind all
of it, are in [`wifi/README.md`](wifi/README.md).

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

## Slip monitor — `slip_monitor.py`

Started by `mote_launch.py`, beside `system_monitor`. It reads the disagreement
between the robot's two motion sources and publishes what it means as the `slip`
status on `/diagnostics`, which the health monitor folds into the roll-up.

kinematic_icp *takes* wheel odometry as its prior and corrects it against the
scan, so the correction is already a measurement of how wrong the wheels were —
a slip signal on existing hardware, with no IMU. Over a 1 s sliding window the
node compares the travel each source reports, in the body frame, and reports:

| state | meaning |
|---|---|
| `slip` | The wheels claim travel the lidar did not see. Wheels spinning on a slippery floor, or a robot wedged against something. |
| `stuck` | Motion is commanded and *neither* source reports any. |
| `icp_fault` | The lidar pose moved in a way the drive cannot produce. Slip makes the wheels over-read, never the lidar, so this is a scan-match excursion — or the robot being moved by hand. |

All three are **DEGRADED**, never FAULT: each is a reason to stop and re-plan,
not a reason to refuse to drive, and a monitor that can halt the robot on a
threshold is a worse failure than the slip it is watching for.

`slip/residual` (`geometry_msgs/TwistStamped`) carries the raw numbers so they
can be recorded and plotted: `linear.x` is the speed residual (wheel minus
lidar, m/s), `linear.y` the speed it is relative to, `angular.z` the yaw-rate
residual.

Three things are worth knowing:

- **Only translation is thresholded.** The yaw residual is published but never
  keyed off: measured on real bags it is dominated by scan-match jitter, reaching
  a p99 as large as the yaw rate itself, so no threshold exists that a hard turn
  would not trip. Translation on the same bags has a p99 of 0.006–0.021 m/s.
- **A stalled lidar is not slip.** Without a guard, a stopped corrector freezes
  the window while the wheels keep turning, which grows without bound and looks
  exactly like slip. A source older than `max_lag` yields no verdict instead.
- **Thresholds are measurements, not guesses.** They come from the residual
  distribution over `~/.mote/bags/mapping`, live in `config/slip.yaml`
  (overridable at `$MOTE_HOME/slip.yaml` — traction is a property of one robot on
  one floor), and are re-checkable with `tools/slip_replay.py`, which drives the
  very same estimator over a bag. The derivation, and the six real events it
  found in those bags, are in `docs/tuning/2026-07-28-slip-detection.md`.

## Health monitor — `mote_health`

Lives in its own package now, and is C++: what a monitor costs is how often it
is woken, and this one consumes ~152 msg/s, which no rclpy callback can afford.
The evidence is `docs/tuning/2026-08-11-monitor-cpu.md`, the port
`docs/tuning/2026-09-01-health-monitor-cpp.md`, and the package's own
`mote_health/README.md`. What it decides is unchanged and is summarised here
because it is read alongside the other monitors above.

Runs as `mote-health.service` (or `pixi run health`). Watches subsystem liveness
and publishes, every second:

- **`/diagnostics_agg`** (`diagnostic_msgs/DiagnosticArray`) — one
  `DiagnosticStatus` per subsystem (scan, filtered scan, joint states, camera,
  odom TF, localisation TF), the `system` and `slip` statuses folded in from the
  shared `/diagnostics`, the last self-check verdict, and a rolled-up `mote`
  status. The standard form the fleet layer can lift later.
- **`/health`** (`std_msgs/String`) — a single human-readable summary line:
  `OK` / `DEGRADED: camera stale` / `FAULT: scan stale (…)`. Easy to eyeball:

  ```bash
  pixi run -- ros2 topic echo /health
  ```

**Severity → roll-up**, set per subsystem in `mote_health/config/health.yaml`
(overridable per-robot at `$MOTE_HOME/health.yaml`, the same rule `mote_home`
holds for Python):

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
  loop-jitter status there. Statuses are therefore lifted by exact name, listed
  in `health.yaml`'s `diagnostic_statuses` (`system`, `slip`), or a third party's
  ERROR gets misattributed to one of ours and drives a spurious robot-level
  FAULT. A named status nobody is publishing is simply absent — asserting
  another monitor's liveness is not this monitor's job.

The monitor is also the systemd watchdog feeder: it sends `READY=1` once up and
pets the watchdog on every publish (`mote_health/src/sd_notify.cpp`, a
dependency-free `$NOTIFY_SOCKET` client that no-ops outside systemd). It moved
with the monitor rather than being copied: `mote-health.service` is the only
`Type=notify` unit, so nothing here was left needing the Python one.

## Clearing stray ROS processes — `sweep_orphans.py`

Two different messes, one module, and the difference is worth knowing before
reaching for either.

`pixi run kill` clears **this checkout's** ROS processes and resets the daemon —
the "my stack is wedged" reset. `pixi run sweep` clears **other jobs'** leftovers:
processes an agent worktree started and never reaped, which reparent to init and
run until the box is rebooted. It reports by default and only acts with `--kill`:

```bash
pixi run sweep                 # what would go, grouped by the job that left it
pixi run sweep -- --kill       # reap them
pixi run sweep -- --json       # for a script
```

That second mess is not just untidiness. Leftovers are the exact process names a
benchmark measures, and the system-wide counters a benchmark sits in — context
switches, interrupts, memory pressure, CPU contention — cannot be scoped the way
`overhead.py` scopes its own match. A drifting background makes every
measurement on the box a little less comparable than it looks.

**Matching is on process identity, never on the command line.** The `pkill -9 -f
'<driver names>'` that `kill` used to run matched the shell running the task
itself — those names are in its own command line — SIGKILLed it, and so never
reached the `ros2 daemon` reset that followed; it also matched every other
checkout and worktree on the machine. Both modes now read `/proc` and require a
ROS environment, a path under the directory in question, and absence from the
sweeper's own ancestry. The sweep additionally requires that the process be
orphaned and older than `--min-age` (30 min), because a deliberately
session-detached run — the sim smoke test `setsid`s its launch — is
indistinguishable from a leak by ancestry alone.

### Why they escape

`ros2 run` is a wrapper: it `Popen`s the real executable and installs no SIGTERM
handler, tolerating only `KeyboardInterrupt` on the assumption that the signal
reached the whole process group — true of a Ctrl-C at a terminal, false of a
`proc.terminate()` from a test fixture. Terminating the wrapper therefore kills
the wrapper and hands the node to init, **once per run, on the path where the run
succeeded**. Measured on `test_twist_mux_arbitration.py`: six tests pass and one
`twist_mux` survives.

So anything spawning `ros2 run` uses `spawn_reapable` / `reap_group` from
`sweep_orphans`, which put the child in its own session and signal the group.
The other half — a job killed outright, taking pytest with it before any teardown
runs — no fixture can fix, and that is what the sweep is for.

## Known gap: battery voltage

The USB-C power bank exposes **no state-of-charge or voltage telemetry**, so the
robot cannot see its own battery in software. The only power signal available is
the Raspberry Pi firmware's `get_throttled` under-voltage bitfield, which
`system_monitor` already reports (a brown-out shows as `DEGRADED`). True battery
sensing needs a hardware change (a fuel-gauge / INA-class sensor on the power
rail) and is tracked as a follow-up — see the reliability follow-up task.
