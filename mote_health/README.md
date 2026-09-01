# mote_health

The robot-level health monitor: one process that watches whether the
safety-critical subsystems are alive, and says so on two topics.

* `/diagnostics_agg` (`diagnostic_msgs/DiagnosticArray`) — a `mote` roll-up
  first, then one `DiagnosticStatus` per subsystem.
* `/health` (`std_msgs/String`) — one line, `OK` / `DEGRADED: ...` /
  `FAULT: ...`, for `ros2 topic echo` and log greps.

Run it with `pixi run health`, or let `pixi run robot` / `mapping` start it
alongside the base (`mote_launch.py`'s `health:=true`). Under systemd it is
`mote-health.service`, `Type=notify` with `WatchdogSec=15`.

## Why this package is C++

Measured on mote-01, an rclpy wake-up costs ~0.78 ms of CPU per message
delivered — the wait set, the take and the dispatch into Python, of which
deserializing the payload is only 13%. This node consumes ~152 msg/s (101 from
four topics, 51 from `/tf`), so ~12 points of a core are spent before it does
anything with them, against a 5-point budget. Nothing a callback does can touch
that, and consuming less means giving up the camera, the joint states or
odometry — which is giving up the monitor. The evidence is
`docs/tuning/2026-08-11-monitor-cpu.md`; the port and its equivalence check are
`docs/tuning/2026-09-01-health-monitor-cpp.md`.

It is its own package because the other two homes are wrong: `mote_bringup` is
`ament_python` and cannot build C++, and `mote_nav`'s charter is "the C++ that
runs *inside* other people's processes" — a Nav2 plugin and two composable
nodes — which a standalone `Type=notify` service node is not. It must stay its
own process for that reason too: composing it into an existing container would
put the watchdog on a process whose liveness is not the monitor's.

## What it watches

`config/health.yaml`, overridable per robot at `$MOTE_HOME/health.yaml`. Each
`topics:` entry names a topic and its message type; the type is handed to
`create_generic_subscription`, so adding a subsystem is a config change and
never a code one. Each `tf:` entry names a transform. `severity` decides how
much an absence costs:

| `severity` | a missing or stale subsystem reports | and the robot summary is |
| --- | --- | --- |
| `critical` | ERROR | FAULT |
| `degraded` | WARN | DEGRADED |
| `info` | OK | unchanged |

`info` exists for edges legitimately absent in a healthy state: `map`→`odom`
appears only once a mission localises, so scoring it would leave an idle robot
permanently DEGRADED. A subsystem that is fresh but below `min_rate` is
DEGRADED, never worse than its own severity.

`diagnostic_statuses` lists the statuses lifted off the shared `/diagnostics`
by **exact name** — `system` from `system_monitor`, `slip` from `slip_monitor`.
The topic is shared (controller_manager publishes its own loop-jitter status
there), so folding in whatever arrives would attribute a third party's level to
one of ours. A named status nobody is publishing is simply absent: asserting
another monitor's liveness is not this monitor's job.

The last pre-flight verdict is read from `$MOTE_HOME/self_check_status.yaml`
and reported as `self_check`, capped at DEGRADED — bringup would not have
started on a hard failure, so a failed pre-flight is informational at runtime.

## Layout

| file | what it is |
| --- | --- |
| `src/health_rollup.cpp` | the decisions: freshness, rate, severity, the summary. No rclcpp, no tf2, no YAML. |
| `src/config.cpp` | health.yaml → watches. |
| `src/health_monitor.cpp` | the node: subscriptions, transform lookups, publishing. |
| `src/mote_home.cpp` | the `$MOTE_HOME` rule, which is `mote_bringup/mote_home.py`'s. |
| `src/sd_notify.cpp` | READY=1 and WATCHDOG=1 over an AF_UNIX datagram. |
| `test/compare_monitors.py` | runs two builds side by side and diffs what they publish. |

Only one of the last two is a second implementation. `sd_notify` **moved**:
`mote-health.service` is the only `Type=notify` unit in the tree, so the Python
client went with the monitor and nothing was left holding a copy. `mote_home`
could not, because `sites.py`, the launch files and `self_check.py` still need
the rule in Python — so `test_mote_home.cpp` pins the C++ half to the same
cases `mote_home.py`'s tests pin. `test_sd_notify.cpp` binds a real socket
rather than trusting that a datagram was sent, which the Python tests could not
do: a watchdog that is never petted has systemd kill the monitor every 15 s,
and nothing inside the process can see it.

## Tests

`pixi run test` runs them; `colcon test --packages-select mote_health` alone.

* `test_health_rollup` — the roll-up decisions, case for case from the Python
  monitor's own tests. These are the behaviour; the subscriptions are not.
* `test_config` — health.yaml parses into watches, an unknown severity is
  refused naming the entry, an *empty* file is refused rather than yielding a
  monitor that watches nothing and reports `OK`, and `$MOTE_HOME` beats the
  packaged default.
* `test_mote_home`, `test_sd_notify` — the two ported pieces.
* `test_health_monitor_node` — the real node against a real publisher. A
  generic subscription that delivers nothing leaves a node which still runs,
  still publishes on time, and reports every subsystem as missing, so the
  delivery has to be asserted rather than assumed.

`test/compare_monitors.py` is the equivalence harness rather than a unit test:
it drives two monitor commands from one set of synthetic publishers, through
healthy / stale / degraded / recovered, and reports any difference in which
subsystems are reported, in what order, at what level, with what message,
which values, and how often. It is what showed the port publishes the same
thing as the Python node it replaced.
