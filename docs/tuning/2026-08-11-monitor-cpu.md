# Monitor-node CPU — the wake-up is the cost, not the payload, 2026-08-11

**Verdict: the two Python fixes landed and are worth ~1 point of a core each;
the ~5% target is not reachable in Python and the C++ port of `health_monitor`
is justified — on evidence that is not the evidence the task expected.** The
premise was that these nodes are expensive because they deserialize high-rate
messages. Measured on the robot, deserialization is **13%** of what a
subscription costs; the other 87% is the rclpy wake-up itself, which no change
to what the callback does can touch.

Raw sampler output is in `2026-08-11-monitor-cpu/`; every figure below is a
`mean` from one of those JSON files and can be re-derived with
`pixi run node-cpu --summary <csv>`.

## How it was measured

`node_cpu.py` samples `/proc/<pid>/stat` (utime+stime deltas) once a second per
node, and records the load average beside each sample. Run on **mote-01**, a
4-core Pi 5, with `pixi run robot` + `pixi run tasks` up and no task dispatched.

Two things had to be got right before any number meant anything.

**The robot's own condition drifts, and it drifts in exactly the variable under
study.** The drive servos answer intermittently: in the first run `MoteHardware`
logged 3159 `Failed to read position` and the 50 Hz control loop had collapsed to
1.6 Hz, taking `/joint_states` to 1.6 Hz and `/tf` to 33 Hz; in the next run the
same stack logged 564 and ran at 51 Hz. Since the monitors' cost is a function of
those rates, two sequential runs are not comparable — and the first sequential
before/after pair says the change made everything *worse*, including
`slip_monitor`, which was not changed at all:

| | `health_monitor` | `task_server` | `slip_monitor` | load1 |
| --- | --- | --- | --- | --- |
| before (`/tf` 33 Hz) | 15.1 | 4.6 | 4.0 | 3.0 |
| after (`/tf` 51 Hz) | 17.3 | 7.1 | 7.3 | 3.2 |

That table measures the servo bus, not the patch. Everything below is therefore
**paired**: both builds run at the same instant on the same machine, the second
one started renamed (`-r __node:=health_monitor_b`), so they see byte-identical
input. `node_cpu.py` identifies a node by the entry point its interpreter is
running plus any `__node:=` rename, which is what makes that possible — and what
keeps it from weighing the `ros2 run` and `pixi run` wrappers, whose command
lines repeat the node's own.

**The 50 Hz `/joint_states` had to be restored.** With the servo bus failing, the
highest-rate topic `health_monitor` watches was not running at its real rate, so
a synthetic 50 Hz publisher stood in for a healthy control loop. It only restores
the arrival rate, which is all the monitor consumes.

## What a subscription actually costs

Four probe nodes, spun side by side against the same live graph
(`probe-tf.json`, 90 s, load1 3.6):

| probe | what it holds | CPU % of a core | delta |
| --- | --- | --- | --- |
| `floor` | nothing — a bare rclpy node | 0.5 | — |
| `raw` | 4 subscriptions, `raw=True` (101 msg/s) | 8.4 | **+7.9** wake-ups |
| `deser` | the same 4, deserialized | 9.6 | **+1.2** deserialization |
| `tf` | one `TransformListener` (`/tf`, 51 msg/s) | 5.3 | **+4.8** |

The four topics are `health_monitor`'s own: `/scan` and `/scan_filtered` at
10 Hz, `/joint_states` at 50 Hz, `/image_raw/compressed` at 29 Hz.

Three things fall out of that table.

**A bare rclpy node is free** (0.5%), so there is no fixed floor to blame and
nothing to reclaim by making a node do less between messages.

**Deserialization is 1.2 of the 9.1 points a subscription set costs above the
floor — 13%.** The remaining 7.9 points is the executor round trip: wait set,
take, dispatch into Python. At 101 msg/s that is **0.78 ms of CPU to deliver one
message to a callback that increments a counter**, which on a ~2.4 GHz core is
around two million cycles.

**A `TransformListener` is the single most expensive thing either node holds.**
It takes the whole `/tf` stream — 51 msg/s, whatever handful of edges the node
asks about — for 4.8 points, more than the camera and both lidar topics
together.

Those numbers account for the nodes as measured. `health_monitor` at 16.9 is
0.5 floor + 7.9 topics + 4.8 TF + ~3.7 for the 1 Hz roll-up (two TF lookups,
eight `DiagnosticStatus` messages, two publishes). `task_server` at 6.9 is
0.5 floor + 4.8 TF — the listener `AcquireObject` creates in `setup()` and uses
only during a fetch — plus its idle subscriptions and its tick.

## The two changes, measured

`ab-patched-vs-unpatched.json`, 120 s, both builds running at once, load1 3.8:

| node | before | after | saved |
| --- | --- | --- | --- |
| `health_monitor` (`raw=True` on the watched topics) | 18.1 | **17.1** | 1.0 |
| `task_server` (idle tick 10 Hz → 1 Hz) | 7.8 | **6.9** | 0.9 |

Both are real and both are small, and the probe explains each independently.

The 1.0 point on `health_monitor` is the deserialization the probe priced at
1.2 — the whole of what `raw=True` can ever be worth, since the wake-up it
cannot avoid is 87% of the cost. It is still worth keeping: it is free, and it
scales with payload size rather than count, so the ~29 fps `CompressedImage` is
most of it.

The 0.9 points on `task_server` is the surprise. Dropping from 10 ticks a second
to 1 removes 90% of the ticks and 12% of the node's CPU, because **ticking was
never what `task_server` spent its time on**. 9 ticks/s of an idling tree cost
0.09 points each; the node's real load is the `/tf` stream it holds through
`AcquireObject`'s listener from the moment it starts until it exits. The idle
tick is still the right change — a tree between missions has nothing to advance,
and the saving is free — but it is not the lever the CPU figure suggested.

Deadman behaviour is unchanged in both. `health_monitor` still pets the systemd
watchdog on every publish, and `/health` and `/diagnostics_agg` are byte-identical
in content and cadence (1.0 Hz, `data: OK`, `mote` roll-up first then one status
per subsystem — asserted in `test_health_monitor_node.py`). Command acceptance
never depended on the tick: `on_command` publishes `accepted:`/`rejected:` from
the subscription callback. What the idle rate could have delayed is the first
tick of the accepted tree, and does not, because `_set_tick_rate` resets the
timer as well as re-periodding it — **setting a period does not move the expiry
already pending**: a 5 s timer 0.3 s into its period still reports 4.7 s to go
after its period is set to 0.05 s, and 0.05 s after `reset()`. Without the reset
the first tick waits out the rest of the idle period (measured: 2.00 s), which
`test_goto_tree.py::test_idle_tick_rate_does_not_delay_the_mission` holds.

## The C++ decision: taken, for `health_monitor`

**The ~5% target is not reachable in Python.** `health_monitor` consumes ~152
msg/s (101 topic + 51 TF). At the measured rclpy wake-up cost of ~0.078 points
per msg/s that is ~12 points before the node does anything with them, against a
5-point budget. Reaching 5% in Python would mean consuming **no more than ~55
msg/s**, i.e. giving up watching the camera, the joint states, or odometry —
which is giving up the monitor.

The alternatives were considered and rejected:

* **Swap the `odometry` TF watch for a topic watch** (the task's stage-1 item 3).
  It buys nothing: the candidate topics (`/diff_drive_controller/odom`,
  `/tf`) are both ~50 Hz, so the wake-up count is unchanged. Dropping *both* TF
  watches would retire the listener and its 4.8 points, but `localization`
  (map→odom) has no topic equivalent, and a topic being fresh is not evidence
  that the TF edge Nav2 consumes was broadcast. ~3 points for a weaker check is
  a bad trade when C++ restores the real check cheaply.
* **Sampling the topics duty-cycled** — subscribe for 200 ms a second — cuts
  wake-ups by 80% and makes a monitor that is not looking 80% of the time.
* **Liveliness QoS** would move freshness into the middleware and cost nothing
  per message, but `MANUAL_BY_TOPIC` has to be asserted by the *publisher*, and
  the publishers are sllidar, v4l2_camera and joint_state_broadcaster.

What the port must be, on this evidence rather than on the original premise:

* The lever is **the number of Python wake-ups**, so the port has to own *all*
  the subscriptions including `/tf` — 4.8 of the 16.9 points are the TF
  listener, which the original framing ruled out as "not the cost".
* `create_generic_subscription(topic, type_string, qos, cb)` takes the type name
  as a runtime string exactly as `get_message(spec["type"])` does, so
  `health.yaml`'s topic list survives the port unchanged, with no codegen.
* It needs a home. `mote_bringup` is `ament_python`; `mote_nav`'s charter is
  "the C++ that runs inside other people's processes", which a standalone
  `Type=notify` service node is not. A new small C++ package.
* Three Python pieces need counterparts, each a second implementation of
  something that currently lives in one place: `mote_home.override()`,
  `sd_notify.py`, and YAML loading (yaml-cpp).
* Equivalence bar: byte-identical `/diagnostics_agg` and `/health` content and
  cadence on the same inputs, and `test_health_monitor.py`'s `_TopicWatch` /
  `_TfWatch` cases ported to gtest — the roll-up rules are the behaviour, not
  the subscriptions.

That is a new first-party package plus three re-implementations, and it is
tracked separately rather than bolted onto this change.

**Done, 2026-09-01: `docs/tuning/2026-09-01-health-monitor-cpp.md`.** The
package is `mote_health`; only one of the three re-implementations turned out to
be one (`sd_notify` moved rather than being copied). Measured paired on mote-01,
10.6% of a core to 1.1. One thing here did not reproduce and is worth knowing
before quoting the table above: on a synthetic bench at these same rates the
Python build measures **10.6, not 17.1**, and the difference is neither load nor
payload — so the figures on this page belong to the live stack specifically, and
a bench cannot be compared against them directly.

## `slip_monitor` is the largest remaining consumer

At **7.1** (`ab-patched-vs-unpatched.json`) it now costs more than `task_server`.
It is out of scope here and is not a port candidate as it stands: its maths is
`mote_bringup/mote_bringup/odom_residual.py`, ROS-free and shared with `tools/slip_replay.py`,
and that sharing is what set `config/slip.yaml`'s thresholds from real bags. A
C++ port forks the maths and quietly invalidates the calibration, so the shared-
maths problem has to be answered first. Filed as a follow-up.

For reference, `system_monitor` costs **0.6** and needs nothing.
