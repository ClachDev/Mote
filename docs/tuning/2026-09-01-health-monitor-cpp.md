# health_monitor in C++ — 10.6% of a core to 1.1%, 2026-09-01

**Verdict: the port lands. Measured paired on mote-01, `health_monitor` falls
from 10.6% of a core to 1.1% and from 66.9 MB resident to 26.2, and publishes
the same thing — every status name, order, level, message and value, and the
same 1.0 Hz cadence.** The one number that did *not* reproduce is the Python
build's absolute cost: the live stack put it at 17.1
(`2026-08-11-monitor-cpu.md`), this synthetic bench puts it at 10.6, and §4 says
what that does and does not change.

Raw sampler output is in `2026-09-01-health-monitor-cpp/`; every figure is a
`mean` from one of those JSON files and can be re-derived with
`pixi run node-cpu --summary <csv>`.

## 1. Why the port, and why *all* the subscriptions

`2026-08-11-monitor-cpu.md` priced an rclpy wake-up at ~0.78 ms of CPU per
message, of which deserializing the payload is 13%. `health_monitor` consumes
~152 msg/s, so ~12 points of a core go before it does anything with them,
against a 5-point budget. That is not reachable in Python, and the two sanctioned
Python fixes were worth ~1 point each.

The lever is therefore the *count* of wake-ups, which makes `/tf` part of the
port and not an exception to it. A `TransformListener` takes the whole `/tf`
stream — 4.8 of the Python node's 16.9 points — whatever handful of edges the
node asks about, so a port that left the TF watches in Python would have kept a
third of the cost while doubling the number of processes.

## 2. What the port is

A new package, `mote_health`, because the two existing homes are wrong:
`mote_bringup` is `ament_python` and cannot build C++, and `mote_nav`'s charter
is "the C++ that runs *inside* other people's processes", which a standalone
`Type=notify` service node is not. It stays its own process for the same reason
it is a service: `mote-health.service` binds a watchdog to it, and composing it
into a container would bind that watchdog to something else's liveness.

Three things carried across without change of design:

* **The config-driven topic list.** `create_generic_subscription(topic,
  type_string, qos, cb)` takes the message type as a runtime string exactly as
  `get_message(spec["type"])` did, so `health.yaml` needed no edit and there is
  no codegen per type. The callback is handed a serialized message it never
  opens — which is what `raw=True` bought in Python, now free.
* **The roll-up rules**, into `src/health_rollup.cpp`, which includes no rclcpp,
  no tf2 and no yaml-cpp. The node contributes arrivals and transform lookups;
  everything else — severity, slow-vs-stale, the summary line — is a plain
  function and a gtest. `test_health_rollup.cpp` is `test_health_monitor.py`'s
  cases ported one for one.
* **`health.yaml` itself**, from `mote_bringup/config/` to
  `mote_health/config/`, with the `$MOTE_HOME` override rule unchanged.

Two Python pieces had to come with it. `sd_notify` **moved**:
`mote-health.service` is the only `Type=notify` unit in the tree, so no copy was
left behind — and the C++ test binds a real socket and reads the datagram, which
the Python test could not, though a watchdog that is never petted has systemd
kill the monitor every 15 s. `mote_home`'s override rule is **duplicated**,
because `sites.py`, `self_check.py` and the launch files still need it in
Python. That is the only duplication the port creates, and `test_mote_home.cpp`
pins it to the same cases.

## 3. Equivalence: measured, not asserted

No bit-identity was available as it was for `OdomTfRelay` — the two builds count
arrivals against their own clocks, so `rate_hz` and `age_s` differ by sampling
jitter however correct both are. `mote_health/test/compare_monitors.py` runs two
monitor commands **at the same instant** against one set of synthetic
publishers, walks them through healthy → one critical topic gone → a forwarded
status degraded → recovered, and diffs everything else.

Everything else is: which subsystems are reported, in what order, at what level,
with what message, with which values *and what those values say*, and how often.
Only `rate_hz` and `age_s` are exempt, and then only within a tolerance
(measured agreement: 0.00 Hz apart on all four topics).

`equivalence.json`, 40 s, both builds live:

| | C++ | Python |
| --- | --- | --- |
| `/diagnostics_agg` publishes | 39 | 39 |
| mean gap | 1.000 s | 1.000 s |
| max gap | 1.002 s | 1.008 s |
| distinct aggregate shapes | 5 | 5 |
| distinct `/health` summaries | 5 | 5 |

The five summaries, identical on both sides:

```
OK
DEGRADED: scan slow (# < # Hz)
DEGRADED: scan slow (# < # Hz), host cpu 12% temp 48C
FAULT: scan stale (#s > #s)
FAULT: scan stale (#s > #s), host cpu 12% temp 48C
```

Two of those matches are worth naming, because they are where two languages had
every chance to disagree and did not. The `localization` watch reports
`error: "map" passed to lookupTransform argument target_frame does not exist.`
— **character for character** from `tf2::TransformException::what()` and from
the rclpy binding's `str(exc)`. And `self_check`'s `at` value reads back
`2026-09-01T09:00:00+00:00` through yaml-cpp exactly as PyYAML wrote it, which
is the one place a YAML library swap could have silently retyped a scalar.

The `host cpu 12% temp 48C` line is the other one: the fixture publishes that
status with an embedded newline, and both builds collapse it. A `/health`
summary that breaks across lines stops being greppable, and the Python
implementation had that bug once.

## 4. CPU: three paired runs on mote-01

A percentage of a core means nothing without the machine's condition beside it,
and the robot's own condition drifts — the servo bus answered in one of the
August runs and not the next, moving `/tf` by 18 Hz and inverting a sequential
before/after. So both builds run **at the same instant**, the Python one started
renamed (`-r __node:=health_monitor_py`), against one input stream. That is what
`node_cpu.py` was extended for: it identified a node by the entry point its
interpreter runs, which no C++ node has, so it now also matches a process whose
`argv[0]` is itself an installed executable (`test_node_cpu.py` holds both, and
holds the wrappers out).

The inputs are **synthetic** — `compare_monitors.py --hold`, publishing `/scan`
and `/scan_filtered` at 10 Hz, `/joint_states` at 50, `/image_raw/compressed` at
29 and `/tf` at 50, plus `/diagnostics` — because the drive stack was not run:
the arrival rates are all the monitor consumes, and the August work had already
had to substitute a synthetic 50 Hz `/joint_states` for a failing servo bus.

| run | conditions | C++ | Python | ratio |
| --- | --- | --- | --- | --- |
| `paired-idle` | load1 0.1, light payloads | **0.8** | 10.5 | 13× |
| `paired-loaded` | load1 3.0 (p95 3.6), light payloads | **0.9** | 9.7 | 11× |
| `paired-real-payloads` | load1 0.6, 40 KB JPEG, 720-point scans, 8 joints | **1.1** | 10.6 | 10× |

Resident memory, same runs: **26.2 MB against 66.9**.

Three things fall out.

**Contention is not the variable.** load1 3.0 against load1 0.1 moved the Python
build by 0.8 points and the C++ build by 0.1. The August table's spread was the
input rates, not the machine being busy — which is consistent with what it
concluded, and worth knowing before anyone reads a monitor's CPU figure as a
sign of how loaded the robot is.

**Payload size is not the variable either.** A 40 KB JPEG at 29 fps against a
4 KB stand-in, 720-point scans against 360 and eight joints against two, is
about 1 MB/s more to copy out of the middleware — and it moves the Python build
by 0.1 points and the C++ build by 0.3 (rows 1 and 3, at load1 0.1 and 0.6, so
neither figure is clean to better than a few tenths). That is the same finding
as August's, from the other direction: what a subscription costs is the wake-up,
not the bytes, and it is worth having measured rather than assumed — the obvious
guess is that the camera dominates because its frames are the big ones.

**The Python build measures 10.6 here and measured 17.1 on the live stack, and
that gap is not explained.** It is not load and it is not payload — the two
things a bench can vary, both varied, neither moved it. The remaining
differences are the ones a synthetic bench cannot have: ~25 DDS participants
instead of 4, a real TF tree of a dozen frames where the buffer inserts and
walks more per message, and two `lookup_transform` calls that succeed through a
chain rather than one failing at the first hop. So this bench is a **lighter
operating point than a robot in mission**, and the absolute figures here are
floors.

That does not change the verdict, and it is worth being explicit about why. The
target was ~5% of a core. The C++ build measures 1.1 at an operating point where
the Python build measures 10.6; scaling by the 17.1/10.6 the live stack implies
puts it near 1.8 in mission. To exceed 5 the port would have to be a further
factor of three worse than anything measured — while the build it replaces
already exceeded 5 by a factor of three.

## 5. Three deliberate divergences, and one the harness could not see

The equivalence harness compares what is *published*, so it says nothing about
inputs it never presents. Reviewing for those found four things.

**An empty `health.yaml` is now refused rather than accepted.** `override_path`
tests only that `$MOTE_HOME/health.yaml` exists, so a truncated write or an
override somebody created before editing it hands the monitor an empty
document. PyYAML returned `None` and the Python monitor died on the next
attribute access — ugly, and loud: systemd restart-looped it into the journal.
yaml-cpp returns a Null node from which every lookup is simply absent, so the
C++ build came up watching **nothing** and published `OK` with `subsystems: 0`
forever. That is the failure the monitor exists to prevent, wearing the monitor's
own colours, so `parse_config` refuses a document that is not a mapping and
`load_config` names the file. An explicit `topics: []` is a different statement
and is still accepted.

**A key written with no value means absent.** `severity:` with nothing after it
is a truthy Null node in yaml-cpp whose `as<std::string>()` is the literal
`"null"`, which `severity_level` would refuse as unknown. PyYAML gave `None`,
which the Python monitor read as absent; absent is what it means here.

**Two locale escapes were closed.** `printf("%.1f")` takes its decimal point
from `LC_NUMERIC` and `isspace()` its answer from `LC_CTYPE`, where Python's
`f"{x:.1f}"` and `str.split()` take neither. Under `LC_NUMERIC=de_DE` the port
would have published `rate_hz: 10,5` and `slow (1,0 < 5,0 Hz)` — on that machine
only, and failing nothing. `fixed1` uses `std::to_chars` and `one_line` scans an
explicit whitespace set; `test_health_rollup.cpp` sets a comma-decimal locale and
asserts under it.

**And one behaviour that changed without being a divergence.**
`tf2_ros::TransformListener` defaults to `spin_thread=true` where rclpy's
defaults to false, so the process now holds a second thread and `/tf` no longer
competes with the 1 Hz tick. The buffer is mutex-protected and the listener's
callback group is deliberately kept out of the node's executor, so the only
consequence is the thread — worth knowing when reading the process table, not a
thing to undo.

## 6. What is unchanged

`/health` and `/diagnostics_agg` in content and cadence (§3). The systemd
watchdog is still petted **after** a successful publish and never before, so a
monitor that has stopped reporting is still killed and restarted; the datagram
itself is asserted rather than assumed (`test_sd_notify.cpp`). `health.yaml` is
read from the same place under the same override rule, so a robot carrying a
local override keeps it. `mote-health.service` did not change at all — it runs
`pixi run health`, and only what that task points at moved.

## 7. Left open

`slip_monitor` is now the largest Python consumer at ~7 points and is
deliberately **not** a port candidate as it stands. Its maths is
`odom_residual.py`, ROS-free and shared with `tools/slip_replay.py`, and that
sharing is what set `slip.yaml`'s thresholds from real bags — a C++ port forks
the maths and quietly invalidates the calibration. The shared-maths problem has
to be answered first.

Reproducing the 17.1 figure, and so measuring this port against the live stack
rather than against a bench, needs the drive stack up on the robot. Everything
in §4 is a floor until that is run.
