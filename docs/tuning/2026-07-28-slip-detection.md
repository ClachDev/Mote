# Wheel-slip detection from the odometry residual

Mote already carries two independent motion sources: wheel odometry, and
kinematic_icp's scan-matched pose. kinematic_icp *takes* the wheel odom as its
prior and corrects it against the scan, so the correction is already a
measurement of how wrong the wheels were — a slip signal, on existing hardware,
with no IMU. Task 165 established from real bags that the quiet baseline is tight
enough for an event to stand out, and that is why the BNO085 is not justified.
This note is how that signal became a live health check: what was measured, what
the thresholds are, and what fired.

Everything here is reproducible:

```bash
# The distribution and the verdicts, over the real mapping bags.
pixi run -- python mote_bringup/tools/slip_replay.py ~/.mote/bags/mapping/*/
# The whole-session survey (path length, yaw, impossible-velocity frames).
pixi run -- python mote_bringup/tools/odom_health.py ~/.mote/bags/mapping/*/
```

`slip_replay.py` drives the *same* `ResidualEstimator` and `classify` the
`slip_monitor` node runs. A threshold calibrated offline only means something if
the robot computes the same number.

## The time-sync question, settled

Task 165 flagged that `odom_health.py` resamples 100 Hz wheel odom onto 10 Hz ICP
stamps, and suspected the yaw residual was inflated by stamp skew — at 90 deg/s,
10 ms of skew manufactures ~9 deg/s of apparent disagreement.

**Measured, there is no such skew.** Shifting the wheel stream by a candidate lag
and picking the lag that minimises the yaw residual puts the optimum within
±10 ms on four of the five usable bags (+50 ms on the fifth, which has only 53
moving windows), and the curve is nearly flat there: ±40 ms moves the p50 by
under 0.6 deg/s, so no plausible offset accounts for a 3 deg/s residual.

| bag | best lag | yaw p50 at that lag | yaw p50 at zero lag |
|---|---|---|---|
| 20260706_133149 | +50 ms | 2.31 deg/s | 2.61 deg/s |
| 20260706_135320 | +10 ms | 2.72 | 2.85 |
| 20260706_172607 | −10 ms | 2.88 | 2.96 |
| 20260706_192836 | +10 ms | 3.13 | 3.14 |
| 20260706_193037 | −10 ms | 3.04 | 3.05 |

What the yaw residual actually is, is **scan-match jitter**, which averages down
as the comparison window grows while a real slip does not — a slip accumulates
displacement. Over the five bags, |yaw| residual p50 falls from ~3.0 deg/s at a
0.1 s window to ~1.1 at 1.0 s and ~0.5–0.9 at 2.0 s.

So the prerequisite is discharged the second way the task allowed: **detection is
on translation only.** The yaw residual is computed and published for logging, so
it can be thresholded later if a use appears, but nothing keys off it. The
measurements say it could not be: on the same bags and the same windows, the
relative yaw residual reaches p99 of 0.86–1.10 (i.e. as large as the yaw rate
itself) where relative *translation* reaches p99 of 0.04–0.53. No yaw threshold
exists that a hard turn would not trip.

The estimator also interpolates **both** streams to the window's own endpoints,
rather than resampling one onto the other's stamps, which removes the
sampling-grid component of the error outright.

## Window length

A longer window is quieter, but it also delays detection, and past ~1 s the
translation residual has stopped improving much:

| window | 135320 trans p99 | 193037 trans p99 | 172607 trans p99 |
|---|---|---|---|
| 0.5 s | 0.0078 | 0.0111 | 0.0531 |
| 1.0 s | 0.0065 | 0.0073 | 0.0342 |
| 2.0 s | 0.0051 | 0.0098 | 0.0494 |

**1.0 s is the choice.** At 0.5 s two of the quiet bags already produce single
windows above 0.03 m/s; at 2.0 s the tail does not improve further and detection
latency doubles. Total latency to a reported verdict is the window plus the
`hold` time, ~1.5 s, which the sim run below shows end to end.

## Thresholds

A verdict needs the residual to clear **both** an absolute floor and a fraction of
the motion reported. The floor rejects noise at low speed; the fraction rejects a
large residual that is merely a large motion measured slightly differently.

Measured over the two bags with no event in them (394 s of driving):

| | 20260706_135320 | 20260706_193037 |
|---|---|---|
| speed residual p50 | +0.0003 m/s | +0.0002 m/s |
| speed residual p99 | +0.0062 | +0.0210 |
| relative p99 | +0.040 | +0.179 |
| stdev | 0.0027 | 0.0042 |

`slip_speed: 0.030` m/s sits ~1.4x above the worst quiet p99 and ~7x above the
better one; `slip_fraction: 0.25` sits above the 0.179 worst quiet relative p99.
The ICP-fault direction uses the same pair. Independently, an ICP speed above
`max_wheel_speed x 1.15` (0.251 m/s) is a fault whatever the wheels say, because
the drive cannot produce it. `test_odom_residual.py` asserts the thresholds stay
above these measured p99s, so lowering one fails a test rather than quietly
enabling false positives.

These bags are one building, with no glass and no featureless corridor. **Treat
the thresholds as provisional** until they have been seen on more varied floors;
`$MOTE_HOME/slip.yaml` overrides them per robot for exactly that reason.

## What fired

Six verdicts across the six bags, ~38 minutes of recording. Every one was checked
against the recorded `/scan_filtered` independently of the detector, and **every
one is a real event** — the bags were less "known-good" than assumed. Two bags
are silent throughout.

| bag | t | verdict | independent evidence |
|---|---|---|---|
| 133149 | 59.1–62.7 s | icp_fault | Wheels report exactly zero translation (in-place rotation); the ICP pose jumps ~12 cm at the onset of the turn. Scan minima do change, so the robot really is rotating. |
| 135218 | 17.5–19.9 s | icp_fault | The robot squeezes past an obstacle — `min range` falls to 0.127 m and the 90° sector jumps 0.26 → 1.31 m as a corner leaves view. ICP reports 0.356 m/s, above the 0.251 m/s the drive can produce. |
| 172607 | 295.5–297.9 s | icp_fault | ICP reports 0.326 m/s. Impossible for the drive; a scan-match excursion. |
| 172607 | 321.2–326.1 s | icp_fault → slip | **Robot stuck.** From t=314 the scan is frozen (0.457 / 1.398 / 0.301 m, unchanging) while the wheels report bursts of 0.15–0.22 m/s and ±28 deg/s spins — Nav2's recovery behaviours on a robot that is not moving. |
| 192836 | 46.1–50.7 s | slip | **Robot jammed.** Scan frozen at min range 0.181 m for ~9 s while the wheels report 0.21 m/s and +15 deg/s. ~1 m of travel and ~80° of rotation the lidar never saw. |
| 135320, 193037 | — | none | 394 s of driving, clean throughout. |

No stream stalled in any of these: `/tf` and `/scan_filtered` run at their
nominal 100 Hz / 10 Hz with no gap above 0.5 s, so the frozen scans are the
robot genuinely not moving, not a dropped sensor.

That last distinction turned out to matter enough to change the code. A stalled
lidar looks *exactly* like slip — the window freezes at the last ICP pose while
the wheels keep turning, so the apparent residual grows without bound. That is
the one failure mode in which this node would blame the wheels for a sensor
dropout, so `ResidualEstimator` carries an explicit staleness guard: a source
older than `max_lag` yields no verdict, not a stale one.

## Demonstrated live, in the sim

`sim_launch.py` with the `wheel_mu` friction knob (task 165), the real
`slip_monitor` running alongside with `use_sim_time:=true`, driving straight at
0.18 m/s in `mote_world.sdf`:

| run | result |
|---|---|
| `wheel_mu:=1.0`, 25 s | Quiet for 20 s — residual p50 +0.0005 m/s. At t≈20 s the robot reaches the wall 3 m away; the residual crosses the threshold at **t=20.6 s** and reaches +0.179 m/s (relative 1.00 — completely stopped, wheels still reporting 0.18 m/s). Reported as `slip` at t≈21.2 s. 36 of 125 moving windows over threshold. |
| `wheel_mu:=0.4`, 12 s (no wall contact) | Residual p50 +0.0002, p99 +0.0099, max +0.0106 m/s; relative max 0.059. **Zero** windows over threshold. |
| `wheel_mu:=0.05`, 12 s (no wall contact) | Residual p50 +0.0002, p99 +0.0100, max +0.0107 m/s; relative max 0.059. **Zero** windows over threshold — indistinguishable from μ=0.4. |

Two things follow. The detector catches an obstruction within ~1 s of contact and
is otherwise silent — the same physical situation as the two real-bag events,
end to end through the real node. And **lowering `wheel_mu` is not on its own a
slip rig**: at 0.18 m/s this robot's traction demand is so far below what even
μ=0.05 supplies that the residual is unchanged to three decimal places across a
20x range of friction. The benchmark's `--slip` flag exercises the *pose*
pipeline, as its README says; to exercise *this* node, obstruct the robot.

Note the sim reports `slip`, not `stuck`, and correctly so: the wheels are
turning, the robot is not. `stuck` is the narrower case where the wheels
themselves report nothing despite a command.

## What this does not cover

- **Yaw.** Deliberately unthresholded, above. A pure-rotation slip in which
  translation stays honest would not be caught.
- **In-place rotation.** Wheel translation is exactly zero during a spin, so the
  relative term is always −1.0 and only the absolute floor protects the
  ICP-fault direction. Measured ICP translation during steady ±28 deg/s spins is
  ≤0.004 m/s, ~7x below the floor, so there is margin — but it is margin, not a
  structural guarantee.
- **One building.** See the threshold caveat above.
- **Hardware.** The live path is proven in the sim and by `test_slip_monitor.py`;
  it has not yet run on the Pi. The detection quality is proven on real robot
  data through the shared estimator.
