# Gating kinematic_icp against the drive envelope — evidence

kinematic_icp occasionally emits a pose implying a body speed the drive cannot
produce. This is the measurement of whether that matters, where the threshold
can sit, and what gating it costs.

Tools: `mote_bringup/tools/odom_health.py` (scores a bag),
`icp_excursions.py` (characterises each excursion), `icp_gate_replay.py`
(feeds a bag's odometry through the real `icp_odom_gate` node and records what
it broadcasts). Bags: `~/.mote/bags/mapping/`, three real mapping sessions.
Raw output in `2026-07-28-icp-velocity-gate/`.

## 1. Do the jumps matter?

They are rare — 10 frames across 25 minutes — and always isolated single
frames, so the first question was whether the scan match simply re-registers on
the next scan and gives the displacement back.

**It does not.** Wheel odometry is the local reference (it does not drift metres
per second, which is what an excursion claims), so the change in the
ICP-minus-wheel along-track gap across an excursion separates the two cases: a
*spike* would show the gap returning, a *step* would show it kept.

```
                              gap rate before   gap rate after
20260706_192836  7 excursions   +0.0031 m/s      +0.0033 m/s
20260706_133149  3 excursions   +0.0223 m/s      +0.0202 m/s
```

The rate either side is the same: nothing is given back. Each excursion is a
permanent step in `odom->base`, and therefore in the map frame and in every zone
taught in it.

`20260706_133149` settles it beyond argument. The robot is **stationary** for
that session — the wheels report 0.5 m of travel in 1218 s — and ordinary
intervals show an ICP-vs-wheel gap of |p99| 0.0001 m, i.e. the scan match holds
still essentially perfectly. Yet three frames fabricate **+0.178 m** of
displacement, one of them 0.121 m in a single 0.1 s scan (1.2 m/s against a
0.218 m/s limit). Those three frames are 100% of that session's odometry error.
Wheel slip cannot explain it in the required direction: slip makes the *wheels*
over-read, never the lidar.

The excursions also correlate with what the robot was doing: in the driving bag
they occur at 0.175–0.219 m/s, essentially full speed, and at a median yaw rate
of 21.5 deg/s against a session median of 4.8.

So: worth fixing.

## 2. Where can the threshold sit?

Two candidate bounds, both from `robot.yaml`'s measured `max_wheel_speed`
(0.218 m/s) and `wheel_separation` (0.22 m).

**The joint per-wheel bound `|v| + S/2·|w|` — the one the Nav2 critic uses —
does not work here.** In `20260706_192836` the *wheel odometry itself* exceeds it
in 18.98% of intervals, and legitimate ICP intervals reach ×1.44 of the limit
while the mildest excursion is also ×1.44. The two populations overlap
completely. The cause is the yaw term: ICP is resampled at ~10 Hz and a small
stamp misalignment during a fast turn inflates the implied yaw rate (the caveat
already recorded in `odom_health.py`).

**Translation and yaw bounded separately do work.** Histogramming every interval
across all three bags:

| | legitimate | excursions |
| --- | --- | --- |
| translation | mass ends at 0.230 m/s; one lone sample at 0.245 (t=81.8 s, isolated from every excursion cluster) | 0.273, 0.276, 0.283, 0.304, 0.306, 0.355, 0.384, 0.387, 0.433, 1.197 |
| yaw rate | max 1.974 rad/s | — |

The band 0.245–0.273 m/s is empty, and `max_wheel_speed × 1.15 = 0.251 m/s`
lands in it: ×1.02 above the highest legitimate sample, ×1.09 below the mildest
excursion. That is the same ×1.15 `odom_health.py` already reports at, so the
gate's rule and the health tool's rule are one number rather than two.

The yaw bound is free insurance. The chassis maximum is
`2·max_wheel_speed/wheel_separation` = 1.982 rad/s, and ICP's fastest measured
turn is 1.974 rad/s — right at it, never over. A bound at ×1.15 (2.279 rad/s)
therefore never fires on this data but still catches a yaw excursion, which
translation alone would miss and which hurts a map more.

## 3. What the gate does

kinematic_icp broadcasts `odom->base` itself, and a TF broadcast cannot be
retracted, so nothing downstream can undo a bad transform. It is therefore
configured to publish only its odometry topic, in a frame of its own
(`odom_icp`), and `mote_nav::IcpOdomGate` owns `odom->base`: it accumulates
ICP's increments, and where one exceeds the envelope it accumulates the **wheel
increment** for that interval instead.

The wheel increment is the right substitute rather than a clamp: in
`20260706_133149` the wheels correctly say "not moving", giving ~0 where a clamp
would still have admitted 0.025 m. It is read from TF through the same
`odom_wheel` leaf kinematic_icp takes its own prior from, so the two cannot
disagree about what the wheels did; a clamp is the fallback if that lookup fails.

kinematic_icp is unaffected — its prior comes from that TF leaf, never from its
own output — so its internal pose keeps the excursion while the published one
does not. The gate's track is then permanently offset from ICP's internal track
by exactly the excursions it absorbed, and tracks it exactly otherwise. That is
the intent, and `IcpOdomGate.AbsorbedExcursionLeavesAConstantOffsetNotADrift`
pins it.

## 4. Before / after

`icp_gate_replay.py` feeds each bag's recorded streams to the **compiled gate
node** over ROS and records its broadcasts; `odom_health.py` then scores the
result. Nothing offline re-implements the decision.

```
                                excursions   icp path    max icp speed
20260706_192836   before             7        10.97 m       0.433 m/s
                  after              0        10.88 m       0.245 m/s
20260706_133149   before             3         0.74 m       1.197 m/s
                  after              0         0.56 m       0.231 m/s
20260706_193037   before             0        13.82 m       0.229 m/s
                  after              0        13.82 m       0.229 m/s
```

Wheel path length is 11.16 m, 0.54 m and 13.75 m respectively. The gate removes
+0.097 m and +0.178 m of fabricated path — exactly the sums measured in step 1 —
taking `133149` from +37% over the wheels to −3.8%.

**`20260706_193037` is the no-regression control.** It contains no excursions,
and every figure `odom_health.py` reports for the gated bag is identical to the
original, to the last printed digit: path length, yaw travelled, all three
residual percentiles, and all six speed percentiles. With nothing to reject the
gate is a pure pass-through, which is the claim that matters for normal
operation.

## 5. No regression in normal operation

The gate must not clip legitimate motion. Two checks, both in the sim.

`pixi run sim-test` passes with the gate owning the edge — 0.600 m commanded
forward measured as 0.600 m, 2.0 rad commanded spin measured as 2.001 rad, both
read back through the gated `odom->base` (`sim-smoke.log`).

`bench.py --worlds mote_world.sdf --trials 3`, gated against an unmodified
worktree at the same commit, run one at a time on an otherwise idle machine:

| | trials | goals | ATE rmse (m) per trial | mean |
| --- | --- | --- | --- | --- |
| baseline | 3/3 | 9/9 | 0.072, 0.064, 1.173 | 0.436 |
| gated | 3/3 | 9/9 | 0.075, 0.069, 0.063 | 0.069 |

Every goal succeeds either way, with no aborts and no recoveries: **no
regression**, which is what this run is here to establish.

The gate logged **zero rejections** across every sim trial, which is the
expected result — Gazebo's scan is noise-free and the excursions being gated are
a real-sensor phenomenon. It also means the sim cannot demonstrate the gate
*working*; only that it costs nothing when it has nothing to do. Bag replay
(§4) is what shows it working.

Read the ATE column with care rather than as a win. The baseline's third trial
blew up to 1.173 m with a 55 s middle goal, and a run-to-run spread that wide at
n=3 is not something three more trials would settle. What can be said is that
the large excursion happened without the gate and that the gated trials cluster
tightly; what cannot be said, from this, is by how much the gate improves
localisation.

## 6. Interaction with slip detection (#77)

`slip_monitor` landed independently and reaches the same threshold from the
other direction: it *reports* an `icp_fault` when the lidar claims a body speed
above `max_wheel_speed x 1.15` — the very frames this gate *removes*. Its own
measurements are in `2026-07-28-slip-detection.md`, and one of its worked
examples (bag 172607, 0.326 m/s) is an excursion of exactly this kind.

The two are complementary, but only if they are wired apart. The monitor read
`odom->base` through TF, which is now the gated edge, where a speed above the
envelope cannot occur by construction — so its primary `icp_fault` branch would
have become unreachable, and a scan match degrading behind a working gate would
have been reported by nobody while the monitor went on publishing OK.

So kinematic_icp still broadcasts, inverted, as the leaf `base -> odom_icp`
(`invert_odom_tf` swaps the frame ids as well as the transform, which is what
keeps `base_footprint` from acquiring a second parent), and `mote_launch.py`
points the monitor's `odom_frame` at it. The monitor therefore still sees the
raw track it was validated against — its thresholds were tuned on bags whose
`odom->base` *is* raw ICP — while navigation runs on the gated edge. The
division is: the gate protects the map frame, the monitor reports that the
scan match needed protecting.

`test_the_slip_monitor_watches_the_ungated_lidar_track` pins it, because nothing
about getting this wrong is visible at runtime.

## Caveats

* The threshold is calibrated against three bags from one robot on one floor.
  It is a fraction of a measured hardware constant rather than a magic number,
  so it moves with `robot.yaml`, but the ×1.15 slack is empirical.
* A *sustained* misregistration — many consecutive frames each individually
  within the envelope — is not caught by this and would not be. Every excursion
  measured so far is a single isolated frame.
* The gate bounds translation and yaw separately. The tighter joint per-wheel
  constraint is unusable until the ICP and wheel streams are properly
  time-synced; that is the same blocker as `odom_health.py`'s yaw caveat.
