# mote_arm

Bring-up for the **SO-101 follower arm**: joint-state publishing, safe
joint-level control, and bench tools. This is the foundation for teleop /
episode recording and for replacing the fetch tree's pick/place stubs. There is
**no leader arm** — nothing here assumes leader-follower teleop.

## Stack decision: direct Feetech control (not LeRobot)

The task weighed **LeRobot** (native SO-101 support, calibration flow, dataset
format) against **direct Feetech control** (reuse the existing SCServo/servo
stack). We chose direct Feetech control:

1. **Same servos as the wheels.** The SO-101 uses Feetech STS-class servos on a
   serial bus — exactly what `mote_hardware` already drives via the vendored
   SCServo SDK. Position control, torque enable/disable, and feedback are a
   handful of register reads/writes we already understand. Reuse beats a new
   stack.
2. **LeRobot fights the lean Pi environment.** LeRobot pulls in `torch` and a
   large ML stack. The repo deliberately keeps `torch` *off* the Pi — depth and
   detection inference run in separate off-board `inference` pixi envs. Pulling
   that onto the Pi purely to move servos over serial is disproportionate. Our
   only new dependency is `feetech-servo-sdk` (`scservo_sdk`) — pure Python,
   aarch64-clean, the same SDK LeRobot itself uses under the hood, so this does
   not foreclose a LeRobot path later.
3. **Episode recording has a ROS-native path.** The repo already records
   provenance rosbags (`record_launch.py`). Teleop/episode capture can ride the
   same mechanism; we don't need LeRobot's dataset format to unblock the
   follow-up. If we later want that format for learning, we can convert bags or
   run LeRobot off-board.
4. **Calibration is cheap for a bring-up.** Per-joint soft limits + a home
   offset in `robot.yaml`, taught with a short scripted bench procedure (see
   `BENCH.md`) — no calibration wizard required.

Everything hardware lives in `mote_description/config/robot.yaml` (`arm:`
section): the single source of truth for the port, baud, servo IDs, per-joint
soft limits, home offsets, and direction.

### Why not an existing ROS 2 SO-101 stack?

Several exist and were reviewed (July 2026):

- [`legalaspro/so101-ros-physical-ai`](https://github.com/legalaspro/so101-ros-physical-ai)
  — the fullest arm stack: Feetech ros2_control driver, MoveIt 2, episode
  recording with LeRobot dataset export, torch isolated off the control path.
  Jazzy. But it is a **standalone arm on its own bus** and its teleop assumes a
  leader arm; we have neither.
- [`brukg/so_arm_100_hardware`](https://github.com/brukg/so_arm_100_hardware)
  — SO-100 ros2_control hardware interface; same standalone-arm shape.
- [`adityakamath/lekiwi_ros2`](https://github.com/adityakamath/lekiwi_ros2) —
  the closest platform to Mote (the LeKiwi mounts an SO-101 on a mobile base,
  and our servo IDs — arm 1-6, wheels 7/9 — follow its convention). As of this
  writing it drives the base and a pan-tilt payload only; its SO-101
  integration is planned, not shipped.

None of them address what dominates this package: the arm **sharing a serial
bus with the drive wheels** — bus ownership, ID collision, and the single-opener
guard. That problem has no shipped public solution, which is why `mote_arm`
exists rather than a dependency.

We are not outside this ecosystem either: `mote_hardware` builds on our fork of
[`adityakamath/SCServo_Linux`](https://github.com/adityakamath/SCServo_Linux)
(packaging fixes upstreamed, v1.0 packaged on the mote prefix.dev channel) —
the same SDK LeRobot and the stacks above sit on. When arm control folds into
`mote_hardware`'s ros2_control interface (task 231), evaluate
[`adityakamath/sts_hardware_interface`](https://github.com/adityakamath/sts_hardware_interface)
first: same SDK underneath, and it already supports mixed position/velocity
modes on one bus — though it targets Kilted (we run Jazzy), is not on
rosdistro, and is a fast-moving single-maintainer WIP, so the honest case for
it is upstream convergence, not saved effort.

## Wiring: the arm shares the drive-wheel bus

Verified on the robot: arm servos are IDs **1–6**, the drive wheels are **7**
and **9**, and all eight are on the one `/dev/mote_servos` (CH343) bus. The arm
needs no udev rule of its own. Two consequences are enforced in code, not just
documented:

- **ID collision** — an arm ID equal to a wheel ID would send arm commands to a
  wheel. `mote_arm.config` rejects that at load time when both share a port.
- **One opener only** — a serial port has no kernel-level exclusion, so a second
  process would interleave packets on the bus that *moves the robot*.
  `mote_arm.bus.FeetechBus.open()` scans `/proc` and refuses to open a port
  another process already holds, naming the offending PID. In practice: the arm
  driver cannot run at the same time as the robot base (`pixi run launch`,
  `mapping`, `robot`) — stop the base first (`pixi run kill`).

Lifting that restriction means moving arm control into the `mote_hardware`
ros2_control `SystemInterface`, so one process owns the bus. That is the natural
next step once the arm needs to move *during* a mission.

## Components

All bus I/O is isolated in `bus.py`; the config maths in `config.py` is
ROS-free and unit-tested (`test/`), so the safety-critical clamping and
conversions are verified without hardware.

| Piece | What it is |
|-------|------------|
| `config.py` | Parses the `arm:` section; encoder<->radian conversion + soft-limit clamping. |
| `bus.py` | `FeetechBus` — thin `scservo_sdk` wrapper (ping, read position/health, torque, position goal). Lazy SDK import. |
| `arm_driver` (node) | **Single bus owner.** Publishes `/joint_states` for the arm, accepts absolute goals on `arm/goal`, exposes `arm/set_torque`. `pixi run arm`. |
| `jog` (CLI) | Interactive per-joint jog. A *client* of the driver — publishes clamped `arm/goal`, calls `arm/set_torque`. `pixi run arm-jog`. |
| `arm_check` (tool) | Standalone enumeration + health + home snapshot. Read-only; run with the driver stopped. `pixi run arm-check`. |
| `calibrate.py` / `arm_calibrate` | Guided range calibration: sweep each joint to its stops, emit `robot.yaml` limits. `pixi run arm-calibrate`. |
| `poses.py` / `arm_pose` | Teach and replay named poses, and narrow limits to a working envelope. `pixi run arm-pose save\|list\|go\|limits\|delete`. |

## Where the soft limits come from

**`pixi run arm-calibrate`.** It walks the six joints in servo-command order;
for each, you move the limp joint gently to both mechanical stops by hand while
it watches the encoder live, and it emits a ready-to-paste `arm.joints` block:

```
pixi run arm-calibrate                        # home = the mid-point of each sweep
pixi run arm-calibrate -- --home capture      # pose a zero per joint instead
pixi run arm-calibrate -- --joints wrist_roll # redo one joint
```

The band it emits is the swept range pulled **inward** by `--margin` (0.05 rad),
because a hard stop is where the operator stopped pushing — a soft limit has to
sit short of it. It opens the serial bus directly, like `arm-check`, so run it
with the driver stopped: the driver reports radians about the very `home` being
replaced, and the arm has to stay limp throughout. The one write it makes is
torque *off*, and it asks first, because an unsupported arm falls when it goes
limp.

Three things it refuses to guess at, rather than emit plausible-looking numbers:

- **An encoder wrap.** A joint whose travel crosses the 12-bit 0/4095 boundary
  has a raw min/max that says nothing about its span, and no `home`/limit pair
  in that scheme can describe it (`rad_to_counts` clamps at the encoder edge).
  The wrap is detected, reported, and the joint keeps its existing values with
  the reason on the line above it. Fix it by re-homing that servo so its
  mid-range sits away from the boundary, then sweep it again.
- **A zero it could never reach.** If `home` lands within a margin of a stop,
  the band would exclude 0 rad and the joint could not be commanded home — which
  is exactly the defect in the pre-calibration `shoulder_pan` limits.
- **A range too short to survive the margin at both ends.**

It also warns, *before* emitting anything, which taught poses a changed `home`
invalidates and by how many radians each has shifted. What was measured is
recorded in `~/.mote/arm_calibration.yaml` for provenance.

### Named poses, and narrowing the envelope

The base layer teaches map positions by driving there and running
`pixi run save-zone`; the arm's analogue is `pixi run arm-pose`. Pose the limp
arm by hand, capture it, and later command it back:

```
pixi run arm-pose save reachy     # read-only capture of the current pose
pixi run arm-pose list            # taught poses, and how far the arm is from each
pixi run arm-pose go reachy       # the only command that moves; asks first
pixi run arm-pose limits          # emit limits spanning the taught poses
```

Poses live in `~/.mote/arm_poses.yaml` (`MOTE_HOME` overrides `~/.mote`) —
per-robot data, since a pose only means anything for one physical arm and its
calibration. **Changing `home` invalidates stored poses** (they are recorded in
radians about it), so re-teach after any re-home.

`arm-pose limits` is **not** the calibration path. It widens *outward* from the
extremes of the taught poses, so it can only describe where the arm has already
been, and it never learns where the stops are: a joint that barely moved between
two poses gets a near-zero band. Its remaining use is the opposite direction —
**narrowing** to a working envelope on top of calibrated hard-stop limits, when
a task wants a joint held tighter than the mechanism allows. Take the hard stops
from `arm-calibrate` first; reach for `limits` only to pull them in.

The values committed in `robot.yaml` today still come from the old envelope
method, and are flagged as such in that file, pending a calibration pass on the
real arm (`BENCH.md` step 3).

`arm-pose go` refuses any move whose largest single-joint travel exceeds
`--max-travel` (0.35 rad by default), so a stale pose or a bad limit change
cannot turn into a large unexpected swing. Raise it deliberately for a known-long
move (`--max-travel 4.0` for the full `home` <-> `reachy` swing).

It **streams** setpoints at 20 Hz rather than commanding the destination in one
jump, so the arm moves continuously at `--speed` (0.5 rad/s default) instead of
lurching between waypoints. Supervision is by *lag* — how far the arm trails the
setpoint it was given: sustained lag beyond `--max-lag` (0.15 rad) for
`--stall-time` means it is no longer keeping up, and the move stops where it is.
Measured lag on the full swing is a steady 0.07-0.10 rad.

Because `arm_driver` and `arm_check` both open the serial port, run **one at a
time**, never both.

## Torque policy

Nothing moves without an explicit command.

- **Startup (`arm_driver`):** torque **OFF** — the arm is limp/back-drivable.
  Have the arm physically supported or resting in a stable pose before power.
  A servo that fails enumeration, or cannot be *confirmed* in position mode
  (mode changes are verified by read-back, never blind-written to EEPROM), is
  excluded from control entirely: its state is still published, but it accepts
  no goals — in wheel mode a position goal is obeyed as a speed.
- **First goal:** a goal on `arm/goal` (or `jog`'s `+`/`-`/`home`) enables
  torque (holding the current pose) and then moves. Torque is engaged
  per-joint: a joint whose position cannot be read at that instant stays limp,
  receives no goals, and is retried on the next command rather than silently
  abandoned. Goals are soft-clamped to the per-joint limits from `robot.yaml`
  in the driver — the authoritative clamp — and again client-side in `jog` for
  immediate feedback.
- **`arm/set_torque false`** (or quitting `jog`): torque OFF — limp. The
  enable direction reports `success: false` and names any joints left limp.
- **Shutdown (`arm_driver`):** torque OFF — the arm is left safely limp.

## Control interfaces

- `/joint_states` (`sensor_msgs/JointState`) — arm joint positions in rad.
  robot_state_publisher animates the arm in TF from these (the joints are in the
  URDF; see `mote_description/urdf/mote.urdf.xacro`, `arm:=true`).
- `arm/goal` (`sensor_msgs/JointState`) — absolute per-joint goal positions
  (rad); only the `name`/`position` fields are read.
- `arm/set_torque` (`std_srvs/SetBool`) — `true` = hold, `false` = limp.

## Physical note (GitHub #2)

The camera does not fit when the SO-101 arm is attached. That is an unresolved
mechanical clash, tracked separately — not addressed here. `arm_driver` is not
part of the mission bringup; run it explicitly with `pixi run arm`.

## Calibration

The committed `home:` values are the arm's **as-found resting counts** read off
the robot, not taught mechanical zeros. So "0 rad" currently means "the pose it
was parked in". That is a deliberately safe reference — jog steps stay small
because they start from the measured position — but it is not yet a real zero.

See `BENCH.md` for the full runbook. In short:

1. `pixi run arm-check` — confirm every joint responds; note IDs.
2. `pixi run arm-calibrate` — sweep every joint to its stops; paste the emitted
   `arm.joints` block into `robot.yaml` and `pixi run build`. This sets `home`
   *and* `min`/`max` together, which is the point: limits only mean something
   relative to the zero they were measured about.
3. Jog each joint (`pixi run arm-jog`) and flip `invert` for any that moves
   opposite the expected sign. `invert` changes what the limits mean, so
   re-calibrate after changing it.

`arm-check -- --save-home` still prints a bare `home:` snapshot of the current
pose. It is a one-joint-at-a-time convenience, not calibration: it measures no
range, so the limits stay whatever they were.

## Verified on hardware

Run against the real arm on 2026-07-25:

| Check | Result |
|-------|--------|
| Bus enumeration | all 6 joints respond; 5.1–5.2 V, 26–29 °C, load 0 |
| `/joint_states` | 6 arm joints at a steady 20.0 Hz, no jitter while limp |
| Startup torque | driver comes up limp; `TORQUE_ENABLE` reads 0 on every joint |
| Port guard | `arm_check` refused the bus while the driver held it, naming its PID |
| Torque engage | seeding goals before enabling moved the arm **0.00000 rad** |
| Jog motion | `elbow_flex` jogged −0.05 rad per step and returned; `/joint_states` tracked |
| Soft-limit clamp | repeated `+` past the limit held at `+0.103` — no further motion |
| Shutdown | SIGINT exits 0, no traceback, torque off, port released |
| Pose replay | full `home` <-> `reachy` move (3.19 rad / 183 deg) completed both ways, streamed at 0.5 rad/s, lag steady 0.07-0.10 rad, settling within 0.026-0.041 rad |
| Servo gains | `arm-gains apply` wrote and verified Kp=32 on all six servos; temps unchanged at 27-30 C after the full move |

`arm-pose go` streams setpoints continuously (see above) and stops when the arm stops keeping up, rather than holding against a load it
cannot overcome. At the shipped `Kp = 16` that guard correctly halted the
`reachy` replay after ~0.13 rad; with `Kp = 32` applied the same move runs to
completion.

## Position accuracy and the servo gains

The arm shipped with `Kp = 16` on every servo, which left a permanent
steady-state error under load: the servo settles where `Kp x error` balances the
holding torque, and `Ki = 0` never integrates that droop away. Measured on
`elbow_flex`, commanded -0.200 rad from rest:

| Kp | reached | steady error | load (of 1000) | Kp x error |
|----|---------|--------------|----------------|------------|
| 16 (as shipped) | -0.129 rad | 0.071 rad | 196 | 1.14 |
| 32 (applied) | -0.167 rad | 0.033 rad | 176 | 1.05 |

That is droop, **not** torque saturation: error halves as Kp doubles while load
stays near 180-196, nowhere near the 1000 that saturation would pin it to, and
`Kp x error` stays constant. The servo was using about a fifth of the effort
available to it, so the 5.1-5.2 V supply was never the binding constraint.

`Kp = 32` (matching the drive wheels and the STS3215 factory default) is now
applied to all six servos and recorded in `robot.yaml`'s `arm.gains`. With it,
the arm completes the full 3.19 rad (183 deg) `home` <-> `reachy` move in both
directions without stalling, holding a residual error of 0.02-0.06 rad
(1-3.5 deg) — the remaining proportional droop.

Gains live in servo EEPROM, so they are invisible config that a servo swap would
silently revert. `robot.yaml` is the source of truth and `pixi run arm-gains`
reconciles hardware with it:

```
pixi run arm-gains show     # read-only comparison against robot.yaml
pixi run arm-gains apply    # write and verify (asks first; EEPROM is persistent)
```

`apply` reports success only when a confirmed read-back matches, because an
EEPROM read-back races the relock: a single read taken too soon returns a
garbled value (observed: 250) and makes a successful write look failed. The bus
layer reads twice and trusts the value only when both agree.

Closing the residual 1-3.5 deg would mean a small `Ki`, which is left alone for
now: integral windup on an arm risks a lunge when a load is removed, so it wants
a deliberate test on an unloaded joint first. Supply voltage is worth revisiting
only after that, since the arm still has not demanded full torque.
