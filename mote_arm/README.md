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
4. **We can borrow the calibration flow without the framework.** LeRobot's
   `lerobot-calibrate` is two phases — write each servo's homing offset so
   mid-travel reads 2048, then record every joint's range in one sweep — and
   both are plain register operations on a bus we already drive. `pixi run
   arm-calibrate` implements that flow directly (see `BENCH.md`).

Everything hardware lives in `mote_description/config/robot.yaml` (`arm:`
section): the single source of truth for the port, baud, servo IDs, per-joint
soft limits, zero offsets, and direction.

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
| `bus.py` | `FeetechBus` — thin `scservo_sdk` wrapper (ping, read position/health, torque, position goal, homing offset). Lazy SDK import. |
| `arm_driver` (node) | **Single bus owner.** Publishes `/joint_states` for the arm, accepts absolute goals on `arm/goal`, exposes `arm/set_torque`. `pixi run arm`. |
| `jog` (CLI) | Interactive per-joint jog. A *client* of the driver — publishes clamped `arm/goal`, calls `arm/set_torque`. `pixi run arm-jog`. |
| `arm_check` (tool) | Standalone enumeration + health + zero snapshot. Read-only; run with the driver stopped. `pixi run arm-check`. |
| `calibrate.py` / `arm_calibrate` | Two-phase range calibration: write the homing offsets, sweep every joint at once, emit `robot.yaml` limits. `pixi run arm-calibrate`. |
| `arm_offsets` (tool) | Read/back up/restore/set the servos' position-correction offsets. The recovery path if a calibration is interrupted. `pixi run arm-offsets`. |
| `poses.py` / `arm_pose` | Teach and replay named poses, and narrow limits to a working envelope. `pixi run arm-pose save\|list\|go\|limits\|delete`. |

## `zero` and `home` are different things

Worth stating plainly, because the two were both called "home" until 2026-07-28
and it confused an operator at the bench:

| Term | What it is | Where it lives |
|------|------------|----------------|
| **zero** | The encoder count that reads 0 rad. After calibration, the *middle of the joint's travel*. | `robot.yaml`, `arm.joints[].zero` |
| **home** | A taught *pose*, normally the arm's rest position. Nothing to do with 0 rad. | `~/.mote/arm_poses.yaml` |

So `arm-jog`'s command to drive a joint to 0 rad is `zero`, not `home` (`home`
still works and says so), and `pixi run arm-pose go home` moves to the rest pose.

## Where the soft limits come from

**`pixi run arm-calibrate`**, in two phases — the same shape as LeRobot's
`lerobot-calibrate`:

```
pixi run arm-calibrate                        # sweep, then centre the zeros
pixi run arm-calibrate -- --skip-homing       # ranges only; writes nothing
pixi run arm-calibrate -- --joints wrist_roll # redo one joint
```

You sweep the joints; everything else is automatic.

**Phase 1 — record the ranges.** You move every joint to both of its mechanical
stops, in any order, while one live table records **all six at once**. One Enter
ends it. The band emitted is the swept range pulled **inward** by `--margin`
(0.05 rad), because a hard stop is where the operator stopped pushing — a soft
limit has to sit short of it.

**Phase 2 — centre the zeros.** Each joint's 0 rad is moved to the *measured*
middle of the range just swept, by writing the servo's position-correction
register (EEPROM, `SMS_STS_OFS_L/H`, address 31). The servo reports
`present = actual - offset`, so this re-centres the joint's whole travel inside
the 0–4095 encoder frame. The arm can be left wherever the sweep ended.

**Why the centre comes from the sweep.** LeRobot's `calibrate()` opens with a
single `input("Move {robot} to the middle of its range of motion and press
ENTER")` and takes every motor's zero from that one pose. It is an awkward,
unbalanced position to hold all six joints in, and eyeballing the middle is less
accurate than the measurement the sweep is about to take anyway.

That ordering is *load-bearing for them*, though, not incidental:
`record_ranges_of_motion` is a plain `min`/`max` over raw positions with **no
wraparound handling at all**. Centring first is what guarantees the sweep never
crosses 0/4095, which is what makes plain min/max safe. Sweeping first, as we do,
means the sweep *can* cross it — so `SweepRecorder` unwraps, and the centre is
recovered from the unwrapped stream. The trade is a small heuristic (a jump of
more than half a revolution between samples is a crossing, which no hand-moved
joint can produce at 20 Hz) in exchange for deleting the awkward step. It also
means a wrapped sweep is handled rather than silently mis-recorded.

LeRobot is Apache-2.0, so borrowing their code would be permitted with
attribution; none is copied here — only the shape of the flow.

**Centring is what makes the limits describable at all.** A joint whose travel
straddles the encoder's 0/4095 boundary has a raw min and max that say nothing
about its real span, and no zero/limit pair in that frame can express it —
`rad_to_counts` clamps at the encoder edge. Measured on the real arm, **two of
six joints did exactly that** (`shoulder_pan` and `wrist_roll`). There is no
software workaround: the servo's own goal register is 0–4095 too.

Offsets are **modular** — `present = (actual - offset) mod 4096`, so an offset of
3056 and one of −1040 command the same thing. An arithmetic result outside the
register's ±2047 is therefore folded, never rejected. (Rejecting one aborted a
real calibration run before this was understood.)

It opens the serial bus directly, like `arm-check`, so run it with the driver
stopped: the driver reports radians about the very zero being replaced, and the
arm has to stay limp throughout. It asks before releasing torque (an unsupported
arm falls) and again before the EEPROM write.

Four things it refuses to guess at, rather than emit plausible-looking numbers:

- **Travel beyond one revolution.** A continuously-rotating joint has no stops
  to calibrate against and fits no single-turn frame. Reported as its own case,
  because unlike a wrap there is no remedy — leave it out with `--joints`.
- **A range too short to survive the margin at both ends.**

**A continuously-rotating joint is only detectable if you rotate it past a whole
turn**, which is the one case refused above. Rotated less, it is indistinguishable
from a joint with stops, and no threshold below a full turn helps: it would miss
most real cases while firing on a long-but-stopped joint. LeRobot sidesteps this
by hard-coding the SO-101's `wrist_roll` as a full-turn motor and skipping its
range entirely. This arm's `wrist_roll` measures 5.88 rad — 94% of a turn — so
whether it has real stops is worth settling by hand; if it spins freely, exclude
it with `--joints` and drive it in relative terms.
- Under `--skip-homing` only, where the zero is not being moved: **an encoder
  wrap**, and **a zero the joint could never reach** (a band excluding 0 rad —
  the defect in the pre-calibration `shoulder_pan` limits, whose
  `[0.010, 0.229]` does not contain 0). Neither can arise from the centred path,
  where the zero *is* the middle of what was swept.

**If it stops partway, the arm is recoverable.** The offset register lives only
in the servo, so overwriting it destroys the previous value. Before the first
write, the existing offsets are snapshotted to `~/.mote/arm_offsets_backup.yaml`;
each servo is then written, verified by read-back, *and* checked to have moved
its reading by exactly the delta written, before moving to the next. Any failure
stops immediately, names the servos already changed, and points at
`pixi run arm-offsets restore`. (An earlier version wrote without a backup and
died mid-run on a dropped serial read — hence both the snapshot and the guard in
`FeetechBus._read`, which turns a short reply into `None` instead of an
`IndexError`.)

Taught poses do not have to be re-taught: they are re-expressed about the new
zeros by exactly the shift the calibration computed, so each still points where
it always did, and the previous file is kept as `.bak`. Only a pose that lands
outside the new soft limits is named — that one was taught somewhere the arm
cannot reach, which is a decision rather than an arithmetic problem.

**It saves the result to `~/.mote/arm.yaml`** — per-robot state, not the repo —
and says so in one line. The numbers are not reprinted: the swept ranges were on
screen a moment ago, the limits are those pulled inward by `--margin`, and the
file itself keeps each value next to the measurement it came from.

That location is the point. Zeros and limits are measurements of *one physical
arm*: two robots with identical hardware have different ones, and the packaged
`mote_description/config/robot.yaml` is shared by the whole fleet and read-only
once installed from a channel. So the package keeps the *design* — ids, names,
direction, gains, and conservative defaults for an arm that has never been
calibrated — and `mote_arm.config` overlays this robot's measured
`zero`/`min`/`max` on top at load time. Delete the file to fall back to the
defaults. It is the same rule as `~/.mote/camera_calibration.yaml` and the site
bundles: `MOTE_HOME` is per-robot, the package is shared.

The file carries the measurement alongside the value — swept range, sample
count, margin, and the `homing_offset` written to the servo, which exists
nowhere else and is the only record if a servo is swapped. Writes go through a
temporary file, and the result is validated through `ArmConfig` before it lands,
because this is what supplies the soft limits that stop the arm.

**Taught poses are migrated for you.** A pose is stored as radians about the
zero, so moving the zero changes which physical position each number names — but
the correction is exactly the shift the calibration just computed, so the poses
are rewritten rather than re-taught. They keep pointing where they always did,
and the previous file is kept as `.bak`. A pose that would land outside the new
limits is reported rather than silently clamped: that means it was taught
somewhere the arm can no longer reach, which is a decision, not an adjustment.

**One confirmation, at the EEPROM write.** Everything else follows from having
run the command: torque is released without asking (the arm is already limp
unless a driver was killed outright, which is detected by reading the torque
register), and the result is saved without asking, since saving it is what the
command is for.

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

The committed `zero:` values are the arm's **as-found resting counts** read off
the robot, not measured mid-travel. A real calibration run showed why that is
bad: the parked pose sits within ~20 counts of a hard stop on `shoulder_lift`,
`elbow_flex` and `gripper`, so "0 rad" currently means "jammed against the
stop", and the committed bands (~0.2 rad) are a small fraction of the ~3.4–3.6
rad these joints actually travel.

See `BENCH.md` for the full runbook. In short:

1. `pixi run arm-check` — confirm every joint responds; note IDs.
2. `pixi run arm-calibrate` — centre the joints, sweep them, paste the emitted
   `arm.joints` block into `robot.yaml`, `pixi run build`. This sets `zero`
   *and* `min`/`max` together, which is the point: limits only mean something
   relative to the zero they were measured about.
3. Re-teach any poses it named (`pixi run arm-pose save <name>`), *after* the
   rebuild.
4. Jog each joint (`pixi run arm-jog`) and flip `invert` for any that moves
   opposite the expected sign. `invert` changes what the limits mean, so
   re-calibrate after changing it.

`arm-check -- --save-zero` still prints a bare `zero:` snapshot of the current
pose. It is a convenience, not calibration: it measures no range and writes no
offset, so the limits stay whatever they were.

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
