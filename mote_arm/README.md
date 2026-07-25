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
| `poses.py` / `arm_pose` | Teach and replay named poses, and derive soft limits from them. `pixi run arm-pose save\|list\|go\|limits\|delete`. |

## Named poses, and where the soft limits come from

The base layer teaches map positions by driving there and running
`pixi run save-zone`; the arm's analogue is `pixi run arm-pose`. Pose the limp
arm by hand, capture it, and later command it back:

```
pixi run arm-pose save reachy     # read-only capture of the current pose
pixi run arm-pose list            # taught poses, and how far the arm is from each
pixi run arm-pose go reachy       # the only command that moves; asks first
pixi run arm-pose limits          # emit robot.yaml limits spanning the taught poses
```

Poses live in `~/.mote/arm_poses.yaml` (`MOTE_HOME` overrides `~/.mote`) —
per-robot data, since a pose only means anything for one physical arm and its
calibration. **Changing `home` invalidates stored poses** (they are recorded in
radians about it), so re-teach after any re-home.

The committed soft limits are **not guesses**: they are the envelope of poses a
human physically posed the arm into and vetted, widened by a 0.10 rad margin
(`arm-pose limits`). Every position inside the band lies between two vetted
poses. Joints that barely moved between poses get a correspondingly tight band —
that is the design, not a defect: nothing may travel further than a human has
demonstrated is safe. Widen it by teaching another pose and re-running `limits`.

`arm-pose go` additionally refuses any move whose largest single-joint travel
exceeds `--max-travel` (0.35 rad by default), so a stale pose or a bad limit
change cannot turn into a large unexpected swing.

Because `arm_driver` and `arm_check` both open the serial port, run **one at a
time**, never both.

## Torque policy

Nothing moves without an explicit command.

- **Startup (`arm_driver`):** torque **OFF** — the arm is limp/back-drivable.
  Have the arm physically supported or resting in a stable pose before power.
- **First goal:** a goal on `arm/goal` (or `jog`'s `+`/`-`/`home`) enables
  torque (holding the current pose) and then moves. Goals are soft-clamped to
  the per-joint limits from `robot.yaml` in the driver — the authoritative
  clamp — and again client-side in `jog` for immediate feedback.
- **`arm/set_torque false`** (or quitting `jog`): torque OFF — limp.
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
2. Pose each joint at its mechanical zero, run `pixi run arm-check -- --save-home`,
   paste the printed `home:` counts into `robot.yaml`.
3. Jog each joint to its safe extremes and set `min`/`max` (rad) in `robot.yaml`;
   flip `invert` if a joint moves opposite the expected sign.

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
| Pose replay | `go home` reached target within 0.006 rad; `go reachy` stopped itself after ~0.13 rad when the joint stopped progressing |

`arm-pose go` walks a move in bounded increments (`--step`, 0.20 rad default)
and stops on a stall rather than holding against a load it cannot overcome —
which is what halted the `reachy` replay, correctly, given the droop above.

## Known limitation: joints settle short of their target (proportional droop)

A commanded position is approached, not reached. Measured on `elbow_flex`,
commanded -0.200 rad from rest:

| Kp | reached | steady error | load (of 1000) | Kp x error |
|----|---------|--------------|----------------|------------|
| 16 (as shipped) | -0.129 rad | 0.071 rad | 196 | 1.14 |
| 32 (wheel/factory default) | -0.167 rad | 0.033 rad | 176 | 1.05 |

**This is a tuning problem, not a power problem.** Doubling Kp halved the error
while the load stayed near 180-196 — nowhere near the 1000 that torque
saturation would pin it to, and `Kp x error` stayed essentially constant. The
servo is settling exactly where its proportional output balances the holding
torque. With `Ki = 0` (as shipped) there is no integral term to erase that
droop, so the error is permanent for as long as the load is present.

Two contributing factors, both in servo EEPROM:

- **`Kp = 16` on every arm servo**, against `Kp = 32` on the drive wheels and
  the STS3215 factory default. Half the gain is double the droop.
- **`Ki = 0`**, so steady-state error is never integrated away.

Not yet applied — changing them writes to servo EEPROM, which is a persistent
hardware-config change and is deliberately left as an explicit decision. Read
the current values with `pixi run arm-check` plus the register probes described
in the bring-up notes. When changing Kp, note that the EEPROM read-back races
the relock: unlock, write, relock, wait ~150 ms, then read twice and trust the
value only when the two reads agree (a single read has been observed returning
a garbled 250).

Supply voltage measures 5.1-5.2 V against the STS3215's 7.4 V rating, so headroom
is genuinely limited and may still cap what the arm can lift once the gains are
right — but the measurements above show the servo is currently using only about
a fifth of the effort available to it, so voltage is not what is stopping it
today.
