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
| `arm_check` (tool) | Standalone enumeration + health + udev-line helper + home snapshot. Run with the driver stopped. `pixi run arm-check`. |

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

Read-only and zero-motion checks run against the real arm (2026-07-25):

| Check | Result |
|-------|--------|
| Bus enumeration | all 6 joints respond; 5.1–5.2 V, 26–29 °C, load 0 |
| `/joint_states` | 6 arm joints at a steady 20.0 Hz, no jitter while limp |
| Startup torque | driver comes up limp; `TORQUE_ENABLE` reads 0 on every joint |
| Port guard | `arm_check` refused the bus while the driver held it, naming its PID |
| Soft-limit clamp | goal 5.0 rad clamped to the limit, logged, **0.00000 rad moved** |
| Shutdown | SIGINT exits 0, no traceback, torque off, port released |

The clamp was proven without moving the arm by pinning the soft limits to the
arm's current pose, so the clamped goal *was* the present position. Jogging each
joint through a real range is still a human bench step (`BENCH.md` steps 5–6).
