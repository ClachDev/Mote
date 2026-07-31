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
the same SDK LeRobot and the stacks above sit on. Arm control has since folded
into `mote_hardware`'s ros2_control interface (below);
[`adityakamath/sts_hardware_interface`](https://github.com/adityakamath/sts_hardware_interface)
does the same job on the same SDK, with mixed position/velocity modes on one
bus, and remains the upstream to converge on — but it targets Kilted (we run
Jazzy), is not on rosdistro, and is a fast-moving single-maintainer WIP, so
adopting it now would trade a working stack for a moving one.

## Wiring: the arm shares the drive-wheel bus

Verified on the robot: arm servos are IDs **1–6**, the drive wheels are **7**
and **9**, and all eight are on the one `/dev/mote_servos` (CH343) bus. The arm
needs no udev rule of its own.

A serial port has no kernel-level exclusion, so a second opener is not refused —
it interleaves packets on the bus that *moves the robot*, and both openers see
corrupt replies. There is therefore exactly one process allowed to hold that
port, and **it is the controller_manager**: `mote_hardware`'s `MoteHardware`
exports velocity command interfaces for the wheels and position command
interfaces for the six arm joints, from one `open()`.

That is what lets the arm move *during* a mission, with Nav2 driving the wheels
at the same time — the point of the arm, and impossible while it lived in its
own process. Two guards keep the arrangement honest rather than merely
documented:

- **ID collision** — an arm ID equal to a wheel ID would send arm commands to a
  wheel. Rejected at load time in `mote_arm.config` *and* in `MoteHardware`, on
  both sides of the language boundary.
- **One opener only** — `MoteHardware::on_activate` scans `/proc` for another
  holder of the port and refuses to start, naming the PID; the read-only bench
  tools (`arm-check`, `arm-gains`) do the same through `mote_arm.bus`. So
  whichever starts first wins and the loser says why, instead of two processes
  quietly corrupting each other's traffic.

The bench tools still open the bus directly, so they still need the control
stack stopped (`pixi run kill`): `arm-check`, `arm-gains`, `arm-calibrate` and
`arm-offsets`. `jog` and `arm-pose` do not — they command the controller.

### Where the calibration enters

`zero`/`min`/`max` are measurements of one physical arm, so they live in
`$MOTE_HOME/arm.yaml` and robot.yaml carries only conservative placeholders
(see "`zero` and `home` are different things" below, and `arm-calibrate`).
The hardware enforces the soft limits, and the hardware reads them from the
URDF — so the URDF has to carry the *calibrated* numbers:

```
robot.yaml arm:  (defaults: ids, names, direction, gains)
        +
$MOTE_HOME/arm.yaml  (this arm's measured zero/min/max)
        |
        v  mote_arm.config.load() — the one implementation of the overlay
  launch_utils.resolved_arm()
        |
        v  written out, passed as xacro's `arm_config:=`
  <ros2_control> joint <param>s  ->  MoteHardware's clamp
```

xacro cannot resolve `$MOTE_HOME` or apply the overlay, so the launch does it
and hands over the answer. A bare `xacro mote.urdf.xacro` (for checking
generation) falls back to the packaged placeholders — never drive a calibrated
arm from that URDF, because calibration *moves the zero*, so the same radian
value names a different physical position.

## Where the arm lives in the control stack

```
robot.yaml (arm:)
   |
   +-- mote.urdf.xacro  -> <ros2_control> joints, one per arm servo, carrying
   |                       its id / soft limits / home / invert as <param>s
   |
   +-- mote_launch.py   -> arm_controller's joint list (never duplicated in
                           controllers.yaml)

controller_manager
   |-- diff_drive_controller   (active)    velocity -> wheels
   |-- joint_state_broadcaster (active)    /joint_states, wheels *and* arm
   \-- arm_controller          (INACTIVE)  position -> arm, on demand
```

`arm_controller` is a `JointTrajectoryController`, spawned **inactive**.
Activating it is what claims the arm's position command interfaces, which is
what makes the hardware enable torque — so "the arm is limp until something
asks it to move" is now a property of the control stack rather than a rule the
driver had to remember.

The hardware also spends the shared bus carefully, because the wheels are on
it: arm states are read one joint per control cycle (~8 Hz per joint at the
50 Hz update rate) rather than six reads every cycle, and arm goals go out as a
single sync-write packet, only when a goal actually changed. An idle arm costs
no bus traffic at all.

## Components

All bus I/O is isolated in `bus.py`; the config maths in `config.py` is
ROS-free and unit-tested (`test/`), so the safety-critical clamping and
conversions are verified without hardware.

| Piece | What it is |
|-------|------------|
| `config.py` | Parses the `arm:` section; encoder<->radian conversion + soft-limit clamping. |
| `bus.py` | `FeetechBus` — thin `scservo_sdk` wrapper (ping, read position/health, torque, position goal, homing offset). Lazy SDK import. |
| `control.py` | The one place that knows how to talk to `arm_controller`: single-point trajectories, and activation as the torque switch. |
| `cli.py` | The plumbing every arm CLI shares: strict argument parsing with ROS's own arguments cut out first, and a shutdown that stops spinning before it destroys the node. Both are properties that fail silently otherwise — see "Exits and arguments" below. |
| `arm_launch.py` (in `mote_bringup`) | Bench bring-up — the same controller_manager, URDF and `controllers.yaml` as a mission, without the lidar/camera/Nav2. `pixi run arm`. |
| `jog` (CLI) | Interactive per-joint jog. A *client of the controller* — publishes clamped trajectories, never opens the bus. `pixi run arm-jog`. |
| `arm_check` (tool) | Standalone enumeration + health + zero snapshot. Read-only, but opens the bus: run with the control stack stopped. `pixi run arm-check`. |
| `calibrate.py` / `arm_calibrate` | Two-phase range calibration: sweep every joint at once, centre its zero, save limits to `$MOTE_HOME/arm.yaml`. Owns the bus: control stack stopped. `pixi run arm-calibrate`. |
| `arm_offsets` (tool) | Read/back up/restore/set the servos' position-correction offsets. The recovery path if a calibration is interrupted. `pixi run arm-offsets`. |
| `poses.py` / `arm_pose` | Teach and replay named poses, and narrow limits to a working envelope. `pixi run arm-pose save\|list\|go\|limits\|delete`. |

## Exits and arguments

Two things every arm CLI needs, both of which fail *quietly* when hand-rolled,
so they live in `cli.py` and nowhere else.

**Destroying a node that `spin()` still holds aborts the process.** The
executor is pulled out from under itself and the interpreter calls
`std::terminate`: exit 134, `terminate called without an active exception`,
after the tool has already done its work. The fix is ordering — shut the
context down, *join the spin thread*, and only then destroy — which is what
`cli.shutdown(node, spinner)` is for. Measured on this arm's CLIs with no
hardware attached: `jog` (stdin closed) and `arm-pose list` each aborted 3 of 3
runs before, and exited 0 on 3 of 3 after. It is not a rare race — with no
stack running to talk to, it reproduced every time. `test_cli.py` watches a
child process's exit status, because nothing in-process can catch an abort.

**ROS's arguments arrive mixed in with the tool's own.** `ros2 run` hands the
executable the whole command line, so a plain `parse_args` rejects
`--ros-args` outright (`arm-pose list --ros-args -p use_sim_time:=true` used to
die with "unrecognized arguments") while the usual workaround,
`parse_known_args`, silently discards anything it does not recognise — which on
a *safety* flag means a mistyped `--max-travel` or `--speed` does nothing and
says nothing, and the arm moves under the default instead. `cli.parse` cuts the
`--ros-args ... --` block out first and then parses what is left strictly, so
ROS arguments pass through and a typo is still an error.

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
calibration. A pose is recorded in radians about its joint's `zero`, so moving a
zero changes which physical position each number names — but `arm-calibrate`
applies exactly the shift it computed, so poses survive a recalibration without
being re-taught. Editing a `zero` by hand does not, and invalidates them.

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

`arm-pose go` and `jog` command `arm_controller`, so they run happily alongside
a mission. `arm-check`, `arm-gains`, `arm-calibrate` and `arm-offsets` open the
bus directly and so still need the control stack stopped — `MoteHardware`'s own
guard will refuse to start against them, and theirs will refuse to start against
it.

Lag is measured against `/joint_states`, which the hardware refreshes one arm
joint per control cycle to stay inside the bus budget it shares with the wheels,
so a stall is caught within a few setpoints rather than instantly.

## Torque policy

Nothing moves without an explicit command. The policy did not change when the
arm moved into ros2_control — it stopped being a rule the driver enforced and
became a consequence of who holds the command interfaces.

- **Bringup:** torque **OFF** — the arm is limp/back-drivable, and
  `arm_controller` is spawned inactive so nothing holds its interfaces. Have the
  arm physically supported or resting in a stable pose before power. A servo
  that fails enumeration, or cannot be *confirmed* in position mode (mode
  changes are verified by read-back, never blind-written to EEPROM), is excluded
  from control entirely: its state is still published, but it accepts no goals —
  in wheel mode a position goal is obeyed as a speed.
- **Taking hold:** activating `arm_controller` makes the hardware seed each
  joint's goal register with its *present* position and only then enable torque
  — the order that stops an arm snapping to a stale goal. It engages one joint
  per control cycle (~120 ms for all six) so no single realtime cycle pays for
  six read-plus-write pairs, and a joint whose position cannot be read stays
  limp rather than being driven against an unknown goal.
- **Letting go:** deactivating `arm_controller` (`jog`'s `torque off`, or
  quitting `jog`) drops torque immediately, inside the switch itself rather than
  on the next write — a component being torn down may never write again.
- **Shutdown:** deactivating the hardware stops the wheels and limps the arm.

Goals are soft-clamped to the per-joint limits from `robot.yaml` **in the
hardware**, on the far side of every client, so a trajectory controller, the jog
CLI and the task layer are all held to the same envelope. `jog` clamps again
client-side purely for immediate feedback.

## Control interfaces

- `/joint_states` (`sensor_msgs/JointState`) — published by
  `joint_state_broadcaster` for the wheels *and* the arm. robot_state_publisher
  animates the arm in TF from these (the joints are in the URDF; see
  `mote_description/urdf/mote.urdf.xacro`, `arm:=true`).
- `arm_controller/joint_trajectory` (`trajectory_msgs/JointTrajectory`) — a
  goal. `allow_partial_joints_goal` is on, so a single-joint trajectory is
  valid and the rest hold where they are.
- `arm_controller/follow_joint_trajectory` (action) — the same thing with
  feedback and cancellation; this is the seam the task layer's pick/place will
  use in place of its stubs.
- `controller_manager/switch_controller` — activate to hold, deactivate to limp.

## Physical note (GitHub #2)

The camera does not fit when the SO-101 arm is attached. That is an unresolved
mechanical clash, tracked separately — not addressed here.

## Calibration

The committed `zero:` values are the arm's **as-found resting counts** read off
the robot, not measured mid-travel. A real calibration run showed why that is
bad: the parked pose sits within ~20 counts of a hard stop on `shoulder_lift`,
`elbow_flex` and `gripper`, so "0 rad" currently means "jammed against the
stop", and the committed bands (~0.2 rad) are a small fraction of the ~3.4–3.6
rad these joints actually travel.

See `BENCH.md` for the full runbook. In short:

1. `pixi run arm-check` — confirm every joint responds; note IDs.
2. `pixi run arm-calibrate` — sweep the joints, centre their zeros, save to
   `~/.mote/arm.yaml`. No rebuild: the file is read at load time, not compiled
   in. This sets `zero` *and* `min`/`max` together, which is the point: limits
   only mean something relative to the zero they were measured about.
3. Re-teach only the poses it reported as outside the new limits — the rest are
   migrated for you.
4. Jog each joint (`pixi run arm-jog`) and flip `invert` for any that moves
   opposite the expected sign. `invert` changes what the limits mean, so
   re-calibrate after changing it.

`arm-check -- --save-zero` still prints a bare `zero:` snapshot of the current
pose. It is a convenience, not calibration: it measures no range and writes no
offset, so the limits stay whatever they were.

## Verified on hardware

> **The ros2_control fold is not in this table.** Everything below was measured
> against the real arm on 2026-07-25, when it ran as a standalone driver. The
> move into `mote_hardware` is verified against a *simulated* STS bus —
> `mote_hardware/test/test_arm_bus.cpp` drives the real plugin through the real
> SCServo SDK over a pty and asserts on the actual packets (comes up limp, seeds
> the goal before enabling torque, clamps to the soft limits, sends nothing when
> a goal has not changed, drops torque on release) — plus a green `pixi run
> sim-test`. The hardware runs in `BENCH.md`'s "still open" list are the real
> gate, in particular jogging the arm *while the wheels are driving*, which is
> the thing this whole change exists to make possible.
>
> The soft limits did not change, and they are still the authoritative gate on
> motion — but they are still the conservative envelope of two taught poses,
> and `home:` is still the arm's as-found resting counts rather than a taught
> mechanical zero (see Calibration). Nothing here widens what the arm may do.

Run against the real arm on 2026-07-25:

| Check | Result |
|-------|--------|
| Bus enumeration | all 6 joints respond; 5.1–5.2 V, 26–29 °C, load 0 |
| `/joint_states` | 6 arm joints at a steady 20.0 Hz, no jitter while limp |
| Startup torque | bringup comes up limp; `TORQUE_ENABLE` reads 0 on every joint |
| Port guard | `arm_check` refused the bus while another process held it, naming its PID |
| Torque engage | seeding goals before enabling moved the arm **0.00000 rad** |
| Jog motion | `elbow_flex` jogged −0.05 rad per step and returned; `/joint_states` tracked |
| Soft-limit clamp | repeated `+` past the limit held at `+0.103` — no further motion |
| Shutdown | SIGINT exits 0, no traceback, torque off, port released |
| Pose replay | full `home` <-> `reachy` move (3.19 rad / 183 deg) completed both ways, streamed at 0.5 rad/s, lag steady 0.07-0.10 rad, settling within 0.026-0.041 rad |
| Servo gains | `arm-gains apply` wrote and verified Kp=32 on all six servos; temps unchanged at 27-30 C after the full move |

Gain tuning, on the same arm on 2026-07-28:

| Check | Result |
|-------|--------|
| Kp sweep (16/32/64/128) | droop confirmed across an 8x gain range; error 0.068 -> 0.008 rad at load 144-188 of 1000, no ripple or reversals at any gain |
| Repeatability | a second sweep reproduced every trial to 1-2 encoder counts |
| Larger step (1.0 rad) | same law, error 0.038 -> 0.004 rad; zero overshoot (the servo's speed profile decelerates into the goal) |
| Double speed (1000 steps/s) | still no overshoot, ripple or reversals, up to Kp=128 |
| Ki sweep (0/1/2/4/8 at Kp=64) | ki=8 reached 99.7% (error 0.001 rad) but settling went 0.46 s -> 2.12 s; left at 0 |
| Kp=64 applied | written and read-back verified on all six servos |
| Pose replay at Kp=64 | full `home` <-> `reachy` both ways, lag 0.05-0.08 rad, settling within 0.012-0.028 rad (was 0.026-0.041 at Kp=32); servo 22-24 C throughout |

`arm-pose go` streams setpoints continuously (see above) and stops when the arm stops keeping up, rather than holding against a load it
cannot overcome. At the shipped `Kp = 16` that guard correctly halted the
`reachy` replay after ~0.13 rad; with `Kp = 32` applied the same move runs to
completion.

## Position accuracy and the servo gains

The arm shipped with `Kp = 16` on every servo, which left a permanent
steady-state error under load: the servo settles where `Kp x error` balances the
holding torque, and `Ki = 0` never integrates that droop away. `arm-gains sweep`
(below) measured it on `elbow_flex`, stepped -0.200 rad from rest:

| Kp | steady error | reached | load (of 1000) | Kp x error | settle | ripple |
|----|--------------|---------|----------------|------------|--------|--------|
| 16 (as shipped) | 0.068 rad | 66.0% | 188 | 1.09 | 0.58 s | 0 |
| 32 | 0.031 rad | 84.4% | 168 | 1.00 | 0.50 s | 0 |
| **64 (applied)** | **0.014 rad** | **92.8%** | **144** | **0.92** | **0.46 s** | **0** |
| 128 | 0.008 rad | 95.9% | 144 | 1.06 | 0.46 s | 0 |

That is droop, **not** torque saturation: error falls 8.2x for an 8x gain rise
while load stays at 144-188, nowhere near the 1000 that saturation would pin it
to, and `Kp x error` holds within 1.18x. The servo was using about a fifth of
the effort available to it, so the 5.1-5.2 V supply was never the binding
constraint. A repeat run agreed to 1-2 encoder counts, and the same law holds on
a 1.0 rad step (0.038 -> 0.004 rad over the same gains) and at double speed.

**`Kp = 64` is applied** to all six servos and recorded in `robot.yaml`. It is
not the best-scoring gain — 128 measured better on every column, with no ripple,
no reversals and no overshoot at either step size or speed. It is chosen anyway,
because what the sweep tests is a *static hold on an unloaded arm*: a stiffer
loop reacts harder to a payload, a collision or hand-guiding, and none of those
are measured. 64 takes half the remaining error and keeps a measured 2x margin
below the highest gain that behaved. Revisit it with a payload on the gripper,
not from this table.

`Ki` stays 0. Swept at `Kp = 64`, integral action does close the gap —
`ki = 8` reached 99.7% of the step, error 0.001 rad — but settling stretched
from 0.46 s to 2.12 s as the integrator wound in, and `arm-pose` streams a fresh
setpoint every 50 ms, so nothing ever waits for that. It also stores the effort
used to hold a load, which is the lunge risk on unloading that made this a
deliberate test rather than a default.

With `Kp = 64` the full 3.19 rad (183 deg) `home` <-> `reachy` move completes in
both directions, lag steady at 0.05-0.08 rad, settling within **0.012-0.028 rad**
(0.7-1.6 deg) — against 0.026-0.041 rad at `Kp = 32`.

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

### Measuring a gain instead of guessing it: `arm-gains sweep`

A gain is only defensible against a measurement, so the third subcommand takes
one and produces the evidence:

```
pixi run arm-gains sweep --joint elbow_flex --kp 16,32,64,128
pixi run arm-gains sweep --joint elbow_flex --kp 32 --ki 0,1,2   # the Ki question
```

It drives that one joint through the **same** step (`--step`, default -0.2 rad
from wherever the joint is resting) under each candidate gain set, sampling
position and load at 50 Hz, and scores each trial (`step_response.py`):

| Column | What it decides |
|--------|-----------------|
| `error` | how far short the joint settles — the droop itself |
| `kp*err` | constant across gains = proportional droop; not constant = something else |
| `load` | effort while holding, of 1000; near 1000 means torque saturation, not droop |
| `settle` | time to enter and stay in the settle band (2% of travel, min 2 counts) |
| `ripple`/`rev` | peak-to-peak motion and direction changes *while holding* — the counter-check on raising gain, since a hunting servo buzzes rather than holds |

It closes with a one-line verdict reading the sweep as droop or as saturation,
and writes every sample to `~/.mote/arm_gain_sweeps/<stamp>.json` so a run can be
re-read or plotted later rather than believed from a terminal.

Each trial writes its gains with torque **off** and then re-enables against the
joint's present position. Gains are EEPROM registers, and a servo that latched
them at torque-enable would run every trial at the same gain and report a droop
that mysteriously ignores `kp` — the sweep exists to stop us assuming that away.
The consequence is that the joint goes briefly limp between trials, so the arm
must be resting in a pose it holds unsupported (the same condition `arm_driver`
starts in).

The sweep moves the arm and writes EEPROM, so it is a bench tool with the
guards to match: it torques **only** the swept joint and leaves the rest limp,
refuses a step that would clamp against the soft limits (trials that command
different travel are not comparable), stops if the servo reaches `--max-temp`
(55 C default), and — via a `finally`, so a crash or Ctrl-C counts too — returns
the joint to where it started, writes the original gains back, and drops torque.
`test_gain_sweep.py` pins those properties against a simulated droopy servo, so
they are checked without hardware.

Closing the residual 1-3.5 deg would mean a small `Ki`, which is left alone for
now: integral windup on an arm risks a lunge when a load is removed, so it wants
a deliberate test on an unloaded joint first — that is what the `--ki` sweep
above is for. Supply voltage is worth revisiting only after that, since the arm
still has not demanded full torque.
