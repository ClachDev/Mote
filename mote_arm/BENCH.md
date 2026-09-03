# SO-101 arm bench validation

A short, scripted first-light session. **A human at the bench is required.**
Keep the arm clear of obstructions and be ready to cut power. The arm starts
limp, so support it or rest it in a stable pose before each step.

Prerequisites: servos wired and enumerated, robot powered, repo built
(`pixi run build`).

---

**Steps 0, 2 and 4–7 have been run against the robot** (2026-07-25); results
are in `README.md`. **Step 3 was run on 2026-07-28**: all six joints swept and
centred, offsets written and confirmed, `~/.mote/arm.yaml` saved. Measured
travel was 4.17 / 3.65 / 3.42 / 3.61 / 5.89 / 2.29 rad — so the packaged
`robot.yaml` bands, still the old pose-envelope output, understate the real
range by an order of magnitude on some joints. They stay as the conservative
default for an arm that has never been calibrated; calibration is per-robot and
does not touch the repo.

These steps are kept because they are the right checks after any rewiring or
recalibration. What is still open:

- **Does the homing offset apply to commanded goals, or only to feedback?**
  The read side is proven — positions moved by exactly the predicted delta on
  every joint, which is what "written and confirmed" checks. The write side
  shows up on the first `arm-jog` move after calibrating: if a commanded angle
  lands roughly one offset away from where you asked, `config.rad_to_counts`
  has to compensate. Try this first.
- **Does `wrist_roll` have real stops?** It swept 5.89 rad, 94% of a turn, and
  LeRobot treats the SO-101's as a full-turn motor. If it in fact spins freely,
  its limits are just wherever the sweep stopped turning.
- **Steps 5–6 for the other five joints** — only `elbow_flex` was jogged and
  clamp-tested, and that was against the old envelope.
- Nothing power-related: the earlier "5 V torque limit" was a misdiagnosis. It
  was proportional droop from `Kp=16`; `Kp=64` is now applied (chosen from a
  sweep, see README) and the arm completes the full home<->reachy move with
  0.012-0.028 rad residual. `Ki` was tested and left at 0.

## Step 0 — wiring (settled)

The arm **shares the drive-wheel bus**: arm IDs 1–6, wheels 7 and 9, all on
`/dev/mote_servos`. `robot.yaml` already reflects this and no udev rule is
needed. One process owns that port and it is the controller_manager, so the arm
now comes up with the base rather than instead of it.

Only one thing on this bench still needs the base stopped: the tools that open
the bus directly — `arm-check` and `arm-gains`. Run `pixi run kill` before
those; they refuse to start otherwise, naming the process that holds the port.
`arm-jog` and `arm-pose` command the controller and need no such care.

## Step 2 — enumerate + health check

```
pixi run arm-check
```

**Expected:** a table with all six joints — `shoulder_pan`, `shoulder_lift`,
`elbow_flex`, `wrist_flex`, `wrist_roll`, `gripper` — each showing a raw
position (0–4095), the supply voltage (measures 5.1–5.2 V today), a temperature
(< 55 °C), and a load near 0 while limp. Any `NO RESPONSE` row means
a wiring/ID problem — fix IDs with `pixi run setup-ids` / `ros2 run
mote_hardware servo_debug` before continuing.

## Step 3 — full-range calibration — needs a human, ~10 minutes

**This is where the soft limits come from.** The committed values are still the
old pose-envelope output, which never learned where the mechanical stops are.
Run one full pass and replace them.

Support the arm — it goes limp at the start of this step, and an unsupported arm
falls. Stop the driver and the robot base first (`pixi run kill`): this tool
opens the serial bus directly.

```
pixi run arm-calibrate
```

### Phase 1 — record the ranges

Move each joint **gently** to both of its mechanical stops — the stop is where it
resists, do not force it, and do not use it to "find" extra range. Take the
joints in any order; all six are recorded at once in a live table:

```
  joint              now    low   high        swept
  shoulder_pan      2019    682   3403     4.17 rad
  ...
```

`low` and `high` are the two ends of *travel*. A joint whose travel crosses the
encoder wrap reads `low` numerically larger than `high`.

Press **Enter** once every joint has been to both stops.

**Expected:** a swept range per joint matching what you felt — the big joints
measured 3.4–4.1 rad, the gripper ~2.3. The range is the number to watch: it
grows only when you reach further than before, so it stops growing once you have
both stops. Anything that cannot be calibrated says why and keeps its
existing values; see the failure table below.

### Phase 2 — centre the zeros (writes servo EEPROM)

Automatic: each joint's 0 rad moves to the *measured* middle of the range you
just swept. **Leave the arm wherever the sweep ended** — this changes only what
the encoders report, not where the arm is. It shows what it intends to write and
asks before touching EEPROM:

```
joint            mid-travel  offset  ->    new
shoulder_pan           2042    -997  ->  -1003
...
Writes servo EEPROM on 6 joint(s) — a persistent change.
write? [y/N]
```

**Expected:** `backed up to ~/.mote/arm_offsets_backup.yaml`, then one line —
`6 joint(s) centred and confirmed`. *Confirmed* is the real check: each servo is
written, read back, and checked to have actually moved its reported position by
the offset delta, which is what proves the servo *acts* on the register the way
we assume and would catch a wrong sign encoding. Success is a count rather than
a list because a servo that fails stops the run by name, below.

**If it stops partway** it names the servos already changed and points at
`pixi run arm-offsets restore`, which puts them back from the snapshot taken
before the first write. Do that before re-running.

If a stop reports a position that looks nothing like the expected one, check
whether the number it read equals the offset just written in sign-magnitude
form (`abs(offset) | 0x800` for a negative one). That means the read picked up
the previous register's reply rather than the position — the same
read-races-the-EEPROM-write hazard documented for `arm-gains`. Reads now clear
the input buffer first and the post-write check requires two agreeing reads, so
this should not recur; if it does, the settle delay needs raising further.

This is the step that stops a joint's travel straddling the encoder wrap. On
this arm `shoulder_pan` and `wrist_roll` both did.

### Then, in this order

It then saves the limits to `~/.mote/arm.yaml` — this robot's own calibration,
not the repo — and says so in one line. The numbers are not reprinted: the swept
ranges were on screen a moment ago, the limits are those pulled inward by
`--margin`, and the file keeps each value next to the measurement it came from.

**If the save fails** — validation rejects the document, or the file cannot be
written — the servos have already been centred, so the arm is calibrated and the
file is not. It says so and names `pixi run arm-offsets restore`. Do one or the
other before `pixi run arm`: until then the soft limits describe a frame the
servos have stopped using.

Taught poses are re-expressed about the new zeros automatically and keep
pointing where they did; the old file is kept as `.bak`. Any pose that lands
outside the new limits is named — that one was taught somewhere the arm cannot
now reach and needs a decision.

```
pixi run arm-check          # rad column reads ~0.000 at the centred pose
```

Nothing in the repo changes — the calibration is per-robot state under
`~/.mote/`. Note `~/.mote/arm.yaml` (this, the arm calibration) is not
`~/.mote/robot.yaml` (fleet identity), and neither is
`mote_description/config/robot.yaml` (the shared hardware description).

`~/.mote/arm.yaml` keeps the measurement next to each value — swept range,
samples, margin, and the homing offset, which is the only record of what was
written to the servo.

### When a joint does not calibrate

| Reported | What to do |
|----------|------------|
| `travel exceeds one revolution; joint is continuous` | The joint spins freely and has no stops to calibrate against. Exclude it: `--joints` the others. |
| `sweep crossed the encoder 0/4095 boundary` | Only under `--skip-homing`, which does not move the zero. Run without it. |
| `zero too close to a stop; zero would be unreachable` | Only under `--skip-homing`. Run without it and the zero is centred by construction. |
| `swept only X rad, too short for the margin` | The sweep did not reach both stops — redo it. If the joint really is that short, lower `--margin`. |


A joint that fails keeps its previous values, with the reason as a comment above
its line, so pasting the block never silently reverts a joint to a guess. A joint
whose sweep is unusable also does not get its zero moved — the usable set is
decided before any EEPROM is touched.

### The goal-range fence

Phase 2 shows registers 9 and 11 on every joint — the band of goal positions
the servo will accept — under the same confirmation as the zeros, and then
rewrites both. A fence binds only under torque, so the sweep you just did went
straight through it: a fence left behind describes travel the arm will refuse to
make, silently, stopping at the same angle every time as if it had run out of
torque. This arm spent four months in exactly that state.

Each joint's fence is written immediately after its own zero, so no joint is
ever left without one. The new band is wider than the soft limits in `arm.yaml`
by `--margin` at each end, so the soft limits always stop the arm first and the
fence only acts if they have gone wrong. `--skip-homing` writes nothing to the
servos, so it reports a cutting fence rather than correcting it.

The as-found bands are snapshotted to `~/.mote/arm_limits_backup.yaml` before
the first write, so:

```
pixi run arm-limits show      # read-only: the band, in counts and radians
pixi run arm-limits clear     # hand the whole range back, outside a calibration
pixi run arm-limits restore   # put the as-found bands back
```

### The offsets themselves

```
pixi run arm-offsets show      # read-only: raw register, decoded value, position
pixi run arm-offsets backup    # snapshot before doing anything risky
pixi run arm-offsets restore   # put the snapshot back
pixi run arm-offsets set --joint shoulder_pan --value=2027
```

`show` prints the raw register next to the decoded value deliberately: the
decode assumes bit 11 is a sign bit, and if those two look unrelated for a
servo, that assumption is the thing to doubt. Note these servos may arrive with
non-zero offsets already set — this arm did (2027, -1723, 1772, -1706, -40,
1317), stable across runs, which is why the existing value is always read and
folded in rather than assumed to be zero.

`--skip-homing` records ranges against the zeros already in `robot.yaml` and
writes nothing to the servos — for re-measuring after a calibrated arm has been
disturbed, not for a first pass.

## Step 4 — joint states live in ROS

Terminal A:

```
pixi run arm
```

**Expected:** `MoteHardware ... Activated on /dev/mote_servos ... arm: 6/6
joints controllable (torque OFF — limp until a controller claims them)`, then
`joint_state_broadcaster` and an **inactive** `arm_controller`. Confirm with:

```
pixi run -- ros2 control list_controllers
```

`arm_controller` must read `inactive` — that is what keeps the arm limp.

Terminal B:

```
pixi run -- ros2 topic echo /joint_states
```

**Expected:** messages naming the six arm joints with positions in rad,
updating as you move the (still limp) arm by hand. **This is acceptance
criterion 1.**

## Step 5 — jog each joint through a small range (done for `elbow_flex`)

Leave `pixi run arm` running in Terminal A (or a full `pixi run robot` — the
arm is part of the mission stack now, and this step works during a mission).
Terminal C:

```
pixi run arm-jog
```

For **each** joint in turn (arm supported, ready to cut power):

1. Select it by number (e.g. `0` for `shoulder_pan`). The status line shows its
   measured position and soft limits.
2. `step 0.05` to set a small increment.
3. Jog `+` a few times, then `-` back — watch the joint move a small amount in
   the commanded direction, and `/joint_states` (Terminal B) track it.
   - If the joint moves the **wrong way**, set `invert: true` for it in
     `robot.yaml`, rebuild, and repeat.
4. `zero` returns it to 0 rad (mid-travel, not the rest pose).

Only `elbow_flex` has been done this way; the remaining five are open. See
README's "Verified on hardware" for what the elbow run showed.

**Expected observations per joint:**

| Observation | Meaning |
|-------------|---------|
| Joint moves smoothly a small amount per `+`/`-` | position control + scaling OK |
| `/joint_states` value follows the jog | feedback + conversion OK |
| Direction matches the `+` sign | `invert` correct |

## Step 5b — teach poses, and optionally narrow the envelope

Poses are how you return the arm to somewhere useful; the hard limits already
came from Step 3.

1. `pixi run arm` in one terminal.
2. Pose the limp arm by hand somewhere useful, then
   `pixi run arm-pose save <name>` (read-only capture).
3. Repeat for each pose worth reaching.
4. `pixi run arm-pose go <name>` moves between taught poses. It prints per-joint
   travel and asks before moving. Expect a large travel on the first `go` after
   a `save`: the arm is limp until a controller claims it, so it falls to rest
   the moment you let go of it.

`pixi run arm-pose limits` prints a `joints:` block spanning every taught pose
plus a margin. It is **not** the calibration path — it widens outward from poses
already reached and never learns where the stops are, which is why joints that
barely differ between two poses come out with a near-zero band. Use it only to
deliberately **narrow** a joint inside its calibrated stops, and never paste it
over calibrated limits expecting to widen them: re-run Step 3 for that.

## Step 5c — measure the position-loop gains

Gains live in servo EEPROM, so they are hardware config, not software config:
`robot.yaml`'s `arm.gains` records them and `arm-gains` reconciles the two.
Choose them from a measurement, not from a datasheet default.

Stop the driver first (`arm-gains` opens the bus itself) and clear the joint's
path — this step moves the arm. Park the arm in a pose it holds unsupported:
each trial drops torque briefly to write the gains, so a raised pose would sag.

1. `pixi run arm-gains show` — what the servos actually hold right now.
2. `pixi run arm-gains sweep --joint elbow_flex --kp 16,32,64,128` — steps the
   joint -0.2 rad under each gain and prints error, load, settling, ripple and
   reversals per trial. **Expected:** error falls as `kp` rises while `kp*err`
   stays roughly constant and load stays far below 1000 (proportional droop);
   the verdict line says so. Watch and listen at the top of the range — ripple
   over a few counts, rising `rev`, or an audible buzz is the joint hunting, and
   that gain is too high whatever its error says.
3. Optional, for the residual droop:
   `pixi run arm-gains sweep --joint elbow_flex --kp <chosen> --ki 0,1,2`.
   Do it with the joint **unloaded** first: integral action stores the effort it
   needed to hold a load, so removing that load can produce a lunge.
4. Put the winner in `robot.yaml`'s `arm.gains`, rebuild, then
   `pixi run arm-gains apply` to write it to all six servos.

The sweep restores the gains it started with and leaves the joint limp, so a run
on its own changes nothing — step 4 is what makes a choice stick. Each run writes
its full trace to `~/.mote/arm_gain_sweeps/<stamp>.json`.

`elbow_flex` is the joint to use: it is the only one whose committed soft limits
leave room for a 0.2 rad step, and it carries the forearm's weight, so there is
a real load to droop under.

## Step 6 — demonstrate the soft-limit clamp (done for `elbow_flex`)

With a joint selected, jog `+` repeatedly toward its upper limit. **Expected:**
motion stops at the configured `max` and the driver logs
`... goal X clamped to Y rad`; it never drives past the soft limit no matter how
many times you press `+`. Repeat toward the lower limit. **This is acceptance
criterion 2.**

## Step 7 — torque-off on exit, and a clean exit

In `arm-jog`, type `quit`. **Expected:** `limping arm (deactivating
arm_controller) and exiting...`; the arm goes back-drivable immediately. Stop
`arm` (Ctrl-C) and confirm it also logs a clean shutdown and leaves the arm
limp. **Nothing should move on startup or shutdown.**

Then check the exit *status*, not just the message — the arm is already limp by
the time the process falls over, so an abort here is invisible unless looked
for:

```
pixi run arm-jog        # 'quit' at the prompt
echo $?                 # expect 0
pixi run arm-pose list
echo $?                 # expect 0
```

**Expected:** `0` from both, and no `terminate called without an active
exception` on stderr. A `134` is the destroy-while-spinning abort (see README,
"Exits and arguments"); it means the tool did its job and then crashed on the
way out.

## Step 8 — virtual-leader teleop, recording and replay

The teleop path has its own guided session, because it needs three terminals
and because three of its checks are observations no script can make (the arm
stopping at a limit, halting on a released key, going limp on panic).

**Rehearse it headless first** — the same loop runs against the mock follower
with no hardware at all, and a failure there is a software bug, not a bench one:

```
pixi run arm-teleop-test
```

Then, on the arm:

```
# terminal A
pixi run arm mirror:=true
# terminal B
pixi run arm-teleop
# terminal C
pixi run arm-bench-teleop
```

Terminal C walks through the safety demonstrations, records an episode while
you teleop it, checks the capture holds a real motion, prints the off-board
export/inspect commands, and replays the episode at quarter speed. It writes
`$MOTE_HOME/episodes/bench/bench-report.txt` — nothing is recorded as passing
that you did not say you saw.

Full workflow and design: [TELEOP.md](TELEOP.md).

---

## Sign-off checklist

Already verified on the robot (2026-07-25):

- [x] `arm.port` points at the shared wheel bus; all six joints respond
- [x] `/joint_states` shows live arm joints (20 Hz)
- [x] bringup starts limp and leaves the arm limp on shutdown
- [x] the soft-limit clamp rejects an out-of-range goal (proven zero-motion)

- [x] `elbow_flex` jogs in the commanded direction; `/joint_states` tracks it
- [x] soft limits clamp during a real jog (repeated `+` held at the limit)
- [x] enabling torque holds the current pose instead of snapping
- [x] `min`/`max` in `robot.yaml` derived from taught poses, not guessed
      (superseded by the calibration pass below — provisional until it runs)
- [x] servo gains applied and verified (`pixi run arm-gains`), full
      home<->reachy move completed both ways
- [x] gains chosen from a sweep, not a default: Kp=64 applied to all six,
      residual on the full move now 0.012-0.028 rad (2026-07-28)

- [x] **Step 3: one full `pixi run arm-calibrate` pass on the real arm**
      (2026-07-28), saved to `~/.mote/arm.yaml`; taught poses migrated
      automatically rather than re-taught
- [x] every servo's homing offset written and confirmed, and no joint reports an
      encoder wrap afterwards (two did before centring existed)
- [x] `Ki` tested and rejected for now (step 5c's `--ki` sweep: ki=8 closes the
      error to 0.001 rad but quadruples settling time)

Still open:

- [ ] `shoulder_pan` can be commanded to 0 rad (its packaged band still excludes
      its own zero; the calibrated band on the arm does not)
- [ ] the offset applies to commanded goals, not only to feedback — the first
      `arm-jog` move settles it
- [ ] **the arm moving under ros2_control on the real robot** — everything
      about the fold is verified against a simulated bus
      (`mote_hardware/test/test_arm_bus.cpp`), not against servos: confirm
      `arm-jog` still moves `elbow_flex` in the commanded direction, that the
      soft limits still hold, and that activating `arm_controller` takes hold
      without a snap
- [ ] **the arm moving while the wheels are driving** — the point of the fold.
      `pixi run robot`, drive a short goal, and jog the arm at the same time;
      watch for wheel-odometry glitches that would mean the bus is oversubscribed
- [ ] step 8: teleop, record, export/inspect and replay on the arm
      (`pixi run arm-bench-teleop`) — verified headless against the mock
      control stack, but not yet on hardware
- [ ] the other five joints jogged and direction-checked (`invert`)
- [ ] re-check the gain with a payload on the gripper — the sweep only measures
      an unloaded static hold, which is why Kp=64 was taken over a better-scoring
      128
