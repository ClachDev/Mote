# SO-101 arm bench validation

A short, scripted first-light session. **A human at the bench is required.**
Keep the arm clear of obstructions and be ready to cut power. The arm starts
limp, so support it or rest it in a stable pose before each step.

Prerequisites: servos wired and enumerated, robot powered, repo built
(`pixi run build`).

---

**Steps 0, 2 and 4–7 have been run against the robot** (2026-07-25); results
are in `README.md`. They are kept here because they are the right checks after
any rewiring or recalibration. What is still open:

- **Step 3 — one full calibration pass.** This is the important one. The
  committed limits are still the old pose-envelope output: they describe where
  the arm has been, not where it can go, and `shoulder_pan`'s band excludes its
  own zero so that joint can never be commanded to 0 rad. `pixi run arm-calibrate`
  measures the stops directly; until it has been run on the real arm, every
  limit below is provisional.
- **Steps 5–6 for the other five joints** — only `elbow_flex` was jogged and
  clamp-tested. The rest stay on a tight provisional envelope until Step 3 runs.
- Nothing power-related: the earlier "5 V torque limit" was a misdiagnosis. It
  was proportional droop from `Kp=16`; `Kp=32` is now applied (see README) and
  the arm completes the full home<->reachy move. A small `Ki` would close the
  remaining 1-3.5 deg, but wants a deliberate windup test first.

## Step 0 — wiring (settled)

The arm **shares the drive-wheel bus**: arm IDs 1–6, wheels 7 and 9, all on
`/dev/mote_servos`. `robot.yaml` already reflects this and no udev rule is
needed. Because the bus is shared, **stop the robot base before running the
arm** (`pixi run kill`) — the driver refuses to start otherwise, naming the
process that holds the port.

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
  joint              min   now   max      span
  shoulder_pan       698  2103  3378    4.11 rad
  ...
```

Press **Enter** once every joint has been to both stops.

**Expected:** a span per joint matching what you felt (the big joints measured
3.4–4.1 rad; the gripper ~2.3). A `spans 0/4095` note is fine and expected on
any joint whose travel crosses the encoder boundary — phase 2 is about to fix
exactly that, and the raw min/max are blanked for those joints because the
encoder numbers (17, 4093) describe the encoder rather than the travel. Watch
the span column, which is correct either way. Anything that cannot be calibrated says why and keeps its
existing values; see the failure table below.

### Phase 2 — centre the zeros (writes servo EEPROM)

Automatic: each joint's 0 rad moves to the *measured* middle of the range you
just swept. **Leave the arm wherever the sweep ended** — this changes only what
the encoders report, not where the arm is. It shows what it intends to write and
asks before touching EEPROM:

```
joint            mid-travel  offset  ->    new
shoulder_pan           3522    1009  ->  -1613
...
write homing offsets? [y/N]
```

**Expected:** `previous offsets backed up to ~/.mote/arm_offsets_backup.yaml`,
then every servo `written, verified, and reading confirmed`. That last phrase is
the real check — it confirms the servo *acts* on the register the way we assume,
and would catch a wrong sign encoding.

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

It then shows a diff of `mote_description/config/robot.yaml` and asks before
writing the new limits in — only between the `# BEGIN arm.joints` / `# END
arm.joints` markers, leaving the rest of the file untouched. Say yes.

It also lists every taught pose the changed zeros invalidate. **Re-teach those
last**, after the file is written, or they record against a zero that is about
to change:

```
pixi run arm-check          # rad column reads ~0.000 at the centred pose
pixi run arm                # in another terminal
pixi run arm-pose save home
git diff mote_description/config/robot.yaml   # review before committing
```

`--print-only` prints the block instead of writing, if you would rather paste it
yourself.

What was measured is kept in `~/.mote/arm_calibration.yaml`, including the
homing offsets — the only record of them outside servo EEPROM.

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

**Expected:** `arm_driver up ... (torque OFF — limp until commanded)`.

Terminal B:

```
pixi run -- ros2 topic echo /joint_states
```

**Expected:** messages naming the six arm joints with positions in rad,
updating as you move the (still limp) arm by hand. **This is acceptance
criterion 1.**

## Step 5 — jog each joint through a small range (done for `elbow_flex`)

Leave `pixi run arm` running in Terminal A. Terminal C:

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
   travel and asks before moving; it refuses any move over `--max-travel`
   (0.35 rad default).

`pixi run arm-pose limits` prints a `joints:` block spanning every taught pose
plus a margin. It is **not** the calibration path — it widens outward from poses
already reached and never learns where the stops are, which is why joints that
barely differ between two poses come out with a near-zero band. Use it only to
deliberately **narrow** a joint inside its calibrated stops, and never paste it
over calibrated limits expecting to widen them: re-run Step 3 for that.

## Step 6 — demonstrate the soft-limit clamp (done for `elbow_flex`)

With a joint selected, jog `+` repeatedly toward its upper limit. **Expected:**
motion stops at the configured `max` and the driver logs
`... goal X clamped to Y rad`; it never drives past the soft limit no matter how
many times you press `+`. Repeat toward the lower limit. **This is acceptance
criterion 2.**

## Step 7 — torque-off on exit

In `arm-jog`, type `quit`. **Expected:** `limping arm (torque off) and
exiting...`; the arm goes back-drivable immediately. Stop `arm` (Ctrl-C) and
confirm it also logs a clean shutdown and leaves the arm limp. **Nothing should
move on startup or shutdown.**

---

## Sign-off checklist

Already verified on the robot (2026-07-25):

- [x] `arm.port` points at the shared wheel bus; all six joints respond
- [x] `/joint_states` shows live arm joints (20 Hz)
- [x] driver starts limp and leaves the arm limp on shutdown
- [x] the soft-limit clamp rejects an out-of-range goal (proven zero-motion)

- [x] `elbow_flex` jogs in the commanded direction; `/joint_states` tracks it
- [x] soft limits clamp during a real jog (repeated `+` held at the limit)
- [x] enabling torque holds the current pose instead of snapping
- [x] `min`/`max` in `robot.yaml` derived from taught poses, not guessed
      (superseded by the calibration pass below — provisional until it runs)
- [x] servo gains applied and verified (`pixi run arm-gains`), full
      home<->reachy move completed both ways

Still open:

- [ ] **Step 3: one full `pixi run arm-calibrate` pass on the real arm**, its
      block pasted into `robot.yaml`, and any taught poses it named re-taught
- [ ] phase 1 wrote and verified a homing offset on every servo, and no joint
      reports an encoder wrap in phase 2 (two did before phase 1 existed)
- [ ] the poses `arm-calibrate` named were re-taught AFTER the rebuild
- [ ] `shoulder_pan` can be commanded to 0 rad afterwards (today its band
      excludes its own zero)
- [ ] the other five joints jogged and direction-checked (`invert`)
- [ ] optional: small `Ki` to remove the residual 1-3.5 deg droop (test windup
      on an unloaded joint first)
