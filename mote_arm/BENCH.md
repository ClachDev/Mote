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

- **Step 3** — the taught poses ("home", "reachy") define the working envelope,
  but `home:` in robot.yaml is still the as-found parked count rather than a
  deliberately taught mechanical zero. Optional; do it if you want "0 rad" to
  mean something specific.
- **Steps 5–6 for the other five joints** — only `elbow_flex` was jogged and
  clamp-tested. The rest have a deliberately tight envelope until you pose the
  arm somewhere that widens it.
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

## Step 3 — teach home offsets (calibration) — optional, needs a human

The committed `home:` values are the arm's as-found resting counts, so "0 rad"
currently means "the pose it was parked in". Replace them with true mechanical
zeros: move each joint by hand (it's limp) to its zero / neutral pose, then:

```
pixi run arm-check -- --save-home
```

Paste the printed per-joint `home:` counts into the `arm.joints` entries in
`robot.yaml`. Re-run Step 2 and confirm each joint's `rad` column now reads
~0.000 at the neutral pose. `pixi run build` to reinstall the config.

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
4. `home` returns it to 0 rad.

Only `elbow_flex` has been done this way; the remaining five are open. See
README's "Verified on hardware" for what the elbow run showed.

**Expected observations per joint:**

| Observation | Meaning |
|-------------|---------|
| Joint moves smoothly a small amount per `+`/`-` | position control + scaling OK |
| `/joint_states` value follows the jog | feedback + conversion OK |
| Direction matches the `+` sign | `invert` correct |

## Step 5b — teach the working envelope

Soft limits come from poses you vet physically, not from guesses:

1. `pixi run arm` in one terminal.
2. Pose the limp arm by hand somewhere useful, then
   `pixi run arm-pose save <name>` (read-only capture).
3. Repeat for each pose worth reaching.
4. `pixi run arm-pose limits` prints a `robot.yaml` `joints:` block spanning
   every taught pose plus a 0.10 rad margin, and sanity-checks that each taught
   pose falls inside it. Paste it into `robot.yaml` and rebuild.
5. `pixi run arm-pose go <name>` moves between taught poses. It prints per-joint
   travel and asks before moving; it refuses any move over `--max-travel`
   (0.35 rad default).

The committed limits came from two poses, `home` and `reachy`. Joints that
barely differ between them have a tight band by construction — teach a pose that
exercises them to widen it.

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
- [x] servo gains applied and verified (`pixi run arm-gains`), full
      home<->reachy move completed both ways
- [x] gains chosen from a sweep, not a default: Kp=64 applied to all six,
      residual on the full move now 0.012-0.028 rad (2026-07-28)

Still open:

- [ ] the other five joints jogged and direction-checked (`invert`)
- [ ] `home:` taught at a true mechanical zero (optional — re-teach poses after)
- [x] `Ki` tested and rejected for now (step 5c's `--ki` sweep: ki=8 closes the
      error to 0.001 rad but quadruples settling time)
- [ ] re-check the gain with a payload on the gripper — the sweep only measures
      an unloaded static hold, which is why Kp=64 was taken over a better-scoring
      128
