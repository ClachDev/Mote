# SO-101 arm bench validation

A short, scripted first-light session. **A human at the bench is required.**
Keep the arm clear of obstructions and be ready to cut power. The arm starts
limp, so support it or rest it in a stable pose before each step.

Prerequisites: servos wired and enumerated, robot powered, repo built
(`pixi run build`).

---

**Steps 0–2 and 4 are already done** — they were run against the robot on
2026-07-25 and their results are recorded in `README.md`. They are kept here
because they are the right first checks after any rewiring. The work that still
genuinely needs a human is **step 3** (teach true mechanical zeros) and
**steps 5–6** (jog each joint, confirm direction and range).

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
position (0–4095), a plausible voltage (~6–12 V depending on supply), a
temperature (< 55 °C), and a load near 0 while limp. Any `NO RESPONSE` row means
a wiring/ID problem — fix IDs with `pixi run setup-ids` / `ros2 run
mote_hardware servo_debug` before continuing.

## Step 3 — teach home offsets (calibration) — **NEEDS A HUMAN**

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

## Step 5 — jog each joint through a small range — **NEEDS A HUMAN**

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

(The clamp path itself is already proven on hardware — see README's "Verified on hardware". What is unproven is that each joint moves the *right way* through a *real* range.)

**Expected observations per joint:**

| Observation | Meaning |
|-------------|---------|
| Joint moves smoothly a small amount per `+`/`-` | position control + scaling OK |
| `/joint_states` value follows the jog | feedback + conversion OK |
| Direction matches the `+` sign | `invert` correct |

## Step 6 — demonstrate the soft-limit clamp — **NEEDS A HUMAN**

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

Still needs a human at the bench:

- [ ] home offsets taught at true mechanical zero; neutral pose reads ~0 rad
- [ ] every joint jogs in the correct direction through a small range
- [ ] soft limits clamp at both ends of a *real* range
- [ ] `min`/`max`/`home`/`invert` in `robot.yaml` updated from what you measured
