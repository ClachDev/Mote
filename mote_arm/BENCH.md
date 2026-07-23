# SO-101 arm bench validation

A short, scripted first-light session. **A human at the bench is required.**
Keep the arm clear of obstructions and be ready to cut power. The arm starts
limp, so support it or rest it in a stable pose before each step.

Prerequisites: servos wired and enumerated, robot powered, repo built
(`pixi run build`).

---

## Step 0 — confirm the wiring (one-time)

Decide how the arm bus is wired and make `robot.yaml` match:

- **Own USB-serial adapter (assumed default):** the arm is a second serial
  device. Continue to Step 1 to install its `/dev/mote_arm` symlink.
- **Shares the wheel driver board:** set `arm.port: /dev/mote_servos` in
  `robot.yaml`, ensure arm servo IDs don't collide with the wheels (7, 9), and
  skip the udev step.

## Step 1 — install the udev symlink (own-adapter wiring only)

With the arm adapter plugged in:

```
pixi run arm-check
```

`arm-check` will fail to open `/dev/mote_arm` the first time (the symlink does
not exist yet) but still prints a **udev helper** line with the adapter's
`idVendor` / `idProduct` / `serial`. Paste that line into
`mote_bringup/udev/99-mote.rules` (uncomment the arm rule, fill the serial),
then:

```
pixi run udev
```

Unplug/replug the adapter and confirm `/dev/mote_arm` now exists
(`ls -l /dev/mote_arm`).

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

## Step 3 — teach home offsets (calibration)

Move each joint by hand (it's limp) to its mechanical zero / neutral pose, then:

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

## Step 5 — jog each joint through a small range

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

**Expected observations per joint:**

| Observation | Meaning |
|-------------|---------|
| Joint moves smoothly a small amount per `+`/`-` | position control + scaling OK |
| `/joint_states` value follows the jog | feedback + conversion OK |
| Direction matches the `+` sign | `invert` correct |

## Step 6 — demonstrate the soft-limit clamp

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

- [ ] `/dev/mote_arm` resolves (or `arm.port` points at the shared wheel bus)
- [ ] all six joints respond in `arm-check`
- [ ] home offsets taught; neutral pose reads ~0 rad
- [ ] `/joint_states` shows live arm joints
- [ ] every joint jogs in the correct direction through a small range
- [ ] soft limits demonstrably clamp both ends
- [ ] jog quit and driver shutdown both leave the arm limp
- [ ] `min`/`max`/`home`/`invert` in `robot.yaml` updated from what you measured
