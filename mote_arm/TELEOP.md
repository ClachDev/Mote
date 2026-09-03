# Keyboard teleop and episode recording

Teleoperating an SO-101 normally takes two arms: an operator moves a **leader**
and the **follower** mirrors it. We have one arm and no intention of buying a
second, so the pose the arm follows is held in software and moved by the
keyboard.

The point of teleoperating at all is the **episodes**: recorded demonstrations
in [LeRobot](https://github.com/huggingface/lerobot)'s dataset format, which is
what a policy would later be learned from. Teleop without recording is just a
slower jog CLI.

```
  keyboard ─► arm_teleop ─► arm_controller ─► MoteHardware ─► servos
                  │  arm_controller/joint_trajectory
                  │
                  └────► episode_record ◄──── /image_raw/compressed
                                 │
                            capture dir
                             ╱        ╲
                 lerobot_export      episode_replay
              (off-board, LeRobot)   (back onto the arm)
```

## Why this shape

The task offered three candidate designs. This is the second, and why:

**LeRobot's own keyboard teleop** would have been cheapest and kept the dataset
story native, but it is not available to us: the [bring-up
decision](README.md#stack-decision-direct-feetech-control-not-lerobot) was
direct Feetech control precisely so that `torch` and the HuggingFace stack stay
off the Pi, the same way inference does. Running LeRobot's teleop means running
LeRobot's robot class, its bus driver and its calibration on the robot — the
thing that decision exists to avoid. LeRobot is still where the *dataset* goes;
it just does not need to be where the *arm* is driven from. That split is the
whole design.

**End-effector jog with IK** was explicitly out for v1, and rightly: there is no
off-the-shelf SO-101 IK we could drop in, and building one is a separate piece
of work.

**A commanded joint pose, moved by the keyboard** is what is built. The frontend
is the replaceable part, and the seam that makes it replaceable is
`mote_arm/teleop.py` — the safety rules as a library with no ROS in it, which a
gamepad or a slider GUI imports. See [Other frontends](#other-frontends).

### One process, and why it was two

This was `virtual_leader` and `arm_mirror`, two nodes with a
`leader/joint_states` topic between them, on the theory that a different
frontend would one day publish that topic. Nothing ever did; DDS here is
loopback-only, so no remote frontend could; and the seam that actually makes a
different frontend possible is the library, not the topic. What the split bought
in practice was a second terminal and a second thing to remember to start.

Two loops survive inside the one process, and that part is not incidental: the
keyboard reads on the main thread and the safety rules tick on their own,
because taking hold of the arm is a `switch_controller` call and a service call
made from inside an executor callback can never complete — the future is
resolved by the executor the callback is blocking.

### Teleop is not jog

The retired `arm-jog` typed a discrete step and pressed Enter. Teleop holds a
key and the arm moves continuously until you let go. That difference is the
reason this exists: an episode recorded from stop-start hops teaches a policy
stop-start hops. Step mode (`m`) is still there when a measured increment is
what you want, which is what jog was for.

## Safety

Everything that decides whether the arm may move lives in one place —
`mote_arm/teleop.py`, tested in `test/test_teleop.py` with no hardware attached.

| Rule | What it does |
|------|--------------|
| **Soft-limit clamping** | A commanded pose outside a joint's soft band is clamped before it becomes a goal. Clamped again in the driver, which is authoritative. |
| **Rate limiting** | The goal advances towards the commanded pose by at most `max_velocity * dt` (0.5 rad/s). A command that *jumps* — a slider dragged, a frontend restarted at a different pose — produces a ramp, never a lunge. |
| **Deadman** | The command's liveness *is* the deadman. A frontend offers a pose only while it is being driven, so a released key, a closed window and a dropped SSH session all arrive as the same thing: no fresh pose. One goal then goes out at the arm's *present* position — stopping it there rather than letting it coast to the setpoint it was travelling towards — and then nothing. |
| **Panic latch** | `SPACE` latches an e-stop. Torque *is* controller activation, so `arm_controller` is deactivated — the same switch `arm-pose` uses — and every goal is refused until `z` clears it. Torque coming back cannot restart the move. The latch no longer has to outlive the process, because the process that set it also holds the arm: exiting drops torque. |
| **Re-seeding** | Resuming after any hold starts from where the arm *is*, not from the command it was last given. Without that, a pause banks up the difference and pays it out as a jump. |

One structural consequence worth knowing: **the safety loop ticks on its own thread,
not on a ROS timer.** Taking hold of the arm is a `switch_controller` call, and a
service call made from inside an executor callback can never complete — the
future is resolved by the executor that the callback is currently blocking.
The retired jog CLI avoided this by driving from its REPL thread; teleop does the same
with a plain loop while `cli.spin_background` spins the node.

Two things the deadman is **not**: it is not a debounce (a single key tap moves
the arm for `--key-timeout` seconds — that is the terminal's key-repeat showing
through, and it is bounded at ~0.09 rad by default), and it does not cut torque
(the arm holds its pose; only panic goes limp).

## The workflow

Three terminals. Everything but the third also runs against `arm-mock`, which
is how you should rehearse it — see [Without hardware](#without-hardware).

### 1. The control stack

```bash
pixi run arm       # bench: controllers only
pixi run launch    # the whole base, when you need the camera
```

Something has to own the servo bus and offer `arm_controller`. During a mission
`pixi run robot` / `mapping` already does, so teleop runs beside it unchanged —
it is only an `arm_controller` client and never opens the bus.

### 2. Teleop

```bash
pixi run arm-teleop
```

```
hold  q/a w/s e/d r/f t/g y/h   move joints 1..6 up/down
tap   0        re-sync the commanded pose to the arm
tap   SPACE    PANIC: torque off, latched     z  clear it
tap   [ ]      slower / faster                ?  help    x  quit
```

The commanded pose starts synced to the arm, so nothing moves until you press a
key, and it re-syncs whenever it goes idle — it can never bank up a lead the arm
has to chase after you have stopped.

`--speed` (default 0.25 rad/s) sets how fast it moves; keep it at or below
`max_velocity` or the arm is permanently behind.

**If a joint stops short and stays there**, the live line marks it
`NOT FOLLOWING` and `pixi run arm-teleop --ros-args -p diagnose:=true` prints
the commanded and measured rates side by side. A command that keeps moving at
0.25 rad/s while the arm sits at 0.00 rad/s, at any load, is neither the rate
limit nor the deadman: check the servo's own goal-range fence with
`pixi run arm-setup limits show` (base stopped). It refuses goals outside its
band in silence, and reads exactly like a joint out of torque. See
[README](README.md#the-servos-own-goal-range-limits-which-are-not-the-soft-limits).

### 3. Record

```bash
pixi run arm-record -- --task "pick up the block" --dataset teleop
```

ENTER starts an episode, ENTER stops and keeps it, `r` discards a bad take, `q`
finishes. Recording samples at 20 Hz:

| Recorded | From |
|----------|------|
| `observation.state` | `joint_states` — where the arm is |
| `observation.images.front` | `/image_raw/compressed`, stored byte-for-byte |
| `action` | `arm_controller/joint_trajectory` — what it was commanded to reach |

The action is the *goal sent to the arm*, not the raw commanded pose, because a policy
replaces whatever produces goals — and it is read off the trajectory topic
rather than from the teleop node, so a session driven by `arm-pose` records too.

> The arm is mounted **rotated 180 degrees** so the camera clears it (GitHub
> #2), so episodes do record camera frames. Use `--no-camera` for a robot whose
> camera is off or fouled: the capture, the export and the replay all handle a
> camera-less dataset.

Captures land in `$MOTE_HOME/episodes/<dataset>/` — per-robot state, alongside
maps, zones and taught poses. The format is documented in `mote_arm/episode.py`:
JSON lines plus the compressed frames, written with nothing but the standard
library, because the Pi carries no parquet or ffmpeg and should not have to.

### 4. Export to a LeRobot dataset (off-board)

```bash
pixi run -e lerobot arm-export -- --capture ~/.mote/episodes/teleop \
    --repo-id mote/teleop-demo
```

The `lerobot` environment is linux-64 and no-default-feature, for the same
reason `inference` has its own: LeRobot brings torch, ffmpeg and the
HuggingFace stack, none of which belongs on the aarch64 Pi that did the
recording. Copy the capture off the robot (`rsync`) and convert it there.

The exporter writes through `LeRobotDataset.create` / `add_frame` /
`save_episode` / `finalize` rather than emitting the files itself. The format
has already moved once (v2.1's file-per-episode became v3.0's aggregated
shards); a hand-rolled writer would be a second implementation of someone
else's schema, wrong the first time it changed.

Two things it does on the way:

- **Resampling.** LeRobot stores no timestamps — it derives them from the frame
  index and the dataset's fps. A capture whose timer slipped would export as if
  it had not, silently stretching the motion, so every episode is put on the
  exact 1/fps grid first (zero-order hold, never a peek ahead).
- **Decoding.** The stored frames are decoded to RGB here, off-board, where
  Pillow exists.

`--dry-run` reports the schema and the resampled frame counts using only the
capture, so the conversion can be checked on a machine with no LeRobot at all.

### 5. Inspect it with LeRobot's own tooling

```bash
pixi run -e lerobot -- lerobot-dataset-viz \
    --repo-id mote/teleop-demo --root <out> --episode-index 0
```

### 6. Replay on the arm

```bash
pixi run arm-replay -- ~/.mote/episodes/teleop --episode 0
```

Stop teleop first — two things commanding `arm_controller` fight
over the arm. (The stall guard does catch it, which is how that was found, but a
caught stall is not a passing replay.)

Replay reads the **capture**, not the exported dataset, so it needs nothing
off-board. Three gates, in order:

1. **Reduced speed** — actions are issued at `fps * --speed-scale`, a quarter of
   the recorded rate by default. The same path, not the same dynamics.
2. **Approach, then replay** — the arm is walked to the episode's first pose
   first, and refused if it starts further away than `--max-travel`. That is a
   check on the recording, not on the motion: a replay begun from somewhere the
   episode never saw will not reproduce it. (`arm-pose go` has no such limit —
   it has no expectation about where the arm starts.)
3. **Lag supervision** — the rule that guards `arm-pose go`: if the arm trails
   its setpoint for `--stall-time`, the replay stops where it is.

Every action is clamped to the *current* `robot.yaml` limits, so an episode
recorded before a limit was tightened cannot replay outside today's envelope.

## Without hardware

`arm-mock` presents exactly the interface ros2_control does — `joint_states`,
`arm_controller/joint_trajectory`, and `controller_manager/switch_controller` —
with no bus behind it, and `--camera` adds a synthetic camera whose picture
tracks the first joint. It starts limp, as the real stack does, so the first
command is what takes hold. Teleop, recording, export and replay cannot tell the
difference.

```bash
pixi run arm-mock -- --camera --droop 0.01   # terminal 1
pixi run arm-teleop                          # terminal 2
```

`--droop` leaves a constant steady-state error, the way a proportional servo
with `ki = 0` settles under load. Without it the mock lands exactly on every
setpoint and a recorded action is indistinguishable from the observed state.

The whole loop runs headless as one command:

```bash
pixi run arm-teleop-test
```

It drives the real nodes (mock follower → `arm_teleop --demo`),
records, checks the capture holds an actual motion, replays it, and plans the
export. Run it before taking anything here to the bench.

## Verified

Run on 2026-08-05 against the mock control stack (no arm, no camera). The
hardware half — the three safety *observations* and a real arm retracing an
episode — is step 8 of `BENCH.md` and is still open.

| Check | Result |
|-------|--------|
| Teleop loop, headless | `pixi run arm-teleop-test`: teleop -> arm_controller -> arm -> record -> replay -> export plan, all green |
| Taking hold | the mock starts with `arm_controller` inactive, as the real stack spawns it; the first commanded goal activates it |
| Deadman in the loop | logged `deadman: no input, holding position` / `following the keyboard` on every pause the demo took |
| Two things commanding one arm | caught by the stall guard before the script learned to stop teleop first — the replay halted at 24/220 with 0.209 rad of lag instead of fighting |
| Recording | 220 frames over 10.9 s at 20 fps, 0 dropped ticks, camera frames all distinct |
| Replay | 220 setpoints at half speed, lag steady at 0.010 rad (the mock's droop), finished within 0.0000 rad of the last action |
| Export (camera) | v3.0 dataset: `data/chunk-000/file-000.parquet`, `videos/observation.images.front/chunk-000/file-000.mp4`, `meta/episodes/chunk-000/file-000.parquet` |
| Export (`--no-camera`) | same, state + action only — the path the arm/camera clash forces today |
| Loads back through LeRobot | 1 episode, 220 frames, 20 fps, `so101_follower`; sample shapes `observation.state (6,)`, `action (6,)`, `observation.images.front (3, 72, 96)`, task string intact |
| LeRobot's own viewer | `lerobot-dataset-viz --save 1` read the dataset and wrote a 619 KB `.rrd` |
| Safety rules | 15 unit tests over `teleop.py` (clamp, rate limit, deadman halt-then-silence, re-seed on resume, panic latch) plus 7 node tests through `ArmTeleop` against the mock |

The unit tests are the load-bearing ones: every safety rule is decided in
`teleop.py`, so it can be checked exhaustively without a bus.

## Other frontends

The replaceable part is `mote_arm/teleop.py`: `LeaderMirror` holds every safety
rule — clamping, the rate limit, the deadman, the panic latch — with no ROS in
it. A gamepad, a slider GUI or a script becomes a frontend by importing that and
feeding it poses, exactly as `ArmTeleop` does; what it must not do is command
`arm_controller` around it.

That seam used to be a ROS topic instead, on the theory that a frontend would
publish `leader/joint_states` from somewhere else. Nothing did, and nothing
could have from off the robot: DDS here is loopback-only by design, so the
control surface for a remote arm would not be a topic in the first place.

## Files

| Piece | What it is |
|-------|------------|
| `teleop.py` | The follow rule — clamping, rate limiting, deadman, panic latch. ROS-free, unit-tested. |
| `arm_teleop.py` | `arm-teleop`: the keyboard, the safety loop and the arm, in one node. `--demo N` sweeps without a terminal. |
| `diagnostics.py` | `-p diagnose:=true`: tick rate, command rate, arm rate and lag, per second. |
| `mock_arm.py` | The control stack's interface with no hardware (`arm-mock`). |
| `episode.py` | The capture format: writer, reader, fps resampling. ROS-free. |
| `episode_record.py` | `arm-record` — observations and actions into a capture. |
| `episode_replay.py` | `arm-replay` — a capture back onto the arm, gated. |
| `motion.py` | Lag supervision, shared with `arm-pose go`. |
| `control.py` | Shared with `arm-pose` and replay: single-point trajectories, and activation as the torque switch. |
| `tools/lerobot_export.py` | Capture → LeRobotDataset, off-board (`-e lerobot`). |
| `test/teleop_loop/` | The headless end-to-end gate (`arm-teleop-test`). |
| `tools/bench_teleop.sh` | The guided hardware session (see `BENCH.md`). |
