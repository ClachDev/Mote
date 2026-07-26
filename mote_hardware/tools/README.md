# mote_hardware tools

Dev tools built alongside the hardware package. All use the SCServo SDK directly,
bypassing ros2_control. Run with `ros2 run mote_hardware <tool>`.

## setup_ids

Guided first-time servo ID assignment. Sets the wheel IDs Mote expects:
left = 7, right = 9 (matching LeKiwi). Fresh STS3215 servos all ship as ID 1, so
they can't be told apart on a shared bus — this tool configures them one at a
time, detecting whichever single ID is on the bus and reassigning it.

```
ros2 run mote_hardware setup_ids [port] [baud]
```

Connect only the left servo when prompted, then only the right. Safe to re-run
(it handles servos already at 7 or 9). Use `swap_ids` instead if both IDs are
already set but the wheels drive the wrong way round.

## servo_debug

Interactive REPL for driving and inspecting individual servos. Useful for
checking wiring, servo IDs, and direction conventions.

```
ros2 run mote_hardware servo_debug [port] [baud]
```

Commands: `mv <id> <speed>`, `stop <id>`, `stopall`, `r <id>`, `m <id> [hz]`,
`ping <lo> <hi>`. Type `help` for the full list.

## servo_ping

Non-interactive counterpart of `servo_debug`'s `ping` — pings a fixed set of
servo IDs and exits `0` if every one responded, `1` otherwise. Used by the
startup self-check (`mote_bringup` `self_check.py`) to gate bringup on the drive
servos actually answering. Run the bus free (before bringup opens it).

```
ros2 run mote_hardware servo_ping [port] [baud] [id ...]
```

Defaults: `/dev/mote_servos 1000000`, IDs `7 9` (the drive IDs in `robot.yaml`).

## velocity_cal

Measures `velocity_scale` (the rad/s → raw servo units conversion factor) by
commanding a series of speeds and computing actual angular velocity from encoder
ticks. Lift the robot before running so the wheels spin freely.

```
ros2 run mote_hardware velocity_cal [servo_id] [port] [baud]
```

Outputs a recommended `velocity_scale` value. Update it in:
- `mote_description/urdf/mote.urdf.xacro`
- `mote_bringup/launch/mote_launch.py`
