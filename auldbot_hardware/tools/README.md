# auldbot_hardware tools

Dev tools built alongside the hardware package. All use the SCServo SDK directly,
bypassing ros2_control. Run with `ros2 run auldbot_hardware <tool>`.

## servo_debug

Interactive REPL for driving and inspecting individual servos. Useful for
checking wiring, servo IDs, and direction conventions.

```
ros2 run auldbot_hardware servo_debug [port] [baud]
```

Commands: `mv <id> <speed>`, `stop <id>`, `stopall`, `r <id>`, `m <id> [hz]`,
`ping <lo> <hi>`. Type `help` for the full list.

## velocity_cal

Measures `velocity_scale` (the rad/s → raw servo units conversion factor) by
commanding a series of speeds and computing actual angular velocity from encoder
ticks. Lift the robot before running so the wheels spin freely.

```
ros2 run auldbot_hardware velocity_cal [servo_id] [port] [baud]
```

Outputs a recommended `velocity_scale` value. Update it in:
- `auldbot_description/urdf/auldbot.urdf.xacro`
- `auldbot_bringup/launch/auldbot_launch.py`
