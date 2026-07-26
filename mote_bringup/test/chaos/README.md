# Chaos validation

`chaos_restart.sh` kills the critical nodes on a running robot and checks that
each one comes back within a bounded time. Recovery comes from `respawn=True` on
those nodes in `mote_launch.py` / `nav2_launch.py`; this proves it end to end
against real hardware.

```bash
pixi run chaos          # on the robot, with the stack up
```

Bring the stack up first (`pixi run robot` / `pixi run mapping`, or the systemd
units if you have enabled them). The script SIGKILLs `ros2_control_node`,
`sllidar_node` and `controller_server` in turn and waits up to 30 s for each to
reappear. Logs go to `/tmp/mote_chaos_log.txt` — override with `CHAOS_LOG`.

It is not part of CI: it needs the live hardware stack, so run it by hand on the
robot when the recovery paths change.

## Safety and gotchas

- It aborts (exit 2, nothing killed) if the stack is not running, so it is
  harmless to invoke on a workstation.
- Nodes are matched by the basename of `/proc/<pid>/exe`, and the script skips
  its own PID, so `pkill -f`-style self-matching cannot happen.
- That matching only finds **compiled** nodes: a Python node's `exe` is the
  interpreter (`python3.12`), not the node. All three targets here are C++
  binaries. If you add a Python target, match on the cmdline instead. A target
  that is never found aborts the run rather than reporting a false failure.
- `bc` is not installed on the Pi, so all timing here is integer milliseconds in
  pure bash.
- **Tearing down afterwards**: `pixi run robot` also starts the rosbag
  recorders, and killing a launch's nodes does not stop them. Sweep for leftover
  `ros2 bag record` processes (and the `~/.mote/bags/<stream>/<timestamp>`
  directories they leave) after a session, or they keep recording.

## Three layers of recovery

1. **Node crash** → the launch system relaunches that node (`respawn=True`).
2. **Launch/process crash** → systemd restarts the whole service, re-running the
   self-check gate on the way back up.
3. **Health-monitor hang** → `mote-health.service` (`Type=notify` +
   `WatchdogSec=15`) is killed and restarted when it stops petting the watchdog.

Layers 2 and 3 are systemd's; `mote_bringup/README.md` documents the unit
configuration that makes them work, including two traps worth knowing about: a
`Wants=` dependent defeats the parent's restart backoff, and hardcoding a
checkout path in a unit template breaks installs from a second checkout.
