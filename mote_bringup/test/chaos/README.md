# Chaos validation

`chaos_restart.sh` kills the critical processes on a running robot and checks
that each one comes back within a bounded time. Recovery comes from
`respawn=True` on the drivers in `mote_launch.py` and on the Nav2 container in
`nav2_launch.py`; this proves it end to end against real hardware.

```bash
pixi run chaos          # on the robot, with the stack up
```

Bring the stack up first (`pixi run robot` / `pixi run mapping`, or the systemd
units if you have enabled them). The script SIGKILLs `ros2_control_node`,
`sllidar_node` and `component_container_isolated` in turn and waits up to 30 s
for each to reappear. Logs go to `/tmp/mote_chaos_log.txt` — override with
`CHAOS_LOG`.

The third target is the whole of Nav2: it is composed into one container
process, so there is no `controller_server` process to kill any more and no
per-server recovery to test — the container is the unit. Its process comes back
on respawn within a couple of seconds, but recovery is not *complete* until the
launch file has reloaded the components into it and the lifecycle managers have
re-activated them, which this script does not wait for. Confirm that separately
with `ros2 node list` (the servers reappear) or `ros2 lifecycle get
/controller_server`.

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

1. **Node crash** → the launch system relaunches that process (`respawn=True`);
   for Nav2 the process is the container, so the whole stack goes together.
2. **Launch/process crash** → systemd restarts the whole service, re-running the
   self-check gate on the way back up.
3. **Health-monitor hang** → `mote-health.service` (`Type=notify` +
   `WatchdogSec=15`) is killed and restarted when it stops petting the watchdog.

Layers 2 and 3 are systemd's; `mote_bringup/README.md` documents the unit
configuration that makes them work, including two traps worth knowing about: a
`Wants=` dependent defeats the parent's restart backoff, and hardcoding a
checkout path in a unit template breaks installs from a second checkout.
