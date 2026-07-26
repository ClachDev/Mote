# Chaos validation

Two scripts prove the reliability stack actually recovers, split by what they
need:

## `chaos_policy_demo.sh` — local, no hardware

Proves the **systemd layer**: a transient `--user` unit mirroring the mote
services' `Restart=always` + `RestartSec`/`RestartSteps`/`RestartMaxDelaySec`
policy is SIGKILLed, and systemd restarts it (new MainPID) within a bounded
time. Runs on any workstation with a user systemd manager — no ROS, no robot.

```bash
bash mote_bringup/test/chaos/chaos_policy_demo.sh
```

Logs go to `/tmp/mote_chaos_policy_log.txt` (override with `CHAOS_LOG`). The
committed `chaos_log.txt` is curated evidence, not scratch output — writing into
a tracked file on every run dirties the worktree and breaks git operations.

## `chaos_restart.sh` — on the robot

Proves **per-node respawn**: with the stack up on `auldbot` (services active, or
`pixi run robot` / `pixi run mapping` running), it SIGKILLs `ros2_control_node`,
`sllidar_node`, and `controller_server` in turn and waits for each to reappear
within 30 s. Recovery comes from `respawn=True` on those nodes in
`mote_launch.py` / `nav2_launch.py`.

```bash
pixi run chaos          # on the robot
```

Logs go to `/tmp/mote_chaos_log.txt` (override with `CHAOS_LOG`).

It aborts safely (exit 2, nothing killed) if the stack is not running, so it is
harmless to invoke on a workstation. Nodes are matched by executable name and
the script excludes its own PID, so `pkill`-style self-matching cannot happen.

**This half must be benched on the robot with Michael** — it is not part of CI
because it needs the live hardware stack. Capture its `chaos_log.txt` from that
run for the record.

## Two layers of recovery

1. **Node crash** → the launch system relaunches it (`respawn=True`, ~2 s).
2. **Launch / process crash** → systemd restarts the whole service (backoff).
3. **Monitor hang** → `mote-health.service` (`Type=notify` + `WatchdogSec=15`)
   is killed and restarted by systemd when it stops petting the watchdog.
