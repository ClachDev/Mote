"""Startup self-check: prove the hardware is present and ready before bringup.

Run as ``ExecStartPre`` of ``mote-bringup.service`` (and by hand via
``pixi run self-check``). It performs fast, static pre-flight checks — no ROS
graph, so it is cheap and can gate the launch:

* nothing else already holds the shared servo bus (the arm sits on it too)
* drive servos answer a bus ping (``servo_ping``, SCServo)
* lidar and camera devices enumerate and open
* enough free disk for logs and bags
* the wall clock looks synchronised (an RTC-less Pi boots at the epoch)
* ``robot.yaml`` parses and the active site resolves

Device paths and servo IDs come from ``robot.yaml`` (the single source of
truth). Each check is CRITICAL or advisory: any failed CRITICAL check makes the
process exit non-zero, which fails the ExecStartPre and keeps the robot in a
safe idle (systemd retries with backoff, so replugging the lidar self-heals).
Advisory failures are logged but do not block.

The result is written to ``$MOTE_HOME/self_check_status.yaml`` (default
``~/.mote``) so the running ``health_monitor`` can surface the last pre-flight
verdict on ``/diagnostics_agg`` after boot. It is also printed to stdout, which
journald captures.

Runtime data-flow liveness (is ``/scan`` actually publishing?) is deliberately
*not* checked here — the drivers are not up yet. That is the health_monitor's
job once bringup runs; this stage proves the hardware is present and claimable.
"""

import argparse
import os
import shutil
import stat
import subprocess
import sys
import time
from datetime import datetime, timezone

import yaml
from ament_index_python.packages import get_package_share_directory

from mote_bringup import mote_home, serial_bus

CRITICAL = "CRITICAL"
ADVISORY = "advisory"

# Free-space floor. Below the critical bound the robot should not start
# (recording would fill the disk); the warn bound is an early heads-up.
DISK_CRITICAL_MB = 500
DISK_WARN_MB = 2048

# A clock before this instant means NTP has not synced yet (or there is no RTC),
# which corrupts every bag and TF stamp. 2024-01-01 UTC.
CLOCK_FLOOR_EPOCH = 1704067200


class Result:
    def __init__(self):
        self.checks = []
        self.ok = True

    def add(self, name, passed, severity, detail):
        self.checks.append(
            {"name": name, "passed": passed, "severity": severity, "detail": detail}
        )
        if not passed and severity == CRITICAL:
            self.ok = False
        mark = "PASS" if passed else ("FAIL" if severity == CRITICAL else "WARN")
        print(f"[{mark:4}] {name}: {detail}")


def _load_robot_cfg():
    share = get_package_share_directory("mote_description")
    with open(os.path.join(share, "config", "robot.yaml")) as f:
        return yaml.safe_load(f)


def _check_device(result, name, path, severity):
    if not os.path.exists(path):
        result.add(name, False, severity, f"{path} not present")
        return False
    try:
        mode = os.stat(path).st_mode
        if not (stat.S_ISCHR(mode) or stat.S_ISBLK(mode)):
            # A dangling symlink resolves but is not a device node.
            result.add(name, False, severity, f"{path} is not a device node")
            return False
        fd = os.open(path, os.O_RDWR | os.O_NONBLOCK)
        os.close(fd)
    except OSError as exc:
        result.add(name, False, severity, f"{path} present but not openable: {exc}")
        return False
    result.add(name, True, severity, f"{path} present and openable")
    return True


def _check_servos(result, cfg, do_ping):
    servos = cfg["servos"]
    port, baud = servos["port"], servos["baud_rate"]
    ids = [servos["left_id"], servos["right_id"]]
    if not _check_device(result, "servo_bus", port, CRITICAL):
        return

    # The arm shares this bus with the wheels, and serial ports have no
    # kernel-level exclusion — pinging into someone else's traffic interleaves
    # packets on a half-duplex bus, so both sides see corrupt replies and the
    # ping reports a misleading "servos did not respond". Refuse instead, naming
    # the holder: the base cannot run while another process owns the bus anyway.
    holders = serial_bus.port_holders(port)
    if holders:
        who = "; ".join(f"pid {pid} ({cmd[:60]})" for pid, cmd in holders)
        result.add("servo_bus_free", False, CRITICAL, f"{port} held by {who}")
        return
    result.add("servo_bus_free", True, CRITICAL, f"{port} not held by another process")

    if not do_ping:
        result.add("servo_ping", True, ADVISORY, "skipped (--no-servo-ping)")
        return
    cmd = ["ros2", "run", "mote_hardware", "servo_ping", port, str(baud)]
    cmd += [str(i) for i in ids]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=20)
    except FileNotFoundError:
        result.add("servo_ping", True, ADVISORY, "servo_ping tool not built; skipped")
        return
    except subprocess.TimeoutExpired:
        result.add("servo_ping", False, CRITICAL, "servo_ping timed out")
        return
    detail = proc.stdout.strip().replace("\n", "; ") or proc.stderr.strip()
    result.add("servo_ping", proc.returncode == 0, CRITICAL, detail or "no output")


def _check_disk(result):
    home = mote_home.mote_dir()
    target = home if home.exists() else os.path.expanduser("~")
    free_mb = shutil.disk_usage(target).free / (1024 * 1024)
    detail = f"{free_mb:.0f} MB free on {target}"
    if free_mb < DISK_CRITICAL_MB:
        result.add("disk_space", False, CRITICAL, detail)
    elif free_mb < DISK_WARN_MB:
        result.add("disk_space", False, ADVISORY, detail + " (low)")
    else:
        result.add("disk_space", True, CRITICAL, detail)


def _check_clock(result):
    now = time.time()
    synced = now >= CLOCK_FLOOR_EPOCH
    iso = datetime.fromtimestamp(now, timezone.utc).isoformat(timespec="seconds")
    result.add(
        "clock",
        synced,
        ADVISORY,
        f"system time {iso}" + ("" if synced else " (unsynced — pre-2024)"),
    )


def _check_config(result, cfg):
    ok = "wheel_radius" in cfg and "servos" in cfg
    result.add("robot_yaml", ok, CRITICAL, "robot.yaml parsed" if ok else "malformed")
    # Active site is only needed for navigation against a saved map; mapping runs
    # without one, so a missing site is advisory.
    try:
        from mote_bringup import sites

        act = sites.active()
        detail = f"active site {act[0]}/{act[1]}" if act else "no active site selected"
        result.add("active_site", act is not None, ADVISORY, detail)
    except Exception as exc:  # sites resolution is best-effort here
        result.add("active_site", False, ADVISORY, f"unresolved: {exc}")


def _write_status(result):
    home = mote_home.mote_dir()
    try:
        home.mkdir(parents=True, exist_ok=True)
        payload = {
            "ok": result.ok,
            "timestamp": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "checks": result.checks,
        }
        with open(home / "self_check_status.yaml", "w") as f:
            yaml.safe_dump(payload, f, sort_keys=False)
    except OSError as exc:
        print(f"[WARN] could not write self_check_status.yaml: {exc}")


def run(do_ping=True):
    result = Result()
    try:
        cfg = _load_robot_cfg()
    except Exception as exc:
        result.add("robot_yaml", False, CRITICAL, f"could not load robot.yaml: {exc}")
        _write_status(result)
        return result
    _check_servos(result, cfg, do_ping)
    lidar_port = cfg.get("lidar", {}).get("port", "/dev/mote_lidar")
    _check_device(result, "lidar", lidar_port, CRITICAL)
    cam_dev = cfg.get("camera", {}).get("device", "/dev/mote_camera")
    _check_device(result, "camera", cam_dev, ADVISORY)
    _check_disk(result)
    _check_clock(result)
    _check_config(result, cfg)
    _write_status(result)
    return result


def main():
    parser = argparse.ArgumentParser(description="Mote startup self-check")
    parser.add_argument(
        "--no-servo-ping",
        action="store_true",
        help="skip the active servo bus ping (device presence still checked)",
    )
    args = parser.parse_args()
    result = run(do_ping=not args.no_servo_ping)
    verdict = "READY" if result.ok else "NOT READY — mission blocked"
    print(f"self-check: {verdict}")
    sys.exit(0 if result.ok else 1)


if __name__ == "__main__":
    main()
