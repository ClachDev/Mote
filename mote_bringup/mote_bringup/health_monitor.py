"""Robot-level health monitor.

Watches the liveness of the safety-critical subsystems (lidar scan, filtered
scan, wheel/joint feedback, odometry TF) plus non-critical ones (camera,
localisation TF) and folds in the host status published by ``system_monitor``.
Every ``period`` seconds it publishes:

* ``/diagnostics_agg`` (``diagnostic_msgs/DiagnosticArray``) — one
  ``DiagnosticStatus`` per subsystem plus a rolled-up ``mote`` status, in the
  standard form the fleet layer can lift later.
* ``/health`` (``std_msgs/String``) — a single human-readable summary line,
  ``OK`` / ``DEGRADED: ...`` / ``FAULT: ...``, easy to ``ros2 topic echo``.

Criticality decides the roll-up: a stale *critical* subsystem is a FAULT
(ERROR), a stale non-critical one is DEGRADED (WARN), and a fresh-but-slow
subsystem is DEGRADED. Expectations live in ``config/health.yaml`` (overridable
at ``~/.mote/health.yaml``).

Runs as its own ``mote-health.service`` (``Type=notify`` + ``WatchdogSec``): it
sends ``READY=1`` once spun up and pets the systemd watchdog on every publish,
so a hung monitor is itself restarted. Outside systemd the watchdog calls are
silent no-ops, so ``pixi run health`` behaves identically.
"""

import os
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String

import tf2_ros

from mote_bringup.sd_notify import SdNotifier

LEVEL_NAME = {
    DiagnosticStatus.OK: "OK",
    DiagnosticStatus.WARN: "DEGRADED",
    DiagnosticStatus.ERROR: "FAULT",
    DiagnosticStatus.STALE: "STALE",
}


def _load_config():
    default = os.path.join(
        get_package_share_directory("mote_bringup"), "config", "health.yaml"
    )
    override = os.path.expanduser("~/.mote/health.yaml")
    path = override if os.path.exists(override) else default
    with open(path) as f:
        return yaml.safe_load(f)


class _TopicWatch:
    """Freshness + rate tracker for one subscribed topic."""

    def __init__(self, spec):
        self.name = spec["name"]
        self.topic = spec["topic"]
        self.min_rate = spec.get("min_rate")
        self.timeout = spec.get("timeout", 2.0)
        self.critical = spec.get("critical", False)
        self.last_stamp = None
        self.count = 0

    def on_msg(self, _msg):
        self.last_stamp = time.monotonic()
        self.count += 1

    def evaluate(self, window):
        rate = self.count / window if window > 0 else 0.0
        self.count = 0
        age = None if self.last_stamp is None else time.monotonic() - self.last_stamp

        values = {"topic": self.topic, "rate_hz": f"{rate:.1f}"}
        if age is None:
            level = DiagnosticStatus.ERROR if self.critical else DiagnosticStatus.WARN
            return level, "no messages received", values
        values["age_s"] = f"{age:.1f}"
        if age > self.timeout:
            level = DiagnosticStatus.ERROR if self.critical else DiagnosticStatus.WARN
            return level, f"stale ({age:.1f}s > {self.timeout:.1f}s)", values
        if self.min_rate is not None and rate < self.min_rate:
            return (
                DiagnosticStatus.WARN,
                f"slow ({rate:.1f} < {self.min_rate:.1f} Hz)",
                values,
            )
        return DiagnosticStatus.OK, "ok", values


class _TfWatch:
    """Freshness tracker for one TF edge."""

    def __init__(self, spec):
        self.name = spec["name"]
        self.parent = spec["parent"]
        self.child = spec["child"]
        self.timeout = spec.get("timeout", 2.0)
        self.critical = spec.get("critical", False)

    def evaluate(self, buffer, now):
        values = {"transform": f"{self.parent}->{self.child}"}
        try:
            tf = buffer.lookup_transform(self.parent, self.child, rclpy.time.Time())
        except tf2_ros.TransformException as exc:
            level = DiagnosticStatus.ERROR if self.critical else DiagnosticStatus.WARN
            values["error"] = str(exc)[:80]
            return level, "unavailable", values
        age = (now - rclpy.time.Time.from_msg(tf.header.stamp)).nanoseconds / 1e9
        values["age_s"] = f"{age:.1f}"
        if age > self.timeout:
            level = DiagnosticStatus.ERROR if self.critical else DiagnosticStatus.WARN
            return level, f"stale ({age:.1f}s > {self.timeout:.1f}s)", values
        return DiagnosticStatus.OK, "ok", values


class HealthMonitor(Node):
    def __init__(self):
        super().__init__("health_monitor")
        cfg = _load_config()
        self.period = float(cfg.get("period", 1.0))

        self.topics = []
        for spec in cfg.get("topics", []):
            watch = _TopicWatch(spec)
            msg_type = get_message(spec["type"])
            self.create_subscription(msg_type, spec["topic"], watch.on_msg, 10)
            self.topics.append(watch)

        self.tf_watches = [_TfWatch(s) for s in cfg.get("tf", [])]
        self.tf_buffer = None
        if self.tf_watches:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.host_status = None
        if cfg.get("subscribe_diagnostics", True):
            self.create_subscription(
                DiagnosticArray, "diagnostics", self._on_diagnostics, 10
            )

        home = os.environ.get("MOTE_HOME", os.path.expanduser("~/.mote"))
        self._selfcheck_path = os.path.join(home, "self_check_status.yaml")
        self._selfcheck_mtime = None
        self._selfcheck_status = None

        self.agg_pub = self.create_publisher(DiagnosticArray, "diagnostics_agg", 10)
        self.health_pub = self.create_publisher(String, "health", 10)

        self._last_tick = time.monotonic()
        self.create_timer(self.period, self._tick)

        # systemd watchdog integration (no-op outside a Type=notify service).
        self._sd = SdNotifier()
        self._sd.ready(status="health monitor up")

    def _on_diagnostics(self, msg):
        # Keep the worst host-level status from system_monitor for the roll-up.
        worst = None
        for status in msg.status:
            if status.name.startswith("system") or status.hardware_id:
                if worst is None or status.level > worst.level:
                    worst = status
        if worst is not None:
            self.host_status = worst

    def _tick(self):
        now_wall = time.monotonic()
        window = now_wall - self._last_tick
        self._last_tick = now_wall
        now_ros = self.get_clock().now()

        statuses = []
        overall = DiagnosticStatus.OK
        faults = []

        for watch in self.topics:
            level, message, values = watch.evaluate(window)
            statuses.append(self._status(watch.name, level, message, values))
            overall = max(overall, level)
            if level >= DiagnosticStatus.WARN:
                faults.append(f"{watch.name} {message}")

        if self.tf_buffer is not None:
            for tf_watch in self.tf_watches:
                level, message, values = tf_watch.evaluate(self.tf_buffer, now_ros)
                statuses.append(self._status(tf_watch.name, level, message, values))
                overall = max(overall, level)
                if level >= DiagnosticStatus.WARN:
                    faults.append(f"{tf_watch.name} {message}")

        if self.host_status is not None:
            statuses.append(self.host_status)
            overall = max(overall, self.host_status.level)
            if self.host_status.level >= DiagnosticStatus.WARN:
                faults.append(f"host {self.host_status.message}")

        selfcheck = self._read_selfcheck()
        if selfcheck is not None:
            statuses.append(selfcheck)
            # A failed pre-flight is informational at runtime (bringup would not
            # have started on a hard failure); surface it without forcing FAULT.
            if selfcheck.level >= DiagnosticStatus.WARN:
                faults.append(f"self_check {selfcheck.message}")
                overall = max(overall, DiagnosticStatus.WARN)

        summary_word = LEVEL_NAME.get(overall, "UNKNOWN")
        summary_text = summary_word
        if faults:
            summary_text = f"{summary_word}: " + ", ".join(faults)

        mote_status = self._status(
            "mote", overall, summary_text, {"subsystems": str(len(statuses))}
        )
        arr = DiagnosticArray(status=[mote_status, *statuses])
        arr.header.stamp = now_ros.to_msg()
        self.agg_pub.publish(arr)
        self.health_pub.publish(String(data=summary_text))

        # Prove liveness to systemd only after a successful publish.
        self._sd.watchdog()

        if overall >= DiagnosticStatus.WARN:
            self.get_logger().warning(summary_text, throttle_duration_sec=5.0)

    def _read_selfcheck(self):
        """Last pre-flight verdict written by self_check, or None if absent.

        Re-read only when the file changes so this stays cheap on every tick.
        """
        try:
            mtime = os.path.getmtime(self._selfcheck_path)
        except OSError:
            return None
        if mtime != self._selfcheck_mtime:
            self._selfcheck_mtime = mtime
            try:
                with open(self._selfcheck_path) as f:
                    data = yaml.safe_load(f) or {}
            except (OSError, yaml.YAMLError):
                return self._selfcheck_status
            passed = bool(data.get("ok"))
            failed = [c["name"] for c in data.get("checks", []) if not c.get("passed")]
            level = DiagnosticStatus.OK if passed else DiagnosticStatus.WARN
            message = "ready" if passed else "failed: " + ", ".join(failed)
            self._selfcheck_status = self._status(
                "self_check", level, message, {"at": str(data.get("timestamp", ""))}
            )
        return self._selfcheck_status

    @staticmethod
    def _status(name, level, message, values):
        return DiagnosticStatus(
            name=name,
            level=level,
            message=message,
            hardware_id="mote",
            values=[KeyValue(key=k, value=str(v)) for k, v in values.items()],
        )


def main():
    rclpy.init()
    node = HealthMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
