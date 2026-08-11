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
at ``$MOTE_HOME/health.yaml``).

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

from mote_bringup import mote_home
from mote_bringup.sd_notify import SdNotifier

LEVEL_NAME = {
    DiagnosticStatus.OK: "OK",
    DiagnosticStatus.WARN: "DEGRADED",
    DiagnosticStatus.ERROR: "FAULT",
    DiagnosticStatus.STALE: "STALE",
}

# Statuses lifted from the shared /diagnostics into the roll-up, matched by exact
# name: system_monitor's host status, and slip_monitor's odometry-residual
# verdict. Both are first-party monitors publishing one named status. Overridable
# via health.yaml's `diagnostic_statuses`.
DIAGNOSTIC_STATUS_NAMES = ("system", "slip")

# How much a missing/stale subsystem degrades the robot summary. "info" reports
# the subsystem without degrading — for edges that are legitimately absent in a
# healthy state (map->odom exists only once a mission localises).
SEVERITY_LEVEL = {
    "critical": DiagnosticStatus.ERROR,
    "degraded": DiagnosticStatus.WARN,
    "info": DiagnosticStatus.OK,
}


def _severity_level(spec):
    """Level a missing/stale subsystem reports, from its config severity."""
    severity = spec.get("severity")
    if severity is None:
        # Back-compat with the boolean form.
        severity = "critical" if spec.get("critical") else "degraded"
    if severity not in SEVERITY_LEVEL:
        raise ValueError(f"unknown severity {severity!r} for {spec.get('name')}")
    return SEVERITY_LEVEL[severity]


def _one_line(text):
    """Collapse whitespace so a summary stays a single line.

    Third-party diagnostic messages can carry embedded newlines, which would
    otherwise shatter the one-line /health summary into several messages.
    """
    return " ".join(text.split())


def _load_config():
    default = os.path.join(
        get_package_share_directory("mote_bringup"), "config", "health.yaml"
    )
    with open(mote_home.override("health.yaml", default)) as f:
        return yaml.safe_load(f)


class _TopicWatch:
    """Freshness + rate tracker for one subscribed topic."""

    def __init__(self, spec):
        self.name = spec["name"]
        self.topic = spec["topic"]
        self.min_rate = spec.get("min_rate")
        self.timeout = spec.get("timeout", 2.0)
        self.fault_level = _severity_level(spec)
        self.last_stamp = None
        self.count = 0

    def on_msg(self, _msg):
        """Record an arrival. The payload is never read, and arrives raw."""
        self.last_stamp = time.monotonic()
        self.count += 1

    def evaluate(self, window):
        rate = self.count / window if window > 0 else 0.0
        self.count = 0
        age = None if self.last_stamp is None else time.monotonic() - self.last_stamp

        values = {"topic": self.topic, "rate_hz": f"{rate:.1f}"}
        if age is None:
            return self.fault_level, "no messages received", values
        values["age_s"] = f"{age:.1f}"
        if age > self.timeout:
            return self.fault_level, f"stale ({age:.1f}s > {self.timeout:.1f}s)", values
        if self.min_rate is not None and rate < self.min_rate:
            # A degraded rate never exceeds the subsystem's own fault level.
            level = min(DiagnosticStatus.WARN, self.fault_level)
            return level, f"slow ({rate:.1f} < {self.min_rate:.1f} Hz)", values
        return DiagnosticStatus.OK, "ok", values


class _TfWatch:
    """Freshness tracker for one TF edge."""

    def __init__(self, spec):
        self.name = spec["name"]
        self.parent = spec["parent"]
        self.child = spec["child"]
        self.timeout = spec.get("timeout", 2.0)
        self.fault_level = _severity_level(spec)

    def evaluate(self, buffer, now):
        values = {"transform": f"{self.parent}->{self.child}"}
        try:
            tf = buffer.lookup_transform(self.parent, self.child, rclpy.time.Time())
        except tf2_ros.TransformException as exc:
            values["error"] = _one_line(str(exc))[:80]
            return self.fault_level, "unavailable", values
        age = (now - rclpy.time.Time.from_msg(tf.header.stamp)).nanoseconds / 1e9
        values["age_s"] = f"{age:.1f}"
        if age > self.timeout:
            return self.fault_level, f"stale ({age:.1f}s > {self.timeout:.1f}s)", values
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
            # Subscribed raw: the watch only counts and timestamps, so the
            # callback never touches a field. These are the robot's highest-rate
            # topics — a 50 Hz JointState, two LaserScans and a ~30 fps
            # CompressedImage — and deserializing each one into a Python object
            # to learn that it arrived costs more than everything this node does
            # with it. The type is still needed: it is what selects the
            # typesupport, only the delivered object changes (bytes).
            self.create_subscription(
                msg_type, spec["topic"], watch.on_msg, 10, raw=True
            )
            self.topics.append(watch)

        self.tf_watches = [_TfWatch(s) for s in cfg.get("tf", [])]
        self.tf_buffer = None
        if self.tf_watches:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.wanted_statuses = tuple(
            cfg.get("diagnostic_statuses", DIAGNOSTIC_STATUS_NAMES)
        )
        self.forwarded = {}
        if cfg.get("subscribe_diagnostics", True):
            # Deserialized, unlike the freshness watches above: this callback
            # reads status.name and status.level. It is also the one low-rate
            # subscription here, published once per aggregation period.
            self.create_subscription(
                DiagnosticArray, "diagnostics", self._on_diagnostics, 10
            )

        self._selfcheck_path = mote_home.path("self_check_status.yaml")
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
        # Only the named first-party statuses feed the roll-up, matched exactly:
        # /diagnostics is a shared topic — controller_manager publishes its own
        # loop-jitter status there — and folding a third party's level in would
        # attribute it to one of ours. Other publishers stay visible on
        # /diagnostics itself.
        for status in msg.status:
            if status.name in self.wanted_statuses:
                self.forwarded[status.name] = status

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

        for name in self.wanted_statuses:
            status = self.forwarded.get(name)
            if status is None:
                # A monitor that is not running is simply absent, exactly as
                # before: its own liveness is not this monitor's to assert.
                continue
            statuses.append(status)
            overall = max(overall, status.level)
            if status.level >= DiagnosticStatus.WARN:
                label = "host" if name == "system" else name
                faults.append(f"{label} {_one_line(status.message)}")

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
        # One line, always: /health is meant for `ros2 topic echo` and log greps.
        summary_text = _one_line(summary_text)

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
