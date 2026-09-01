#!/usr/bin/env python3
"""Run two health monitors side by side and diff what they publish.

The C++ monitor replaced a Python one, and no bit-identity was available the way
it was for ``OdomTfRelay``: the two count arrivals against their own clocks, so
``rate_hz`` and ``age_s`` differ by sampling jitter however correct both are.
What must be identical is everything else — which subsystems are reported, in
what order, at what level, with what message and with which values — and how
often. This drives both builds from one set of synthetic publishers, at the same
instant, and reports where they disagree.

At the same instant is the point. The two are compared against *the same*
arrival stream rather than against two runs of one, so a scan that stutters is
seen by both and cannot be mistaken for a difference between them.

    python mote_health/test/compare_monitors.py \\
        --a "ros2 run mote_health health_monitor" \\
        --b "python -m mote_bringup.health_monitor" \\
        --duration 30

Each command is given its own output topics and its own node name, so both can
run on one graph. The inputs walk through the states worth comparing: healthy,
one critical topic gone (stale -> FAULT), a forwarded status degraded, and
recovery.

``--hold SECONDS`` publishes those same inputs steadily and starts no monitor,
which is the workload half of a paired CPU measurement (``pixi run node-cpu``,
and ``docs/tuning/2026-08-11-monitor-cpu.md`` for why a pair): what a monitor
costs is a function of the rates arriving at it, and a robot's real rates drift
with its servo bus between one run and the next.
"""

from __future__ import annotations

import argparse
import json
import os
import random
import re
import shlex
import statistics
import subprocess
import sys
import time
from pathlib import Path

os.environ.setdefault("ROS_DOMAIN_ID", str(random.randint(64, 200)))
os.environ.setdefault("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")

import rclpy  # noqa: E402
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus  # noqa: E402
from geometry_msgs.msg import TransformStamped  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import CompressedImage, JointState, LaserScan  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from tf2_ros import TransformBroadcaster  # noqa: E402

# The real robot's rates, so the comparison exercises the same arrival volume
# the monitor is measured against (docs/tuning/2026-08-11-monitor-cpu.md).
RATES = {
    "scan": 10.0,
    "scan_filtered": 10.0,
    "joint_states": 50.0,
    "camera": 29.0,
    "tf": 50.0,
}

# And its payload sizes, because a monitor that never opens a message still
# copies it out of the middleware. Worth ~0.1 points of a core against a 4 KB
# stand-in for the JPEG (docs/tuning/2026-09-01-health-monitor-cpp.md §4) — small,
# and the point of setting them is that the figure is known rather than assumed.
SCAN_RANGES = 720
JPEG_BYTES = 40_000
ARM_AND_WHEEL_JOINTS = [
    "left_wheel_joint",
    "right_wheel_joint",
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

CONFIG = """\
period: 1.0
topics:
  - name: scan
    topic: /scan
    type: sensor_msgs/msg/LaserScan
    min_rate: 5.0
    timeout: 2.0
    severity: critical
  - name: scan_filtered
    topic: /scan_filtered
    type: sensor_msgs/msg/LaserScan
    min_rate: 5.0
    timeout: 2.0
    severity: critical
  - name: joint_states
    topic: /joint_states
    type: sensor_msgs/msg/JointState
    min_rate: 5.0
    timeout: 2.0
    severity: critical
  - name: camera
    topic: /image_raw/compressed
    type: sensor_msgs/msg/CompressedImage
    min_rate: 5.0
    timeout: 5.0
    severity: degraded
tf:
  - name: odometry
    parent: odom
    child: base_footprint
    timeout: 2.0
    severity: critical
  - name: localization
    parent: map
    child: odom
    timeout: 5.0
    severity: info
subscribe_diagnostics: true
diagnostic_statuses:
  - system
  - slip
"""

SELF_CHECK = """\
ok: true
timestamp: '2026-09-01T09:00:00+00:00'
checks:
- name: servos
  passed: true
  severity: CRITICAL
  detail: ok
"""

# A measured rate or age is the one thing the two cannot agree on exactly, so
# every number with a decimal point is blanked before shapes are compared. The
# numbers themselves are reported separately.
NUMBER = re.compile(r"-?\d+\.\d+")

# The two values that are measurements. Everything else — the watched topic, the
# TF edge, the reason a lookup failed, the pre-flight timestamp, the subsystem
# count — has to match character for character, which is what catches a message
# that reads the same and says something else.
MEASURED = ("rate_hz", "age_s")


def _level(status: DiagnosticStatus) -> int:
    """DiagnosticStatus.level, which rclpy hands over as a one-byte bytes."""
    return status.level[0] if isinstance(status.level, bytes) else int(status.level)


def shape(arr: DiagnosticArray) -> list:
    """What must be identical: names, order, levels, messages, values."""
    return [
        [
            status.name,
            _level(status),
            NUMBER.sub("#", status.message),
            status.hardware_id,
            [[kv.key, "#" if kv.key in MEASURED else kv.value] for kv in status.values],
        ]
        for status in arr.status
    ]


def rates(arr: DiagnosticArray) -> dict:
    """The measured numbers, which are compared with a tolerance instead."""
    out = {}
    for status in arr.status:
        for kv in status.values:
            if kv.key == "rate_hz":
                out[status.name] = float(kv.value)
    return out


class Inputs(Node):
    """Every publisher the config watches, plus the phase schedule."""

    def __init__(self):
        super().__init__("compare_inputs")
        self.scan = self.create_publisher(LaserScan, "/scan", 10)
        self.scan_filtered = self.create_publisher(LaserScan, "/scan_filtered", 10)
        self.joints = self.create_publisher(JointState, "/joint_states", 10)
        self.camera = self.create_publisher(
            CompressedImage, "/image_raw/compressed", 10
        )
        self.diagnostics = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.tf = TransformBroadcaster(self)

        self.scan_running = True
        self.system_level = DiagnosticStatus.OK
        # Built once: allocating 40 KB 29 times a second would make the input
        # process the expensive thing in the run.
        self._jpeg = bytes(JPEG_BYTES)

        for name, rate in RATES.items():
            self.create_timer(1.0 / rate, getattr(self, f"_pub_{name}"))
        self.create_timer(1.0, self._pub_diagnostics)

    def _stamp(self):
        return self.get_clock().now().to_msg()

    def _scan_msg(self):
        msg = LaserScan()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = "laser"
        msg.ranges = [1.0] * SCAN_RANGES
        msg.intensities = [0.0] * SCAN_RANGES
        return msg

    def _pub_scan(self):
        if not self.scan_running:
            return
        self.scan.publish(self._scan_msg())

    def _pub_scan_filtered(self):
        self.scan_filtered.publish(self._scan_msg())

    def _pub_joint_states(self):
        msg = JointState()
        msg.header.stamp = self._stamp()
        msg.name = ARM_AND_WHEEL_JOINTS
        msg.position = [0.0] * len(ARM_AND_WHEEL_JOINTS)
        msg.velocity = [0.0] * len(ARM_AND_WHEEL_JOINTS)
        self.joints.publish(msg)

    def _pub_camera(self):
        msg = CompressedImage()
        msg.header.stamp = self._stamp()
        msg.format = "jpeg"
        msg.data = self._jpeg
        self.camera.publish(msg)

    def _pub_tf(self):
        tf = TransformStamped()
        tf.header.stamp = self._stamp()
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_footprint"
        tf.transform.rotation.w = 1.0
        self.tf.sendTransform(tf)

    def _pub_diagnostics(self):
        arr = DiagnosticArray()
        arr.header.stamp = self._stamp()
        for name, message in (
            ("system", "cpu 12%\ntemp 48C"),
            ("slip", "residual 0.01 m"),
        ):
            status = DiagnosticStatus()
            status.name = name
            status.level = (
                self.system_level if name == "system" else DiagnosticStatus.OK
            )
            status.message = message
            status.hardware_id = "auldbot"
            arr.status.append(status)
        self.diagnostics.publish(arr)


class Recorder(Node):
    def __init__(self, labels):
        super().__init__("compare_recorder")
        self.summaries = {label: [] for label in labels}
        self.aggregates = {label: [] for label in labels}
        for label in labels:
            self.create_subscription(
                String, f"/{label}_health", self._summary(label), 10
            )
            self.create_subscription(
                DiagnosticArray, f"/{label}_agg", self._aggregate(label), 10
            )

    def _summary(self, label):
        def cb(msg):
            self.summaries[label].append((time.monotonic(), msg.data))

        return cb

    def _aggregate(self, label):
        def cb(msg):
            self.aggregates[label].append((time.monotonic(), msg))

        return cb


def spawn(command: str, label: str, env: dict) -> subprocess.Popen:
    argv = shlex.split(command) + [
        "--ros-args",
        "-r",
        f"__node:=health_monitor_{label}",
        "-r",
        f"health:={label}_health",
        "-r",
        f"diagnostics_agg:={label}_agg",
    ]
    return subprocess.Popen(argv, env=env, start_new_session=True)


def cadence(stamps: list[float]) -> dict:
    gaps = [b - a for a, b in zip(stamps, stamps[1:])]
    if not gaps:
        return {}
    return {
        "publishes": len(stamps),
        "mean_gap_s": round(statistics.mean(gaps), 3),
        "max_gap_s": round(max(gaps), 3),
    }


def write_config(home: Path) -> Path:
    """Both builds read one file, so a difference cannot be a config difference."""
    home.mkdir(parents=True, exist_ok=True)
    (home / "health.yaml").write_text(CONFIG)
    (home / "self_check_status.yaml").write_text(SELF_CHECK)
    return home


def hold(args) -> int:
    """Publish the inputs steadily and start nothing else."""
    write_config(Path(args.mote_home))
    rclpy.init()
    inputs = Inputs()
    # The domain is printed because in this mode the monitors are started by
    # somebody else: a monitor on domain 0 while these publish on 137 reports
    # every subsystem missing and costs nothing, which reads as a fast monitor.
    print(
        f"publishing {RATES} for {args.hold:.0f}s\n"
        f"  ROS_DOMAIN_ID={os.environ['ROS_DOMAIN_ID']} MOTE_HOME={args.mote_home}",
        flush=True,
    )
    deadline = time.monotonic() + args.hold
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(inputs, timeout_sec=0.005)
    except KeyboardInterrupt:
        pass
    finally:
        inputs.destroy_node()
        rclpy.try_shutdown()
    return 0


def run(args) -> int:
    home = write_config(Path(args.mote_home))
    env = dict(os.environ, MOTE_HOME=str(home))

    rclpy.init()
    inputs = Inputs()
    recorder = Recorder(["a", "b"])
    procs = {
        "a": spawn(args.a, "a", env),
        "b": spawn(args.b, "b", env),
    }

    # Healthy, one critical topic gone, a forwarded status degraded, recovered.
    quarter = args.duration / 4.0
    phases = [
        (quarter, lambda: None),
        (quarter, lambda: setattr(inputs, "scan_running", False)),
        (quarter, lambda: setattr(inputs, "system_level", DiagnosticStatus.WARN)),
        (
            quarter,
            lambda: (
                setattr(inputs, "scan_running", True),
                setattr(inputs, "system_level", DiagnosticStatus.OK),
            ),
        ),
    ]
    try:
        for seconds, enter in phases:
            enter()
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                rclpy.spin_once(inputs, timeout_sec=0.005)
                rclpy.spin_once(recorder, timeout_sec=0.0)
    finally:
        for proc in procs.values():
            # `ros2 run` hands the node to init on a plain terminate; the whole
            # session is the reapable scope (mote_bringup/sweep_orphans.py).
            try:
                os.killpg(os.getpgid(proc.pid), 15)
            except ProcessLookupError:
                pass
            proc.wait(timeout=10)
        inputs.destroy_node()
        recorder.destroy_node()
        rclpy.try_shutdown()

    return report(recorder, args)


def report(recorder: Recorder, args) -> int:
    result = {"a": args.a, "b": args.b, "duration_s": args.duration}
    problems = []

    for label in ("a", "b"):
        if not recorder.aggregates[label]:
            problems.append(f"{label} published nothing on /{label}_agg")
    if problems:
        print("\n".join(problems), file=sys.stderr)
        return 1

    # Cadence: the same number of publishes at the same interval.
    for label in ("a", "b"):
        result[f"{label}_cadence"] = cadence([t for t, _ in recorder.aggregates[label]])
        result[f"{label}_health_cadence"] = cadence(
            [t for t, _ in recorder.summaries[label]]
        )

    counts = [result[f"{label}_cadence"]["publishes"] for label in ("a", "b")]
    if abs(counts[0] - counts[1]) > 1:
        problems.append(f"cadence differs: {counts[0]} vs {counts[1]} publishes")

    # Content: the set of distinct shapes each build produced. The first tick is
    # dropped from both — a monitor's opening window is however long it took the
    # process to start, and the two processes do not start together.
    shapes = {}
    summaries = {}
    for label in ("a", "b"):
        seen, order = set(), []
        for _, arr in recorder.aggregates[label][1:]:
            key = json.dumps(shape(arr))
            if key not in seen:
                seen.add(key)
                order.append(json.loads(key))
        shapes[label] = order
        summaries[label] = sorted(
            {NUMBER.sub("#", text) for _, text in recorder.summaries[label][1:]}
        )
        result[f"{label}_shapes"] = order
        result[f"{label}_summaries"] = summaries[label]

    if summaries["a"] != summaries["b"]:
        problems.append("the /health summaries differ")
    canonical = {
        label: sorted(json.dumps(s) for s in shapes[label]) for label in ("a", "b")
    }
    if canonical["a"] != canonical["b"]:
        problems.append("the /diagnostics_agg shapes differ")

    # The measured numbers, which cannot match exactly and must still agree.
    for label in ("a", "b"):
        per_subsystem = {}
        for _, arr in recorder.aggregates[label][1:]:
            for name, rate in rates(arr).items():
                per_subsystem.setdefault(name, []).append(rate)
        result[f"{label}_rate_hz_mean"] = {
            name: round(statistics.mean(values), 2)
            for name, values in per_subsystem.items()
        }
    for name, mean_a in result["a_rate_hz_mean"].items():
        mean_b = result["b_rate_hz_mean"].get(name)
        if mean_b is None or abs(mean_a - mean_b) > args.rate_tolerance:
            problems.append(f"{name} rate differs: {mean_a} vs {mean_b}")

    result["problems"] = problems
    result["verdict"] = "equivalent" if not problems else "differs"
    text = json.dumps(result, indent=2)
    print(text)
    if args.out:
        Path(args.out).write_text(text + "\n")
    return 0 if not problems else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--a", default="ros2 run mote_health health_monitor")
    ap.add_argument("--b", default="", help="the build to compare against")
    ap.add_argument("--duration", type=float, default=40.0)
    ap.add_argument(
        "--hold",
        type=float,
        default=0.0,
        help="publish the inputs for this many seconds and start no monitor",
    )
    ap.add_argument("--rate-tolerance", type=float, default=1.0, help="Hz")
    ap.add_argument(
        "--mote-home",
        default="/tmp/mote_health_compare",
        help="MOTE_HOME both builds read health.yaml from",
    )
    ap.add_argument("--out", default="", help="write the JSON report here too")
    args = ap.parse_args()
    if args.hold > 0:
        return hold(args)
    if not args.b:
        ap.error("--b is required unless --hold is given")
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
