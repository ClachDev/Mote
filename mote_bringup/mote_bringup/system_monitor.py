"""Host health monitor: CPU load, memory, temperature, and Pi power flags.

Publishes a diagnostic_msgs/DiagnosticArray on ``diagnostics`` at a fixed rate
so recorded bags carry the compute/power context alongside the robot data
(a loaded or brown-throttled Pi shows up as odometry stalls downstream).
Reads /proc and /sys directly — no psutil. The Raspberry Pi firmware's
``get_throttled`` bitfield is the only power telemetry available on this
robot (the USB-C power bank exposes no state of charge): bit 0 is
under-voltage now, bit 2 is throttled now, bits 16/18 the has-occurred
latches.
"""

import socket
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node

THROTTLED_PATH = Path("/sys/devices/platform/soc/soc:firmware/get_throttled")
TEMP_PATH = Path("/sys/class/thermal/thermal_zone0/temp")

UNDERVOLTAGE_NOW = 1 << 0
THROTTLED_NOW = 1 << 2
UNDERVOLTAGE_EVER = 1 << 16
THROTTLED_EVER = 1 << 18


def _cpu_times():
    fields = Path("/proc/stat").read_text().splitlines()[0].split()[1:]
    times = [int(f) for f in fields]
    idle = times[3] + times[4]  # idle + iowait
    return idle, sum(times)


def _meminfo():
    info = {}
    for line in Path("/proc/meminfo").read_text().splitlines():
        key, value = line.split(":", 1)
        info[key] = int(value.split()[0])
    return info


class SystemMonitor(Node):
    def __init__(self):
        super().__init__("system_monitor")
        period = self.declare_parameter("period", 1.0).value
        self.pub = self.create_publisher(DiagnosticArray, "diagnostics", 10)
        self.prev_cpu = _cpu_times()
        self.create_timer(period, self._tick)

    def _tick(self):
        status = DiagnosticStatus(
            name="system", hardware_id=socket.gethostname(), level=DiagnosticStatus.OK
        )
        values = {}
        warnings = []

        idle, total = _cpu_times()
        prev_idle, prev_total = self.prev_cpu
        self.prev_cpu = (idle, total)
        busy = 1.0 - (idle - prev_idle) / max(total - prev_total, 1)
        values["cpu_percent"] = f"{100 * busy:.1f}"
        load1, load5, load15 = Path("/proc/loadavg").read_text().split()[:3]
        values.update(load1=load1, load5=load5, load15=load15)

        mem = _meminfo()
        used = 1.0 - mem["MemAvailable"] / mem["MemTotal"]
        values["mem_percent"] = f"{100 * used:.1f}"

        if TEMP_PATH.exists():
            temp = int(TEMP_PATH.read_text()) / 1000
            values["cpu_temp_c"] = f"{temp:.1f}"
            if temp > 80:
                warnings.append(f"cpu {temp:.0f}C")

        if THROTTLED_PATH.exists():
            flags = int(THROTTLED_PATH.read_text(), 16)
            values["throttled_flags"] = f"{flags:#x}"
            values["undervoltage_now"] = str(bool(flags & UNDERVOLTAGE_NOW))
            values["throttled_now"] = str(bool(flags & THROTTLED_NOW))
            values["undervoltage_ever"] = str(bool(flags & UNDERVOLTAGE_EVER))
            values["throttled_ever"] = str(bool(flags & THROTTLED_EVER))
            if flags & (UNDERVOLTAGE_NOW | THROTTLED_NOW):
                status.level = DiagnosticStatus.ERROR
                warnings.append("power: under-voltage/throttled")

        if status.level == DiagnosticStatus.OK and warnings:
            status.level = DiagnosticStatus.WARN
        status.message = ", ".join(warnings) or "ok"
        status.values = [KeyValue(key=k, value=v) for k, v in values.items()]

        msg = DiagnosticArray(status=[status])
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)
        if warnings:
            self.get_logger().warning(status.message)


def main():
    rclpy.init()
    node = SystemMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
