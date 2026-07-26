"""Host health monitor: CPU load, memory, temperature, fan, and Pi power flags.

Publishes a diagnostic_msgs/DiagnosticArray on ``diagnostics`` at a fixed rate
so recorded bags carry the compute/power context alongside the robot data
(a loaded or brown-throttled Pi shows up as odometry stalls downstream).
Reads /proc and /sys directly — no psutil. The Raspberry Pi firmware's
``get_throttled`` bitfield is the only power telemetry available on this
robot (the USB-C power bank exposes no state of charge): bits 0-3 are the
now-active conditions, bits 16-19 their has-occurred latches. On a Pi 5 that
bitfield is reachable only through ``vcgencmd`` — the Pi 4 sysfs node
(``/sys/devices/platform/soc/soc:firmware/get_throttled``) does not exist.
The active cooler's tachometer is read from the ``pwmfan`` hwmon.
"""

import shutil
import socket
import subprocess
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node

TEMP_PATH = Path("/sys/class/thermal/thermal_zone0/temp")
HWMON_ROOT = Path("/sys/class/hwmon")

# get_throttled bit -> (diagnostic key, human label). Each condition has a
# now-bit and a has-occurred-since-boot latch 16 bits higher.
THROTTLE_FLAGS = (
    (0, "undervoltage", "under-voltage"),
    (1, "freq_capped", "arm frequency capped"),
    (2, "throttled", "throttled"),
    (3, "soft_temp_limit", "soft temperature limit"),
)
EVER_SHIFT = 16


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


def parse_throttled(output):
    """Parse ``vcgencmd get_throttled`` output (``throttled=0xe0000``)."""
    return int(output.strip().split("=", 1)[1], 16)


def read_throttled(vcgencmd):
    """Current get_throttled bitfield, or None if the firmware call failed."""
    try:
        result = subprocess.run(
            [vcgencmd, "get_throttled"],
            capture_output=True,
            text=True,
            timeout=2.0,
            check=True,
        )
        return parse_throttled(result.stdout)
    except (OSError, subprocess.SubprocessError, IndexError, ValueError):
        return None


def throttle_values(flags):
    """get_throttled bitfield -> diagnostic key/value pairs."""
    values = {"throttled_flags": f"{flags:#x}"}
    for bit, key, _ in THROTTLE_FLAGS:
        values[f"{key}_now"] = str(bool(flags & 1 << bit))
        values[f"{key}_ever"] = str(bool(flags & 1 << (bit + EVER_SHIFT)))
    return values


def throttle_warning(flags):
    """Warning text for the now-active conditions, or None if none are set."""
    active = [label for bit, _, label in THROTTLE_FLAGS if flags & 1 << bit]
    return "power: " + "/".join(active) if active else None


def find_pwmfan(root=HWMON_ROOT):
    """Directory of the active cooler's hwmon, or None if no fan is fitted."""
    try:
        candidates = sorted(root.glob("hwmon*"))
    except OSError:
        return None
    for hwmon in candidates:
        try:
            if hwmon.joinpath("name").read_text().strip() == "pwmfan":
                return hwmon
        except OSError:
            continue
    return None


def read_fan_rpm(hwmon):
    """Fan tachometer in RPM, or None if the read failed."""
    try:
        return int(hwmon.joinpath("fan1_input").read_text().strip())
    except (OSError, ValueError):
        return None


class SystemMonitor(Node):
    def __init__(self):
        super().__init__("system_monitor")
        period = self.declare_parameter("period", 1.0).value
        self.pub = self.create_publisher(DiagnosticArray, "diagnostics", 10)
        self.prev_cpu = _cpu_times()
        self.vcgencmd = shutil.which("vcgencmd")
        if self.vcgencmd is None:
            self.get_logger().info("vcgencmd not found — no throttle reporting")
        self.pwmfan = find_pwmfan()
        self.throttle_read_failed = False
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

        if self.pwmfan is not None:
            rpm = read_fan_rpm(self.pwmfan)
            if rpm is not None:
                values["fan_rpm"] = str(rpm)

        if self.vcgencmd is not None:
            flags = read_throttled(self.vcgencmd)
            if flags is None:
                if not self.throttle_read_failed:
                    self.throttle_read_failed = True
                    self.get_logger().warning("vcgencmd get_throttled failed")
            else:
                self.throttle_read_failed = False
                values.update(throttle_values(flags))
                warning = throttle_warning(flags)
                if warning:
                    status.level = DiagnosticStatus.ERROR
                    warnings.append(warning)

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
