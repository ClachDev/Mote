#!/usr/bin/env python3
"""Headless smoke test for the Mote Gazebo sim.

Run by run_sim_smoke.sh once sim_launch.py + slam_launch.py are up. Drives the
robot and asserts the whole sim stack behaves: odometry integrates commanded
motion, the simulated lidar publishes sane scans, and slam_toolbox builds a map.

Exits 0 on PASS, 1 on any failed assertion (printed as "FAIL: ...").
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan


class Verifier(Node):
    def __init__(self):
        super().__init__("sim_smoke_verifier")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.odom = None
        self.scans = []
        self.map = None
        self.create_subscription(
            Odometry, "/diff_drive_controller/odom", self.on_odom, 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        # slam_toolbox latches /map with transient-local reliable QoS
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)
        self.cmd_pub = self.create_publisher(
            TwistStamped, "/diff_drive_controller/cmd_vel", 10)

    def on_odom(self, msg):
        self.odom = msg

    def on_scan(self, msg):
        self.scans.append((time.monotonic(), msg))

    def on_map(self, msg):
        self.map = msg

    def sim_now(self):
        # node has use_sim_time=True, so this is /clock (sim) time in seconds
        return self.get_clock().now().nanoseconds / 1e9

    def drive(self, vx, wz, seconds):
        # Gate on sim time, not wall time: the sim does not run at realtime
        # (RTF varies with machine load), so a wall-clock duration would
        # translate to a variable, unpredictable distance. wall_cap is a
        # safety net so a stalled /clock can never hang the test.
        start = self.sim_now()
        wall_cap = time.monotonic() + seconds * 60 + 10
        while self.sim_now() - start < seconds:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist.linear.x = vx
            msg.twist.angular.z = wz
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() > wall_cap:
                raise AssertionError("FAIL: sim clock not advancing (drive stalled)")

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def pose(self):
        p = self.odom.pose.pose
        yaw = 2.0 * math.atan2(p.orientation.z, p.orientation.w)
        return p.position.x, p.position.y, yaw


def main():
    rclpy.init()
    node = Verifier()

    # wait for odom + scan to start flowing
    deadline = time.monotonic() + 40
    while (node.odom is None or not node.scans) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    assert node.odom is not None, "FAIL: no /diff_drive_controller/odom received"
    assert node.scans, "FAIL: no /scan received"

    x0, y0, yaw0 = node.pose()
    node.scans.clear()

    # drive forward 0.2 m/s for 3 s -> expect ~0.6 m forward
    node.drive(0.2, 0.0, 3.0)
    node.drive(0.0, 0.0, 0.5)
    x1, y1, yaw1 = node.pose()
    dist = math.hypot(x1 - x0, y1 - y0)
    print(f"forward: moved {dist:.3f} m (expected ~0.6)")
    assert 0.3 < dist < 0.9, f"FAIL: forward distance {dist:.3f} not in (0.3, 0.9)"

    # spin 1.0 rad/s for 2 s -> expect ~2 rad yaw change
    node.drive(0.0, 1.0, 2.0)
    node.drive(0.0, 0.0, 0.5)
    _, _, yaw2 = node.pose()
    dyaw = abs(math.atan2(math.sin(yaw2 - yaw1), math.cos(yaw2 - yaw1)))
    print(f"spin: rotated {dyaw:.3f} rad (expected ~2.0, wrapped)")
    assert 1.0 < dyaw, f"FAIL: yaw change {dyaw:.3f} too small"

    # scan rate + content over the drive window above. Rate is computed from
    # message header stamps (sim time) so it reflects the configured sensor
    # rate regardless of how fast the sim runs relative to wall time.
    n = len(node.scans)

    def stamp_s(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

    span = stamp_s(node.scans[-1][1]) - stamp_s(node.scans[0][1])
    rate = (n - 1) / span if span > 0 else 0.0
    scan = node.scans[-1][1]
    finite = [r for r in scan.ranges if scan.range_min < r < scan.range_max]
    print(f"scan: {n} msgs, {rate:.1f} Hz, {len(finite)}/{len(scan.ranges)} finite ranges, "
          f"min {min(finite):.2f} max {max(finite):.2f}")
    assert rate > 5.0, f"FAIL: scan rate {rate:.1f} Hz too low"
    assert len(finite) > len(scan.ranges) * 0.5, "FAIL: too few finite ranges"
    assert max(finite) < 12.0 and min(finite) > 0.05, "FAIL: ranges outside lidar spec"

    # slam_toolbox should have built a map by now; give it a few seconds to
    # process the scans accumulated during the drive
    deadline = time.monotonic() + 20
    while node.map is None and time.monotonic() < deadline:
        node.spin_for(1.0)
    assert node.map is not None, "FAIL: no /map published by slam_toolbox"
    info = node.map.info
    print(f"map: {info.width}x{info.height} @ {info.resolution:.3f} m/cell")
    assert info.width > 0 and info.height > 0, "FAIL: empty map"
    assert 0.01 < info.resolution < 0.5, f"FAIL: map resolution {info.resolution} implausible"

    print("PASS: sim smoke test")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
