"""Wheel-slip / stuck-robot monitor, read off the odometry the robot already has.

kinematic_icp takes wheel odometry as its prior and corrects it against the scan,
so the correction *is* a measurement of how wrong the wheels were. This node
compares the two over a sliding window and reports what their disagreement means:

* ``slip``      — the wheels claim travel the lidar did not see. Wheels spinning
                  on a slippery floor, or a robot wedged against something.
* ``stuck``     — motion is commanded and neither source reports any.
* ``icp_fault`` — the lidar pose moved in a way the drive cannot produce. Slip
                  makes the *wheels* over-read, never the lidar, so this is a
                  scan-match excursion (or the robot being moved by hand).

It publishes:

* ``diagnostics`` (``diagnostic_msgs/DiagnosticArray``) — one status named
  ``slip``, folded into the robot summary by ``health_monitor`` exactly as
  ``system_monitor``'s host status is.
* ``slip/residual`` (``geometry_msgs/TwistStamped``) — the raw residual, so it
  can be recorded, plotted in Foxglove and re-thresholded later without
  re-deriving it. ``linear.x`` is the speed residual (wheel minus lidar, m/s),
  ``linear.y`` the speed the comparison is relative to, and ``angular.z`` the yaw
  rate residual, which is *reported but never thresholded* — see
  ``odom_residual`` for why.

The estimator and thresholds are shared with ``tools/slip_replay.py``, which is
what set them from recorded bags: a threshold calibrated offline only means
something if the robot computes the same number.
"""

import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

import tf2_ros

from mote_bringup import mote_home
from mote_bringup.odom_residual import (
    ICP_FAULT,
    OK,
    SLIP,
    STUCK,
    UNKNOWN,
    ResidualEstimator,
    Thresholds,
    VerdictFilter,
    classify,
    yaw_of_quat,
)

STATUS_NAME = "slip"

# A raised verdict degrades the robot but never faults it: every one of these is
# a reason to stop and re-plan, not a reason to refuse to drive, and a monitor
# that can halt the robot on a threshold is a worse failure than the slip.
LEVEL = {
    OK: DiagnosticStatus.OK,
    UNKNOWN: DiagnosticStatus.OK,
    SLIP: DiagnosticStatus.WARN,
    STUCK: DiagnosticStatus.WARN,
    ICP_FAULT: DiagnosticStatus.WARN,
}


def load_config():
    """Thresholds from config/slip.yaml, overridable per robot in $MOTE_HOME."""
    default = os.path.join(
        get_package_share_directory("mote_bringup"), "config", "slip.yaml"
    )
    with open(mote_home.override("slip.yaml", default)) as f:
        cfg = yaml.safe_load(f) or {}
    with open(
        os.path.join(
            get_package_share_directory("mote_description"), "config", "robot.yaml"
        )
    ) as f:
        max_wheel_speed = float(yaml.safe_load(f)["max_wheel_speed"])

    fields = {k: v for k, v in cfg.items() if k in Thresholds.__dataclass_fields__}
    tolerance = float(cfg.get("max_body_speed_tolerance", 1.15))
    thresholds = Thresholds(**fields).with_max_wheel_speed(max_wheel_speed, tolerance)
    return thresholds, float(cfg.get("rate", 5.0))


class SlipMonitor(Node):
    def __init__(self):
        super().__init__("slip_monitor")
        self.thresholds, rate = load_config()
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_footprint").value
        # Nothing has been commanded yet, so stuck is not yet distinguishable
        # from parked; classify() is told so by a None command.
        self.command = None
        self.command_stamp = None

        self.estimator = ResidualEstimator(self.thresholds)
        self.filter = VerdictFilter(self.thresholds)

        self.create_subscription(
            Odometry, "/diff_drive_controller/odom", self._on_odom, 20
        )
        self.create_subscription(
            TwistStamped, "/diff_drive_controller/cmd_vel", self._on_command, 10
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.diagnostics = self.create_publisher(DiagnosticArray, "diagnostics", 10)
        self.residual_pub = self.create_publisher(TwistStamped, "slip/residual", 10)

        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"watching wheel-vs-lidar odometry over a {self.thresholds.window:.1f}s "
            f"window (slip > {self.thresholds.slip_speed:.3f} m/s and "
            f"{100 * self.thresholds.slip_fraction:.0f}%)"
        )

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.estimator.add_wheel(
            self._seconds(msg.header), p.x, p.y, yaw_of_quat(q.x, q.y, q.z, q.w)
        )

    def _on_command(self, msg):
        self.command = (msg.twist.linear.x, msg.twist.angular.z)
        self.command_stamp = self._now()

    @staticmethod
    def _seconds(header):
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _pull_icp(self):
        """Latest odom->base transform into the estimator.

        Read from TF rather than a topic because kinematic_icp's correction is
        published only as a transform. Sampling the *latest available* one is why
        the estimator carries a staleness guard: a stalled corrector would
        otherwise keep handing back the same pose.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, rclpy.time.Time()
            )
        except tf2_ros.TransformException:
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        self.estimator.add_icp(
            self._seconds(tf.header), t.x, t.y, yaw_of_quat(q.x, q.y, q.z, q.w)
        )

    def _command_now(self):
        """The current command, or None if it is old enough to be meaningless.

        A command that stopped arriving is not a command: DiffDriveController's
        own ``cmd_vel_timeout`` has already zeroed the wheels, so holding the last
        value would report a stuck robot that is simply parked.
        """
        if self.command is None or self.command_stamp is None:
            return None
        if self._now() - self.command_stamp > self.thresholds.max_lag:
            return None
        return self.command

    def _tick(self):
        self._pull_icp()
        now = self._now()
        residual = self.estimator.residual(now=now)
        raw = classify(
            residual, self.thresholds, self._command_now(), self.estimator.reason
        )
        verdict = self.filter.update(now, raw)

        values = {"state": verdict.state, "window_s": f"{self.thresholds.window:.1f}"}
        if residual is not None:
            values.update(
                speed_residual=f"{residual.speed_residual:+.4f}",
                relative=f"{residual.relative:+.3f}",
                yaw_rate_residual=f"{residual.yaw_rate_residual:+.4f}",
                wheel_speed=f"{residual.wheel_speed:.3f}",
                icp_speed=f"{residual.icp_speed:.3f}",
            )
            self._publish_residual(residual)

        status = DiagnosticStatus(
            name=STATUS_NAME,
            level=LEVEL[verdict.state],
            message=verdict.detail,
            hardware_id="mote",
            values=[KeyValue(key=k, value=v) for k, v in values.items()],
        )
        msg = DiagnosticArray(status=[status])
        msg.header.stamp = self.get_clock().now().to_msg()
        self.diagnostics.publish(msg)

        if verdict.state not in (OK, UNKNOWN):
            self.get_logger().warning(
                f"{verdict.state}: {verdict.detail}", throttle_duration_sec=5.0
            )

    def _publish_residual(self, residual):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.twist.linear.x = residual.speed_residual
        msg.twist.linear.y = residual.scale
        msg.twist.angular.z = residual.yaw_rate_residual
        self.residual_pub.publish(msg)


def main():
    rclpy.init()
    node = SlipMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
