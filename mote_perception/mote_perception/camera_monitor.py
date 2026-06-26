"""Camera health monitor.

Subscribes to an image stream and periodically logs the measured frame rate,
resolution, and encoding, warning when no frames arrive in an interval. Uses
rclpy + sensor_msgs only, no OpenCV/cv_bridge.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraMonitor(Node):
    def __init__(self):
        super().__init__("camera_monitor")
        self.report_period = self.declare_parameter("report_period", 5.0).value

        self.count = 0
        self.last_msg = None

        self.create_subscription(Image, "image", self._cb, 10)
        self.create_timer(self.report_period, self._report)

    def _cb(self, msg):
        self.count += 1
        self.last_msg = msg

    def _report(self):
        fps = self.count / self.report_period
        if self.count == 0 or self.last_msg is None:
            self.get_logger().warn(
                f"No frames received in the last {self.report_period:.0f} s"
            )
        else:
            m = self.last_msg
            self.get_logger().info(
                f"{fps:.1f} FPS, {m.width}x{m.height}, encoding={m.encoding}"
            )
        self.count = 0


def main():
    rclpy.init()
    node = CameraMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
