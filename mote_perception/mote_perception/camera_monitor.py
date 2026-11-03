"""Camera health monitor.

Subscribes to the compressed image stream and periodically logs the measured
frame rate, resolution, and format, warning when no frames arrive in an
interval. Uses rclpy + sensor_msgs only, no OpenCV/cv_bridge.

The compressed stream, not the raw one: the node counts frames and never looks
at a pixel, and at 640x480 rgb8 x 30 Hz the raw stream is ~27 MB/s deserialized
into Python for nothing. `CompressedImage.format` carries the source encoding
and the codec (e.g. "bgr8; jpeg compressed bgr8"); the resolution comes from
`camera_info`.

`camera_info` is read once rather than watched: it arrives at frame rate, and
for a node like this the cost is the wake-up, not the message, so a standing
subscription adds about a third to what counting the frames costs, to report a
value that only changes when the driver restarts. A report with no frames in it
is that restart's signature, so it re-arms the read.
Measurements: docs/tuning/2026-09-11-camera-monitor.md.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage


class CameraMonitor(Node):
    def __init__(self):
        super().__init__("camera_monitor")
        self.report_period = self.declare_parameter("report_period", 5.0).value

        self.count = 0
        self.format = None
        self.resolution = None
        self._info_sub = None

        self.create_subscription(CompressedImage, "image/compressed", self._cb, 10)
        self._arm_info()
        self.create_timer(self.report_period, self._report)

    def _cb(self, msg):
        self.count += 1
        self.format = msg.format

    def _arm_info(self):
        if self._info_sub is None:
            self._info_sub = self.create_subscription(
                CameraInfo, "camera_info", self._on_info, 1
            )

    def _on_info(self, msg):
        self.resolution = (msg.width, msg.height)
        self.destroy_subscription(self._info_sub)
        self._info_sub = None

    def _report(self):
        fps = self.count / self.report_period
        if self.count == 0:
            self.get_logger().warning(
                f"No frames received in the last {self.report_period:.0f} s"
            )
            self.resolution = None
            self._arm_info()
        else:
            size = (
                "%dx%d" % self.resolution if self.resolution else "?x? (no camera_info)"
            )
            self.get_logger().info(f"{fps:.1f} FPS, {size}, format={self.format}")
        self.count = 0


def main():
    rclpy.init()
    node = CameraMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
