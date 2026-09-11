#!/usr/bin/env python3
"""A synthetic stand-in for the robot's camera driver, for weighing its consumers.

Publishes what `v4l2_camera` publishes, at its rate and sizes: `/image_raw`
(640x480 rgb8, 921,600 bytes), `/image_raw/compressed` (a 40 KB stand-in for the
JPEG, format string as `compressed_image_transport` writes it) and `/camera_info`,
all stamped alike, at 30 Hz. Nothing is decoded by anything under test, so the
pixels are zeros; only arrival rate and payload size reach a subscriber.
"""

import argparse

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

WIDTH, HEIGHT = 640, 480
JPEG_BYTES = 40_000


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--rate", type=float, default=30.0)
    args = ap.parse_args()

    rclpy.init()
    node = Node("camera_stream")
    raw_pub = node.create_publisher(Image, "/image_raw", 10)
    jpeg_pub = node.create_publisher(CompressedImage, "/image_raw/compressed", 10)
    info_pub = node.create_publisher(CameraInfo, "/camera_info", 10)

    raw = Image(
        height=HEIGHT,
        width=WIDTH,
        encoding="rgb8",
        step=WIDTH * 3,
        data=bytes(WIDTH * HEIGHT * 3),
    )
    jpeg = CompressedImage(format="rgb8; jpeg compressed bgr8", data=bytes(JPEG_BYTES))
    info = CameraInfo(height=HEIGHT, width=WIDTH, distortion_model="plumb_bob")
    for msg in (raw, jpeg, info):
        msg.header.frame_id = "camera_optical_link"

    def tick():
        stamp = node.get_clock().now().to_msg()
        for msg, pub in ((raw, raw_pub), (jpeg, jpeg_pub), (info, info_pub)):
            msg.header.stamp = stamp
            pub.publish(msg)

    node.create_timer(1.0 / args.rate, tick)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
