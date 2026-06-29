"""Off-board depth -> obstacle PointCloud2 node (no torch; runs anywhere).

Subscribes the compressed camera stream, forwards each frame to the depth server
(tools/depth_server.py, in the pixi depth environment) over a socket, metrically
rescales the returned depth against the known floor plane, back-projects to 3D,
keeps points standing above the floor, and publishes them as a PointCloud2 for a
Nav2 obstacle layer. The cloud is stamped with the IMAGE capture time so Nav2
places it via tf at the moment it was seen — which is how the (off-board, ~0.6 s)
latency is absorbed without inflation or a speed cap.

Lidar stays the primary, low-latency obstacle/clearing source; this is a slow
supplementary marker for the low/thin things the 2D scan misses.
"""

import socket
import struct

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros

from mote_perception.ground_projection import GroundProjector, transform_to_matrix
from mote_perception.depth_rescale import DepthFloorRescaler


def recvall(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


class DepthObstacleNode(Node):
    def __init__(self):
        super().__init__("depth_obstacle_node")
        self.declare_parameter("server_host", "127.0.0.1")
        self.declare_parameter("server_port", 5601)
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("optical_frame", "camera_optical_link")
        self.declare_parameter("z_obstacle", 0.02)
        self.declare_parameter("z_ceiling", 1.6)
        self.declare_parameter("range_min", 0.25)
        self.declare_parameter("range_max", 3.0)
        self.declare_parameter("pixel_stride", 3)
        self.declare_parameter("socket_timeout", 2.0)

        self.base_frame = self.get_parameter("base_frame").value
        self.optical_frame = self.get_parameter("optical_frame").value
        self.z_obs = self.get_parameter("z_obstacle").value
        self.z_ceil = self.get_parameter("z_ceiling").value
        self.rmin = self.get_parameter("range_min").value
        self.rmax = self.get_parameter("range_max").value
        self.stride = self.get_parameter("pixel_stride").value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.cam_info = None
        self.proj = None
        self.rescaler = None
        self.grid = None
        self.rays_opt = None
        self.sock = None

        self.create_subscription(CameraInfo, "camera_info", self._on_info, 10)
        self.create_subscription(CompressedImage, "image/compressed", self._on_image, 5)
        self.pub = self.create_publisher(PointCloud2, "camera_obstacles", 5)
        self.get_logger().info("depth_obstacle_node up; waiting for camera_info + tf")

    def _on_info(self, msg):
        self.cam_info = msg

    def _ensure_setup(self):
        if self.proj is not None:
            return True
        if self.cam_info is None:
            return False
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.optical_frame, rclpy.time.Time()
            )
        except tf2_ros.TransformException:
            return False
        T = transform_to_matrix(tf.transform.translation, tf.transform.rotation)
        self.proj = GroundProjector.from_camera_info(self.cam_info, T)
        self.rescaler = DepthFloorRescaler(self.proj)
        u, v = np.meshgrid(np.arange(self.proj.width), np.arange(self.proj.height))
        self.grid = (u.ravel(), v.ravel())
        uv = np.column_stack(self.grid).astype(np.float64).reshape(-1, 1, 2)
        norm = cv2.undistortPoints(uv, self.proj.K, self.proj.D).reshape(-1, 2)
        self.rays_opt = np.column_stack([norm[:, 0], norm[:, 1], np.ones(len(norm))])
        self.get_logger().info(
            f"setup done: camera at {self.proj.camera_height:.3f} m; publishing obstacles"
        )
        return True

    def _drop_connection(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

    def _connect(self):
        if self.sock is not None:
            return self.sock
        host = self.get_parameter("server_host").value
        port = self.get_parameter("server_port").value
        timeout = float(self.get_parameter("socket_timeout").value)
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(timeout)
            s.connect((host, port))
            self.sock = s
            self.get_logger().info(f"connected to depth server {host}:{port}")
        except OSError as e:
            self.get_logger().warn(f"depth server unavailable ({e}); skipping frame")
            self.sock = None
        return self.sock

    def _infer(self, jpeg):
        s = self._connect()
        if s is None:
            return None
        try:
            s.sendall(struct.pack(">I", len(jpeg)) + jpeg)
            hdr = recvall(s, 8)
            if hdr is None:
                raise ConnectionError("server closed")
            h, w = struct.unpack(">II", hdr)
            body = recvall(s, h * w * 4)
            if body is None:
                raise ConnectionError("server closed mid-depth")
            depth = np.frombuffer(body, np.float32).reshape(h, w)
            return depth
        except (OSError, ConnectionError) as e:
            self.get_logger().warn(f"inference failed ({e}); will reconnect")
            self._drop_connection()
            return None

    def _on_image(self, msg):
        if not self._ensure_setup():
            return
        depth = self._infer(bytes(msg.data))
        if depth is None:
            return
        depth_corr, (a, b, frac) = self.rescaler.rescale(depth)
        if frac < 0.4:
            self.get_logger().warn(
                f"low floor-fit inliers ({frac:.0%}); skipping frame"
            )
            return

        d = depth_corr.ravel()
        pts = (self.rays_opt * d[:, None]) @ self.proj.R.T + self.proj.C
        bx, by, bz = pts[:, 0], pts[:, 1], pts[:, 2]
        rng = np.hypot(bx, by)
        keep = (
            (bz > self.z_obs)
            & (bz < self.z_ceil)
            & (rng > self.rmin)
            & (rng < self.rmax)
        )
        cloud = pts[keep][:: self.stride].astype(np.float32)

        header = msg.header
        header.frame_id = (
            self.base_frame
        )  # cloud is in the base frame, stamped at capture
        self.pub.publish(point_cloud2.create_cloud_xyz32(header, cloud))

        latency_ms = (
            self.get_clock().now() - rclpy.time.Time.from_msg(msg.header.stamp)
        ).nanoseconds / 1e6
        self.get_logger().info(
            f"obstacles: {len(cloud)} pts  fit_inliers {frac:.0%}  "
            f"stamp-to-publish {latency_ms:.0f} ms",
            throttle_duration_sec=2.0,
        )


def main():
    rclpy.init()
    node = DepthObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
