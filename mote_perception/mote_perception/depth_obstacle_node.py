"""Off-board depth -> obstacle PointCloud2 node (no torch; runs anywhere).

Subscribes the compressed camera stream, forwards each frame to the depth server
(tools/depth_server.py, in the pixi depth environment) over a socket, metrically
rescales the returned depth (against time-synced lidar range returns, falling back
to the floor plane), back-projects to 3D, keeps points standing above the floor,
and publishes them as a PointCloud2 for a Nav2 obstacle layer. The cloud is
stamped with the IMAGE capture time so Nav2
places it via tf at the moment it was seen — which is how the (off-board, ~0.6 s)
latency is absorbed without inflation or a speed cap.

Lidar stays the primary, low-latency obstacle/clearing source; this is a slow
supplementary marker for the low/thin things the 2D scan misses.
"""

import socket
import struct
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import (
    CameraInfo,
    CompressedImage,
    Image,
    LaserScan,
    PointCloud2,
    PointField,
)
import tf2_ros

from mote_perception.ground_projection import GroundProjector, transform_to_matrix
from mote_perception.depth_rescale import DepthFloorRescaler, apply_affine_disparity
from mote_perception.lidar_rescale import LidarDepthRescaler


def make_xyzrgb_cloud(header, xyz, bgr):
    """Pack an Nx3 float xyz array and Nx3 uint8 BGR array into an XYZRGB PointCloud2.

    The colour is the per-point camera pixel, packed into a FLOAT32 `rgb` field (the
    PCL convention RViz's RGB8 transformer reads): the uint32 0x00RRGGBB bit pattern is
    stored verbatim, not numerically cast, so RViz shows the cloud in true camera colour.
    """
    n = len(xyz)
    data = np.zeros(
        n,
        dtype=[
            ("x", np.float32),
            ("y", np.float32),
            ("z", np.float32),
            ("rgb", np.uint32),
        ],
    )
    data["x"], data["y"], data["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    data["rgb"] = (
        bgr[:, 2].astype(np.uint32) << 16
        | bgr[:, 1].astype(np.uint32) << 8
        | bgr[:, 0].astype(np.uint32)
    )
    msg = PointCloud2()
    msg.header = header
    msg.height, msg.width = 1, n
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * n
    msg.is_dense = True
    msg.data = data.tobytes()
    return msg


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
        self.declare_parameter("publish_debug", True)
        # "auto" = lidar-anchored scale, holding the last good fit when a scan can't
        # constrain it (floor fit only until the first lidar fit, or when forced);
        # "lidar" / "floor" force one source.
        self.declare_parameter("rescale_source", "auto")
        self.declare_parameter("scan_topic", "/scan_filtered")
        self.declare_parameter("scan_max_dt", 0.2)

        self.base_frame = self.get_parameter("base_frame").value
        self.optical_frame = self.get_parameter("optical_frame").value
        self.z_obs = self.get_parameter("z_obstacle").value
        self.z_ceil = self.get_parameter("z_ceiling").value
        self.rmin = self.get_parameter("range_min").value
        self.rmax = self.get_parameter("range_max").value
        self.stride = self.get_parameter("pixel_stride").value
        self.source = self.get_parameter("rescale_source").value
        self.scan_max_dt = float(self.get_parameter("scan_max_dt").value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.cam_info = None
        self.proj = None
        self.rescaler = None
        self.lidar_rescaler = None
        self.scans = deque(maxlen=50)
        self.last_ab = None
        self.diag = ""
        self.grid = None
        self.rays_opt = None
        self.sock = None

        self.create_subscription(CameraInfo, "camera_info", self._on_info, 10)
        self.create_subscription(CompressedImage, "image/compressed", self._on_image, 5)
        if self.source != "floor":
            scan_topic = self.get_parameter("scan_topic").value
            # Own callback group so scans keep buffering on another thread while the
            # ~0.5 s blocking inference runs in _on_image (else the buffer goes stale
            # and no scan lands near an image's capture stamp).
            self.create_subscription(
                LaserScan,
                scan_topic,
                self._on_scan,
                10,
                callback_group=MutuallyExclusiveCallbackGroup(),
            )
        self.pub = self.create_publisher(PointCloud2, "camera_obstacles", 5)
        self.debug = self.get_parameter("publish_debug").value
        if self.debug:
            self.depth_pub = self.create_publisher(Image, "camera_depth", 5)
            self.depth_raw_pub = self.create_publisher(Image, "camera_depth_raw", 5)
            self.full_cloud_pub = self.create_publisher(
                PointCloud2, "camera_cloud_full", 5
            )
        self.get_logger().info("depth_obstacle_node up; waiting for camera_info + tf")

    def _publish_depth_image(self, depth, stamp, pub):
        img = Image()
        img.header.stamp = stamp
        img.header.frame_id = self.optical_frame
        img.height, img.width = depth.shape
        img.encoding = "32FC1"
        img.is_bigendian = 0
        img.step = img.width * 4
        img.data = np.ascontiguousarray(depth, dtype=np.float32).tobytes()
        pub.publish(img)

    def _on_info(self, msg):
        self.cam_info = msg

    def _on_scan(self, msg):
        stamp = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds
        self.scans.append((stamp, msg))

    def _nearest_scan(self, stamp):
        """Buffered scan nearest the capture stamp and its |dt| in ms (no cutoff)."""
        scans = list(self.scans)  # snapshot: _on_scan appends from another thread
        if not scans:
            return None, None
        target = rclpy.time.Time.from_msg(stamp).nanoseconds
        best_stamp, best = min(scans, key=lambda s: abs(s[0] - target))
        return best, abs(best_stamp - target) / 1e6

    def _ensure_lidar(self):
        """Build the lidar rescaler once the scan frame and intrinsics are known."""
        if self.lidar_rescaler is not None:
            return True
        if not self.scans or self.cam_info is None:
            return False
        frame = self.scans[-1][1].header.frame_id
        try:
            tf = self.tf_buffer.lookup_transform(
                self.optical_frame, frame, rclpy.time.Time()
            )
        except tf2_ros.TransformException:
            return False
        T = transform_to_matrix(tf.transform.translation, tf.transform.rotation)
        self.lidar_rescaler = LidarDepthRescaler(self.cam_info.k, self.cam_info.d, T)
        self.get_logger().info(f"lidar rescaler ready (scan frame {frame})")
        return True

    def _rescale(self, depth, stamp):
        """Metric-rescale the depth map, preferring lidar.

        Lidar scale when this scan constrains it; otherwise hold the last good lidar
        (a, b) rather than fall back to the floor fit, which is the very method the
        lidar replaces — falling back to it would reintroduce the over-range exactly
        when lidar can't help (e.g. facing a flat wall, no depth spread). The floor
        fit is used only before any lidar fit has succeeded, or when forced.

        Returns (corrected_depth, (a, b, frac), source) or (None, None, source).
        """
        self.diag = ""
        if self.source != "floor" and self._ensure_lidar():
            scan, dt_ms = self._nearest_scan(stamp)
            if scan is not None and dt_ms <= self.scan_max_dt * 1e3:
                out = self.lidar_rescaler.rescale(depth, scan)
                self.diag = (
                    f"  scans {len(self.scans)} dt {dt_ms:.0f}ms "
                    f"pairs {self.lidar_rescaler.last_npairs}"
                )
                if out is not None:
                    self.last_ab = out[1][:2]
                    return (*out, "lidar")
            else:
                self.diag = f"  scans {len(self.scans)} dt {dt_ms:.0f}ms (no match)"
            if self.last_ab is not None:
                a, b = self.last_ab
                return apply_affine_disparity(depth, a, b), (a, b, float("nan")), "hold"
            if self.source == "lidar":
                return None, None, "lidar"
        return (*self.rescaler.rescale(depth), "floor")

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

    def _subscribed(self, pub):
        return pub is not None and pub.get_subscription_count() > 0

    def _on_image(self, msg):
        if not self._ensure_setup():
            return
        # Lazy publishing: skip everything — including the ~0.6 s inference — unless an
        # output is actually subscribed, and gate each stage on its own consumer, so a
        # view only costs work while it is open in RViz (or Nav2 is driving the cloud).
        obs = self._subscribed(self.pub)
        raw = self.debug and self._subscribed(self.depth_raw_pub)
        cor = self.debug and self._subscribed(self.depth_pub)
        full = self.debug and self._subscribed(self.full_cloud_pub)
        if not (obs or raw or cor or full):
            return

        jpeg = bytes(msg.data)
        depth = self._infer(jpeg)
        if depth is None:
            return
        if raw:
            self._publish_depth_image(depth, msg.header.stamp, self.depth_raw_pub)
        if not (obs or cor or full):  # only the raw map was wanted
            return

        depth_corr, ab, source = self._rescale(depth, msg.header.stamp)
        if depth_corr is None:
            self.get_logger().warn(
                "no lidar scale this frame; skipping", throttle_duration_sec=2.0
            )
            return
        frac = ab[2]
        if cor:
            self._publish_depth_image(depth_corr, msg.header.stamp, self.depth_pub)
        if frac < 0.4:
            self.get_logger().warn(
                f"low {source}-fit inliers ({frac:.0%}); skipping frame"
            )
            return
        if not (obs or full):  # only the corrected map was wanted
            return

        d = depth_corr.ravel()
        pts = (self.rays_opt * d[:, None]) @ self.proj.R.T + self.proj.C
        bx, by, bz = pts[:, 0], pts[:, 1], pts[:, 2]
        rng = np.hypot(bx, by)

        header = msg.header
        header.frame_id = (
            self.base_frame
        )  # cloud is in the base frame, stamped at capture

        img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if img.shape[:2] != (self.proj.height, self.proj.width):
            img = cv2.resize(img, (self.proj.width, self.proj.height))
        colors = img.reshape(-1, 3)  # row-major BGR, aligned with the pixel grid

        if full:
            shown = np.isfinite(rng) & (rng < self.rmax) & (bz < self.z_ceil)
            s = self.stride
            self.full_cloud_pub.publish(
                make_xyzrgb_cloud(
                    header, pts[shown][::s].astype(np.float32), colors[shown][::s]
                )
            )

        if obs:
            keep = (
                (bz > self.z_obs)
                & (bz < self.z_ceil)
                & (rng > self.rmin)
                & (rng < self.rmax)
            )
            cloud = pts[keep][:: self.stride].astype(np.float32)
            self.pub.publish(
                make_xyzrgb_cloud(header, cloud, colors[keep][:: self.stride])
            )
            latency_ms = (
                self.get_clock().now() - rclpy.time.Time.from_msg(msg.header.stamp)
            ).nanoseconds / 1e6
            self.get_logger().info(
                f"obstacles: {len(cloud)} pts  scale:{source} {frac:.0%} "
                f"a={ab[0]:.3f} b={ab[1]:.3f}{self.diag}  "
                f"stamp-to-publish {latency_ms:.0f} ms",
                throttle_duration_sec=2.0,
            )


def main():
    rclpy.init()
    node = DepthObstacleNode()
    # MultiThreaded so the scan callback group runs while _on_image blocks on inference.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
