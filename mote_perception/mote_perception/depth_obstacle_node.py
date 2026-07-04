"""Off-board depth -> obstacle PointCloud2 node (no torch; runs anywhere).

Subscribes the compressed camera stream, forwards each frame to the depth server
(tools/depth_server.py, in the pixi depth environment; protocol in depth_wire.py),
metrically rescales the returned depth against time-synced lidar range returns,
back-projects to 3D, keeps points standing above the floor, and publishes them as
a PointCloud2 for a Nav2 obstacle layer. The cloud is stamped with the IMAGE
capture time so Nav2 places it via tf at the moment it was seen — which is how
the (off-board, ~0.6 s) latency is absorbed without inflation or a speed cap.

Lidar stays the primary, low-latency obstacle/clearing source; this is a slow
supplementary marker for the low/thin things the 2D scan misses.
"""

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

from mote_perception.depth_wire import DepthClient
from mote_perception.ground_projection import (
    GroundProjector,
    fit_ground_plane,
    level_rotation,
    transform_to_matrix,
)
from mote_perception.lidar_rescale import LidarDepthRescaler, apply_affine_disparity


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


class DepthObstacleNode(Node):
    def __init__(self):
        super().__init__("depth_obstacle_node")
        self.declare_parameter("server_host", "127.0.0.1")
        self.declare_parameter("server_port", 5601)
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("optical_frame", "camera_optical_link")
        self.declare_parameter("z_obstacle", 0.02)
        # Upper bound of the obstacle band: anything taller is overhead the robot
        # drives under (a chair seat, a tabletop), so it must NOT be marked -- only
        # what falls in the robot's vertical envelope is an obstacle. The legs of
        # such furniture reach the floor and still mark, so the robot avoids the legs
        # but paths through the clear space beneath. Nav2's camera_layer
        # max_obstacle_height is the authoritative go-under gate; this is a generous
        # publish ceiling above it, kept low to trim the cloud streamed over Wi-Fi.
        self.declare_parameter("z_obstacle_max", 0.5)
        # Upper bound of the full debug cloud only (lets RViz show the whole scene).
        self.declare_parameter("z_ceiling", 1.6)
        self.declare_parameter("range_min", 0.25)
        # The 0.10 m camera mount leaves a usable floor band of ~0.25-1.2 m; past that
        # the monocular depth compresses badly, so obstacles there are a false-positive
        # source. Clamp the published cloud to the near band this layer is trusted for
        # (the goal is the low, near things the lidar plane misses, not far detection).
        self.declare_parameter("range_max", 1.2)
        self.declare_parameter("pixel_stride", 3)
        self.declare_parameter("socket_timeout", 2.0)
        self.declare_parameter("publish_debug", True)
        self.declare_parameter("scan_topic", "/scan_filtered")
        self.declare_parameter("scan_max_dt", 0.2)
        # Fit the floor plane each frame and rotate the cloud level, removing residual
        # camera tilt: stands verticals up and flattens the floor for z-classification.
        self.declare_parameter("plane_fit", True)
        self.declare_parameter("plane_max_tilt_deg", 5.0)

        self.base_frame = self.get_parameter("base_frame").value
        self.optical_frame = self.get_parameter("optical_frame").value
        self.z_obs = self.get_parameter("z_obstacle").value
        self.z_obs_max = self.get_parameter("z_obstacle_max").value
        self.z_ceil = self.get_parameter("z_ceiling").value
        self.rmin = self.get_parameter("range_min").value
        self.rmax = self.get_parameter("range_max").value
        self.stride = self.get_parameter("pixel_stride").value
        self.scan_max_dt = float(self.get_parameter("scan_max_dt").value)
        self.plane_fit = self.get_parameter("plane_fit").value
        self.plane_max_tilt = float(self.get_parameter("plane_max_tilt_deg").value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.client = DepthClient(
            self.get_parameter("server_host").value,
            int(self.get_parameter("server_port").value),
            float(self.get_parameter("socket_timeout").value),
            warn=lambda m: self.get_logger().warn(m, throttle_duration_sec=2.0),
        )
        self.cam_info = None
        self.proj = None
        self.lidar_rescaler = None
        self.scans = deque(maxlen=50)
        self.last_ab = None
        self.last_R = (
            None  # last good ground-level rotation; held when a frame can't fit
        )
        self.last_c = 0.0  # floor-plane offset held with last_R
        self.diag = ""
        self.plane_diag = ""

        self.create_subscription(CameraInfo, "camera_info", self._on_info, 10)
        # Depth 1: inference is slower than the frame rate, so only ever process the
        # freshest frame and let the rest drop -- a deeper queue would just feed stale
        # frames that publish an obstacle cloud already behind the robot.
        self.create_subscription(CompressedImage, "image/compressed", self._on_image, 1)
        # Own callback group so scans keep buffering on another thread while the
        # ~0.5 s blocking inference runs in _on_image (else the buffer goes stale
        # and no scan lands near an image's capture stamp).
        self.create_subscription(
            LaserScan,
            self.get_parameter("scan_topic").value,
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
        """Metric-rescale the depth map against the time-nearest lidar scan.

        When the nearest scan can't constrain the fit (no time match, too few
        pairs, no depth spread — e.g. facing a flat wall), hold the last good
        (a, b): the correction drifts slowly, so a briefly stale scale beats a
        missing cloud. Before the first successful fit there is nothing to hold
        and the frame is skipped.

        Returns (corrected_depth, (a, b, frac), source) or (None, None, source).
        """
        self.diag = ""
        if not self._ensure_lidar():
            return None, None, "lidar"
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
            dt = f"{dt_ms:.0f}ms" if dt_ms is not None else "n/a"
            self.diag = f"  scans {len(self.scans)} dt {dt} (no match)"
        if self.last_ab is not None:
            a, b = self.last_ab
            return apply_affine_disparity(depth, a, b), (a, b, float("nan")), "hold"
        return None, None, "lidar"

    def _ground_correct(self, pts):
        """Rotate the cloud so the fitted floor plane is level.

        Removes the residual camera tilt (dynamic pitch/floor slope) the level URDF
        transform misses: stands verticals up and flattens the floor so a point's z
        is its true height above the floor. Rejects an over-large or unconstrained fit
        and holds the last good rotation (identity until the first one), so the whole
        cloud can't snap flat<->tilted frame to frame.
        """
        self.plane_diag = ""
        if not self.plane_fit:
            return pts
        fit = fit_ground_plane(pts)
        if fit is not None:
            a, b, c, frac = fit
            tilt = float(np.degrees(np.arctan(np.hypot(a, b))))
            if tilt <= self.plane_max_tilt:
                self.last_R = level_rotation(a, b)
                self.last_c = c
                self.plane_diag = f" tilt {tilt:.1f}d {frac:.0%}"
            else:
                self.plane_diag = f" tilt {tilt:.1f}d(rej)"
        if self.last_R is not None:
            pts = (pts - self.proj.C) @ self.last_R.T + self.proj.C
            pts[:, 2] -= self.last_c
        return pts

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
        self.get_logger().info(
            f"setup done: camera at {self.proj.camera_height:.3f} m; publishing obstacles"
        )
        return True

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

        blob = bytes(msg.data)
        # image_transport sets format to e.g. "rgb8; jpeg compressed bgr8"
        fmt = (msg.format or "").lower()
        if fmt and not any(c in fmt for c in ("jpeg", "jpg", "png")):
            self.get_logger().warn(
                f"unsupported compressed format {fmt!r}; skipping",
                throttle_duration_sec=2.0,
            )
            return
        depth = self.client.infer(blob)
        if depth is None:
            return
        if depth.shape != (self.proj.height, self.proj.width):
            depth = cv2.resize(depth, (self.proj.width, self.proj.height))
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

        pts = self.proj.back_project(depth_corr)
        pts = self._ground_correct(pts)
        bx, by, bz = pts[:, 0], pts[:, 1], pts[:, 2]
        rng = np.hypot(bx, by)

        header = msg.header
        header.frame_id = (
            self.base_frame
        )  # cloud is in the base frame, stamped at capture

        img = cv2.imdecode(np.frombuffer(blob, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            self.get_logger().warn("corrupt compressed frame; skipping")
            return
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
                & (bz < self.z_obs_max)
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
                f"a={ab[0]:.3f} b={ab[1]:.3f}{self.diag}{self.plane_diag}  "
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
        node.client.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
