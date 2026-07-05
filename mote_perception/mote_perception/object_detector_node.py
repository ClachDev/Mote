"""Off-board open-vocabulary detection -> map-frame object poses (no torch).

Subscribes the compressed camera stream, forwards each frame plus the current
label set to the detection server (tools/detect_server.py, in the pixi depth
environment; protocol in detect_wire.py), grounds each detection by dropping
the bbox bottom-centre pixel through the floor plane (the fetch mission's
objects sit on the floor, so no depth model is needed in this loop), and
publishes the labelled poses for the task layer.

Labels arrive on ``detect/labels`` (std_msgs/String, comma-separated,
transient_local so a set outlives publisher and node restarts); an empty
string idles the node. The task layer's AcquireObject behaviour sets the label
while it needs a pose and clears it after, so inference only runs during
acquisition. Detections go out on ``detected_objects``
(vision_msgs/Detection3DArray) in the map frame, transformed at the image
capture stamp so the off-board latency does not smear poses, plus a 2D debug
topic and an annotated overlay image for RViz.
"""

import cv2
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import String
import tf2_ros
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)

from mote_perception.detect_wire import DetectClient
from mote_perception.ground_projection import GroundProjector, transform_to_matrix

LABELS_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__("object_detector_node")
        self.declare_parameter("server_host", "127.0.0.1")
        self.declare_parameter("server_port", 5602)
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("min_score", 0.3)
        # Floor-ray grounding degrades as the ray grazes the floor (the camera
        # sits at ~0.10 m), so far detections carry metre-scale error. Good
        # enough to navigate towards, not to trust blindly: clamp to the band
        # where the standoff goal lands near the object.
        self.declare_parameter("range_max", 3.0)
        self.declare_parameter("socket_timeout", 10.0)

        self.base_frame = self.get_parameter("base_frame").value
        self.map_frame = self.get_parameter("map_frame").value
        self.min_score = float(self.get_parameter("min_score").value)
        self.range_max = float(self.get_parameter("range_max").value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.client = DetectClient(
            self.get_parameter("server_host").value,
            int(self.get_parameter("server_port").value),
            float(self.get_parameter("socket_timeout").value),
            warn=lambda m: self.get_logger().warn(m, throttle_duration_sec=2.0),
        )
        self.cam_info = None
        self.proj = None
        self.labels = []

        self.create_subscription(CameraInfo, "camera_info", self._on_info, 10)
        self.create_subscription(String, "detect/labels", self._on_labels, LABELS_QOS)
        self.image_sub = None
        self.image_group = MutuallyExclusiveCallbackGroup()
        self.pub_3d = self.create_publisher(Detection3DArray, "detected_objects", 5)
        self.pub_2d = self.create_publisher(Detection2DArray, "detections", 5)
        self.pub_overlay = self.create_publisher(
            CompressedImage, "detections/overlay/compressed", 5
        )
        self.get_logger().info(
            "object_detector_node up; waiting for labels on detect/labels"
        )

    def _on_info(self, msg):
        self.cam_info = msg

    def _on_labels(self, msg):
        labels = [w.strip() for w in msg.data.split(",") if w.strip()]
        if labels == self.labels:
            return
        self.labels = labels
        self.get_logger().info(f"detecting {labels or 'nothing (idle)'}")
        # Subscribe only while there is something to detect: the camera stream
        # is not pulled (over Wi-Fi, typically) while the node idles. Depth 1 +
        # own callback group: inference is slower than the frame rate, so only
        # the freshest frame is processed while labels/tf/info stay live on the
        # other executor threads.
        if labels and self.image_sub is None:
            self.image_sub = self.create_subscription(
                CompressedImage,
                "image/compressed",
                self._on_image,
                1,
                callback_group=self.image_group,
            )
        elif not labels and self.image_sub is not None:
            self.destroy_subscription(self.image_sub)
            self.image_sub = None

    def _ensure_setup(self):
        if self.proj is not None:
            return True
        if self.cam_info is None:
            return False
        optical = self.cam_info.header.frame_id
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, optical, rclpy.time.Time()
            )
        except tf2_ros.TransformException:
            return False
        T = transform_to_matrix(tf.transform.translation, tf.transform.rotation)
        self.proj = GroundProjector.from_camera_info(self.cam_info, T)
        self.get_logger().info(
            f"setup done: camera at {self.proj.camera_height:.3f} m ({optical})"
        )
        return True

    def _map_from_base(self, stamp):
        """map<-base at the image capture stamp (inference already spent the
        time tf needs to catch up past it), or None with a warning."""
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, stamp)
        except tf2_ros.TransformException as e:
            self.get_logger().warn(
                f"no {self.map_frame}<-{self.base_frame} at capture time ({e}); "
                "skipping frame",
                throttle_duration_sec=2.0,
            )
            return None
        return transform_to_matrix(tf.transform.translation, tf.transform.rotation)

    def _on_image(self, msg):
        if not self.labels or not self._ensure_setup():
            return
        blob = bytes(msg.data)
        fmt = (msg.format or "").lower()
        if fmt and not any(c in fmt for c in ("jpeg", "jpg", "png")):
            self.get_logger().warn(
                f"unsupported compressed format {fmt!r}; skipping",
                throttle_duration_sec=2.0,
            )
            return
        labels = list(self.labels)
        dets = self.client.infer(blob, labels)
        if dets is None:
            return
        dets = [d for d in dets if d[1] >= self.min_score]

        T_map_base = self._map_from_base(msg.header.stamp)
        if T_map_base is None:
            return

        out2d = Detection2DArray()
        out2d.header.stamp = msg.header.stamp
        out2d.header.frame_id = self.cam_info.header.frame_id
        out3d = Detection3DArray()
        out3d.header.stamp = msg.header.stamp
        out3d.header.frame_id = self.map_frame
        grounded = []
        for label, score, (x0, y0, x1, y1) in dets:
            d2 = Detection2D()
            d2.header = out2d.header
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = label
            hyp.hypothesis.score = score
            d2.results.append(hyp)
            d2.bbox.center.position.x = (x0 + x1) / 2.0
            d2.bbox.center.position.y = (y0 + y1) / 2.0
            d2.bbox.size_x = x1 - x0
            d2.bbox.size_y = y1 - y0
            out2d.detections.append(d2)

            base_pt = self.proj.pixels_to_ground([[(x0 + x1) / 2.0, y1]])[0]
            if not np.isfinite(base_pt).all():
                continue
            if np.hypot(base_pt[0], base_pt[1]) > self.range_max:
                continue
            map_pt = T_map_base[:3, :3] @ base_pt + T_map_base[:3, 3]
            d3 = Detection3D()
            d3.header = out3d.header
            hyp3 = ObjectHypothesisWithPose()
            hyp3.hypothesis.class_id = label
            hyp3.hypothesis.score = score
            hyp3.pose.pose.position.x = map_pt[0]
            hyp3.pose.pose.position.y = map_pt[1]
            hyp3.pose.pose.position.z = map_pt[2]
            hyp3.pose.pose.orientation.w = 1.0
            d3.results.append(hyp3)
            d3.bbox.center.position.x = map_pt[0]
            d3.bbox.center.position.y = map_pt[1]
            d3.bbox.center.position.z = map_pt[2]
            d3.bbox.center.orientation.w = 1.0
            out3d.detections.append(d3)
            grounded.append((label, score, map_pt))

        self.pub_2d.publish(out2d)
        self.pub_3d.publish(out3d)
        if grounded:
            summary = ", ".join(
                f"{lb} {sc:.0%} ({p[0]:.2f}, {p[1]:.2f})" for lb, sc, p in grounded
            )
            self.get_logger().info(f"detected {summary}", throttle_duration_sec=1.0)
        if self.pub_overlay.get_subscription_count() > 0:
            self._publish_overlay(blob, msg.header, dets)

    def _publish_overlay(self, blob, header, dets):
        img = cv2.imdecode(np.frombuffer(blob, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        for label, score, (x0, y0, x1, y1) in dets:
            p0, p1 = (int(x0), int(y0)), (int(x1), int(y1))
            cv2.rectangle(img, p0, p1, (0, 255, 0), 2)
            cv2.putText(
                img,
                f"{label} {score:.0%}",
                (p0[0], max(p0[1] - 5, 12)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )
        out = CompressedImage()
        out.header = header
        out.format = "jpeg"
        out.data = cv2.imencode(".jpg", img)[1].tobytes()
        self.pub_overlay.publish(out)


def main():
    rclpy.init()
    node = ObjectDetectorNode()
    # MultiThreaded so tf/labels/camera_info stay live while _on_image blocks
    # on inference.
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
