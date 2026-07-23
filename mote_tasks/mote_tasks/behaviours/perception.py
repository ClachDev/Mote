"""Behaviours that resolve mission targets through the perception stack."""

import math

import py_trees
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray

LABELS_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class AcquireObject(py_trees.behaviour.Behaviour):
    """Resolve the mission's object pose, detecting it by label when needed.

    Zone-target missions arrive with ``pose_key`` already set and pass straight
    through. Label missions (``label_key`` set instead) publish the label to
    the detector on ``detect/labels`` and wait for a matching detection on
    ``detected_objects``; the first match at or above ``min_score`` becomes a
    standoff goal — ``standoff`` metres short of the object along the line from
    the robot, facing it — written to ``pose_key``. The label is cleared on the
    way out (success, failure, or preemption) so the detector idles between
    missions. FAILURE when no detection arrives within ``timeout`` seconds.
    """

    def __init__(
        self,
        name: str,
        pose_key: str,
        label_key: str,
        standoff: float = 0.4,
        min_score: float = 0.3,
        timeout: float = 30.0,
    ):
        super().__init__(name)
        self.pose_key = pose_key
        self.label_key = label_key
        self.standoff = standoff
        self.min_score = min_score
        self.timeout = timeout
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(pose_key, access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(label_key, access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.labels_pub = self.node.create_publisher(
            String, "detect/labels", LABELS_QOS
        )
        self.node.create_subscription(
            Detection3DArray, "detected_objects", self._on_detections, 5
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
        self.base_frame = "base_footprint"
        self.label = None
        self.match = None

    def _on_detections(self, msg):
        if not self.label:
            return
        best = None
        for det in msg.detections:
            for hyp in det.results:
                if hyp.hypothesis.class_id != self.label:
                    continue
                if hyp.hypothesis.score < self.min_score:
                    continue
                if best is None or hyp.hypothesis.score > best[0]:
                    best = (hyp.hypothesis.score, msg.header, hyp.pose.pose)
        if best is not None:
            self.match = best

    def initialise(self):
        self.match = None
        self.label = None
        if self.blackboard.exists(self.pose_key):
            return
        self.label = self.blackboard.get(self.label_key)
        self.deadline = self.node.get_clock().now() + rclpy.duration.Duration(
            seconds=self.timeout
        )
        self.labels_pub.publish(String(data=self.label))
        self.node.get_logger().info(f"{self.name}: looking for '{self.label}'")

    def update(self):
        if self.label is None:
            return py_trees.common.Status.SUCCESS
        if self.match is not None:
            score, header, pose = self.match
            goal = self._standoff_goal(header, pose)
            if goal is None:
                return py_trees.common.Status.RUNNING
            self.blackboard.set(self.pose_key, goal)
            p = goal.pose.position
            self.node.get_logger().info(
                f"{self.name}: '{self.label}' at {score:.0%} -> "
                f"standoff ({p.x:.2f}, {p.y:.2f})"
            )
            return py_trees.common.Status.SUCCESS
        if self.node.get_clock().now() > self.deadline:
            self.node.get_logger().error(
                f"{self.name}: no '{self.label}' seen within {self.timeout:.0f} s"
            )
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING

    def _standoff_goal(self, header, pose):
        """Goal `standoff` metres short of the object, facing it, in the
        detection's frame. None (retry next tick) while the robot pose is
        unavailable."""
        try:
            tf = self.tf_buffer.lookup_transform(
                header.frame_id, self.base_frame, rclpy.time.Time()
            )
        except tf2_ros.TransformException as e:
            self.node.get_logger().warn(
                f"{self.name}: no robot pose yet ({e})", throttle_duration_sec=2.0
            )
            return None
        rx, ry = tf.transform.translation.x, tf.transform.translation.y
        dx, dy = pose.position.x - rx, pose.position.y - ry
        dist = math.hypot(dx, dy)
        yaw = math.atan2(dy, dx)
        goal = PoseStamped()
        goal.header.frame_id = header.frame_id
        if dist > self.standoff:
            back = self.standoff / dist
            goal.pose.position.x = pose.position.x - dx * back
            goal.pose.position.y = pose.position.y - dy * back
        else:
            goal.pose.position.x = rx
            goal.pose.position.y = ry
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        return goal

    def terminate(self, new_status):
        if self.label is not None:
            self.labels_pub.publish(String(data=""))
            self.label = None
