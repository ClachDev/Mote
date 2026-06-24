"""Republish wheel odometry as an inverted TF leaf so a lidar-odometry node can
own the odom->base edge while still receiving the wheel odometry as a prior.

diff_drive publishes the wheel pose (base in odom) on a topic; this node
broadcasts its inverse as base_frame -> child_frame (a leaf). kinematic_icp,
configured with wheel_odom_frame = child_frame, reads that leaf as its motion
prior and is then free to publish the real odom -> base transform itself.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def _rotate(q, v):
    """Rotate vector v by quaternion q = (x, y, z, w)."""
    x, y, z, w = q
    ux = 2.0 * (y * v[2] - z * v[1])
    uy = 2.0 * (z * v[0] - x * v[2])
    uz = 2.0 * (x * v[1] - y * v[0])
    return (
        v[0] + w * ux + (y * uz - z * uy),
        v[1] + w * uy + (z * ux - x * uz),
        v[2] + w * uz + (x * uy - y * ux),
    )


class OdomTfRelay(Node):
    def __init__(self):
        super().__init__("odom_tf_relay")
        self.child_frame = self.declare_parameter("child_frame", "odom_wheel").value
        self.br = TransformBroadcaster(self)
        self.create_subscription(Odometry, "odom_in", self._cb, 10)

    def _cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        q_inv = (-q.x, -q.y, -q.z, q.w)
        ti = _rotate(q_inv, (p.x, p.y, p.z))

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.child_frame_id  # base_footprint
        t.child_frame_id = self.child_frame  # odom_wheel (leaf)
        t.transform.translation.x = -ti[0]
        t.transform.translation.y = -ti[1]
        t.transform.translation.z = -ti[2]
        t.transform.rotation.x = q_inv[0]
        t.transform.rotation.y = q_inv[1]
        t.transform.rotation.z = q_inv[2]
        t.transform.rotation.w = q_inv[3]
        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = OdomTfRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
