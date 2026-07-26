"""Stamp an unstamped Twist so a browser teleop panel can drive the controller.

DiffDriveController is configured for stamped commands (`enable_stamped_cmd_vel`
in nav2_params.yaml, `stamped:=true` for the keyboard teleop), because the stamp
is what its `cmd_vel_timeout` measures staleness against. Foxglove's Teleop panel
publishes `geometry_msgs/Twist` and only that, so a remote operator has no way to
reach the controller without something adding a header.

This relay is deliberately dumb: one message in, one message out, no timer and no
memory of the last command. That is the safety property, not an omission -- the
robot stops when commands stop arriving, so a dropped link is a stop. A relay
that re-published the last command on a timer would keep the wheels turning after
the operator's link died, which is precisely the failure the controller's
`cmd_vel_timeout` exists to prevent.

Velocity limits are not enforced here either; DiffDriveController already clamps
to the `linear.x`/`angular.z` maxima in controllers.yaml, so a mistyped value in
a Foxglove layout is bounded by the robot's own configuration rather than by the
correctness of this file.
"""

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


def to_stamped(twist, stamp, frame_id):
    """Wrap a Twist in a TwistStamped carrying `stamp` and `frame_id`."""
    out = TwistStamped()
    out.header.stamp = stamp
    out.header.frame_id = frame_id
    out.twist = twist
    return out


class TwistRelay(Node):
    def __init__(self):
        super().__init__("twist_relay")
        # The command is a body-frame velocity, so the base frame is the honest
        # label; DiffDriveController does not transform it.
        self.frame_id = self.declare_parameter("frame_id", "base_footprint").value
        self.pub = self.create_publisher(TwistStamped, "cmd_vel_out", 10)
        self.create_subscription(Twist, "cmd_vel_in", self._cb, 10)

    def _cb(self, msg):
        self.pub.publish(
            to_stamped(msg, self.get_clock().now().to_msg(), self.frame_id)
        )


def main():
    rclpy.init()
    node = TwistRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
