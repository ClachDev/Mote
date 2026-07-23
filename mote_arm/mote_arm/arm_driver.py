"""Arm driver node: the single owner of the arm servo bus.

Responsibilities:
  * publish ``/joint_states`` for the arm joints (so robot_state_publisher can
    put the arm in TF),
  * accept absolute joint goals on ``arm/goal`` (sensor_msgs/JointState, rad),
    soft-clamped to the per-joint limits from robot.yaml,
  * expose ``arm/set_torque`` (std_srvs/SetBool) to hold (True) or go limp
    (False).

Torque policy (see mote_arm/README.md):
  * starts LIMP — torque disabled, nothing moves until commanded,
  * a goal implicitly enables torque (an explicit command has arrived),
  * shutdown drops torque so the arm is left safely back-drivable.

Because it owns the port, run the driver *or* the standalone ``arm_check`` /
``jog --standalone`` tools, never both at once.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

from mote_arm import config
from mote_arm.bus import BusError, FeetechBus


class ArmDriver(Node):
    def __init__(self):
        super().__init__("arm_driver")
        self.declare_parameter("robot_yaml", "")
        self.declare_parameter("publish_rate", 20.0)

        path = self.get_parameter("robot_yaml").get_parameter_value().string_value
        self.cfg = config.ArmConfig.from_yaml_file(path) if path else config.load()

        self.bus = FeetechBus(self.cfg.port, self.cfg.baud_rate)
        self.bus.open()
        self._torque_on = False
        for joint in self.cfg.joints:
            if not self.bus.ping(joint.id):
                self.get_logger().warn(
                    f"servo for joint '{joint.name}' (id {joint.id}) did not respond"
                )
                continue
            self.bus.ensure_position_mode(joint.id)
            self.bus.set_torque(joint.id, False)  # start limp

        self._pub = self.create_publisher(JointState, "joint_states", 10)
        self.create_subscription(JointState, "arm/goal", self._on_goal, 10)
        self.create_service(SetBool, "arm/set_torque", self._on_set_torque)

        rate = self.get_parameter("publish_rate").get_parameter_value().double_value
        self.create_timer(1.0 / max(1.0, rate), self._publish_states)

        self.get_logger().info(
            f"arm_driver up on {self.cfg.port}: joints {self.cfg.names} "
            "(torque OFF — limp until commanded)"
        )

    def _publish_states(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for joint in self.cfg.joints:
            counts = self.bus.read_position(joint.id)
            if counts is None:
                continue
            msg.name.append(joint.name)
            msg.position.append(joint.counts_to_rad(counts))
        if msg.name:
            self._pub.publish(msg)

    def _set_all_torque(self, enable: bool) -> None:
        for joint in self.cfg.joints:
            try:
                self.bus.set_torque(joint.id, enable)
            except BusError as exc:
                self.get_logger().warn(str(exc))
        self._torque_on = enable

    def _on_goal(self, msg: JointState) -> None:
        if not self._torque_on:
            # An explicit command has arrived: hold current pose, then move.
            self._set_all_torque(True)
        for name, rad in zip(msg.name, msg.position):
            try:
                joint = self.cfg.joint(name)
            except KeyError:
                self.get_logger().warn(f"ignoring goal for unknown joint '{name}'")
                continue
            clamped = joint.clamp_rad(rad)
            if clamped != rad:
                self.get_logger().warn(
                    f"joint '{name}' goal {rad:.3f} clamped to {clamped:.3f} rad"
                )
            counts = joint.rad_to_counts(clamped)
            try:
                self.bus.write_goal(
                    joint.id, counts, self.cfg.moving_speed, self.cfg.moving_acc
                )
            except BusError as exc:
                self.get_logger().warn(str(exc))

    def _on_set_torque(self, request, response):
        if request.data:
            # Seed each goal with the present position so enabling torque holds
            # the current pose rather than snapping to a stale target.
            for joint in self.cfg.joints:
                counts = self.bus.read_position(joint.id)
                if counts is not None:
                    try:
                        self.bus.set_torque(joint.id, True)
                        self.bus.write_goal(
                            joint.id, counts, self.cfg.moving_speed, self.cfg.moving_acc
                        )
                    except BusError as exc:
                        self.get_logger().warn(str(exc))
            self._torque_on = True
            response.message = "torque enabled (holding current pose)"
        else:
            self._set_all_torque(False)
            response.message = "torque disabled (limp)"
        response.success = True
        return response

    def shutdown(self) -> None:
        try:
            self._set_all_torque(False)
        finally:
            self.bus.close()


def main() -> None:
    rclpy.init()
    node = ArmDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
