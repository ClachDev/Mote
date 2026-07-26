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
from rclpy.executors import ExternalShutdownException
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
        # Joints confirmed present and in position mode; only these may be
        # commanded. A servo whose mode cannot be verified might be in wheel
        # mode, where a position goal spins it continuously.
        self._ready: set[str] = set()
        # Joints currently holding torque. Engagement is per-joint so one
        # failed engage is retried on the next command instead of a single
        # flag marking the whole arm engaged and leaving that joint limp.
        self._engaged: set[str] = set()
        for joint in self.cfg.joints:
            if not self.bus.ping(joint.id):
                self.get_logger().warn(
                    f"servo for joint '{joint.name}' (id {joint.id}) did not respond"
                )
                continue
            in_position_mode = self.bus.ensure_position_mode(joint.id)
            self.bus.set_torque(joint.id, False)  # start limp
            if not in_position_mode:
                self.get_logger().warn(
                    f"joint '{joint.name}' (id {joint.id}) not confirmed in "
                    "position mode — excluded from control (state reads only)"
                )
                continue
            self._ready.add(joint.name)

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

    def _disengage_all(self) -> None:
        for joint in self.cfg.joints:
            try:
                self.bus.set_torque(joint.id, False)
            except BusError as exc:
                self.get_logger().warn(str(exc))
        self._engaged.clear()

    def _engage_joint(self, joint) -> bool:
        """Enable torque on one joint without moving it; True on success.

        A servo drives to whatever its GOAL_POSITION register holds the instant
        torque is enabled, and that register may be stale (a previous session, or
        the factory default). So seed the joint's goal with its *present*
        position first, then enable — order matters: enabling first is what makes
        an arm snap to a pose nobody asked for.
        """
        counts = self.bus.read_position(joint.id)
        if counts is None:
            self.get_logger().warn(
                f"joint '{joint.name}': cannot read position, leaving it limp "
                "rather than enabling torque against an unknown goal"
            )
            return False
        try:
            self.bus.write_goal(
                joint.id, counts, self.cfg.moving_speed, self.cfg.moving_acc
            )
            self.bus.set_torque(joint.id, True)
        except BusError as exc:
            self.get_logger().warn(str(exc))
            return False
        self._engaged.add(joint.name)
        return True

    def _engage_all(self) -> None:
        """Take hold of the current pose on every controllable joint.

        Joints already holding are left alone; joints that fail stay limp and
        are retried on the next command.
        """
        for joint in self.cfg.joints:
            if joint.name in self._ready and joint.name not in self._engaged:
                self._engage_joint(joint)

    def _on_goal(self, msg: JointState) -> None:
        # An explicit command has arrived: take hold of the current pose
        # first, then move only the joints named in the goal.
        self._engage_all()
        for name, rad in zip(msg.name, msg.position):
            try:
                joint = self.cfg.joint(name)
            except KeyError:
                self.get_logger().warn(f"ignoring goal for unknown joint '{name}'")
                continue
            if name not in self._engaged:
                self.get_logger().warn(
                    f"ignoring goal for joint '{name}' — not holding torque "
                    "(failed enumeration, mode check, or engage)"
                )
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
            self._engage_all()
            limp = [j.name for j in self.cfg.joints if j.name not in self._engaged]
            response.success = not limp
            if limp:
                response.message = f"torque enabled; left limp: {', '.join(limp)}"
            else:
                response.message = "torque enabled (holding current pose)"
        else:
            self._disengage_all()
            response.success = True
            response.message = "torque disabled (limp)"
        return response

    def shutdown(self) -> None:
        """Leave the arm limp and release the bus.

        Safety-critical and best-effort: a failure to drop torque on one joint
        must not stop us trying the rest, or leave the port held.
        """
        try:
            self._disengage_all()
        except Exception as exc:  # noqa: BLE001 - never mask the port close
            self.get_logger().error(f"failed to disable torque on shutdown: {exc}")
        finally:
            self.bus.close()


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = ArmDriver()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:  # noqa: BLE001 - see below
        # SIGINT tears the rcl context down underneath spin(), which surfaces
        # as an ExternalShutdownException or a bare RCLError depending on how
        # the node was launched (`ros2 run` vs directly). Neither is publicly
        # catchable as one type, so treat "the context is gone" as the ordinary
        # stop it is, and re-raise anything that failed while it was still up.
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
