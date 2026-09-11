"""Talking to the arm through ros2_control.

The arm shares the drive-wheel serial bus, so `MoteHardware` owns the port and
exports the arm's position command interfaces; nothing in this package opens the
bus to *move* the arm any more. Everything that commands it — the jog CLI, pose
replay — goes through `arm_controller` instead, and this module is the one place
that knows how.

Two things are worth stating once, here, rather than in each client:

* **"Torque" is controller activation.** Claiming the position command
  interfaces is what makes the hardware take hold of the arm's current pose
  (MoteHardware::perform_command_mode_switch); releasing them drops torque. So
  the arm is limp whenever `arm_controller` is inactive, which is how it is
  spawned, and a client that wants to move the arm activates it first and
  deactivates it on the way out. **Whether it is active is read, never
  assumed**: `arm-pose go` leaves the controller holding the pose it reached, so
  the next command client starts against an arm that is already held, and a
  process that assumed otherwise asked for a STRICT switch the controller
  manager refuses — once per streamed setpoint.
* **A goal is a single-point trajectory starting now.** Leaving the header stamp
  at zero means "start now" on the *robot's* clock, so an operator's clock never
  enters the motion path.
"""

from __future__ import annotations

import time

from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers, SwitchController
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_CONTROLLER = "arm_controller"
TRAJECTORY_TOPIC = f"{ARM_CONTROLLER}/joint_trajectory"
SWITCH_SERVICE = "controller_manager/switch_controller"
LIST_SERVICE = "controller_manager/list_controllers"


def duration_msg(seconds: float) -> Duration:
    msg = Duration()
    msg.sec = int(seconds)
    msg.nanosec = int(round((seconds - msg.sec) * 1e9))
    return msg


def trajectory(goals: dict[str, float], seconds: float) -> JointTrajectory:
    """A single-point trajectory moving ``goals`` (name -> rad), starting now."""
    names = list(goals)
    point = JointTrajectoryPoint()
    point.positions = [goals[n] for n in names]
    point.time_from_start = duration_msg(seconds)

    msg = JointTrajectory()
    msg.joint_names = names
    msg.points = [point]
    return msg


class ArmControl:
    """The publisher and the activation switch a command client needs.

    Composed into a Node rather than subclassed from one so the same wiring
    serves the jog REPL and pose replay without either inheriting the other's
    behaviour.
    """

    def __init__(self, node):
        self._node = node
        self._pub = node.create_publisher(JointTrajectory, TRAJECTORY_TOPIC, 10)
        self._switch = node.create_client(SwitchController, SWITCH_SERVICE)
        self._list = node.create_client(ListControllers, LIST_SERVICE)
        # None until asked. Whether the arm is held is a fact about the
        # controller manager, not about this process: `arm-pose go` leaves
        # the controller active on purpose, so the next command client
        # starts against an arm that is already holding.
        self.holding: bool | None = None

    def send(self, goals: dict[str, float], seconds: float) -> bool:
        """Command the given joints, taking hold of the arm first if it is limp."""
        if not self.set_holding(True):
            return False
        self._pub.publish(trajectory(goals, seconds))
        return True

    @property
    def held(self) -> bool:
        """Whether the arm is being held, asked of the graph the first time."""
        if self.holding is None:
            self.holding = self.active()
        return bool(self.holding)

    def set_holding(self, hold: bool, timeout: float = 5.0) -> bool:
        """Activate (hold) or deactivate (limp) the arm controller.

        Returns False if the request could not be delivered — the caller must
        not then assume the arm is holding.
        """
        if self.holding is None:
            self.holding = self.active(timeout)
        if hold == self.holding:
            return True
        if not self._switch.wait_for_service(timeout_sec=timeout):
            self._node.get_logger().warn(
                f"{SWITCH_SERVICE} unavailable — is a stack that owns the servo "
                "bus running (`pixi run arm`, or `pixi run robot`)?"
            )
            return False

        req = SwitchController.Request()
        if hold:
            req.activate_controllers = [ARM_CONTROLLER]
        else:
            req.deactivate_controllers = [ARM_CONTROLLER]
        req.strictness = SwitchController.Request.STRICT

        result = self._call(self._switch, req, timeout)
        if result is None or not result.ok:
            # A STRICT switch refuses a controller already in the state asked
            # for, which is the caller's success. Re-read rather than assume
            # either way: something else on the graph may have moved it.
            if self.active(timeout) == hold:
                self.holding = hold
                return True
            self._node.get_logger().warn(
                f"failed to {'activate' if hold else 'deactivate'} {ARM_CONTROLLER}"
            )
            return False
        self.holding = hold
        return True

    def active(self, timeout: float = 5.0) -> bool | None:
        """Whether arm_controller is active, or None if that cannot be read."""
        if not self._list.wait_for_service(timeout_sec=timeout):
            return None
        result = self._call(self._list, ListControllers.Request(), timeout)
        if result is None:
            return None
        for controller in result.controller:
            if controller.name == ARM_CONTROLLER:
                return controller.state == "active"
        return None

    def _call(self, client, request, timeout: float):
        """Await a service call on the executor spinning this node elsewhere.

        A sleep rather than a spin: these clients are used from a REPL or a
        streaming loop on the main thread while `cli.spin_background` drives the
        executor, and spinning here would be the second spinner.
        """
        future = client.call_async(request)
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.02)
        return future.result() if future.done() else None
