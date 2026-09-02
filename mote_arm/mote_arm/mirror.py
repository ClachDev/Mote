"""The mirror node: the only thing that turns a virtual leader into arm motion.

It subscribes to a leader pose, the arm's measured state and the e-stop flag,
and commands `arm_controller` through `mote_arm.control`. Every safety rule
lives in `mote_arm.teleop.LeaderMirror` (clamping, rate limiting, the deadman,
the panic latch) so it can be tested without a bus, a controller, or a terminal;
this node is the ROS wiring around it.

Keeping it separate from the frontend is what makes the frontend replaceable:
the keyboard leader, a slider GUI publishing `leader/joint_states`, or a
recorded episode being replayed are all the same thing from here.

**Panic is controller deactivation, and it latches.** Since `MoteHardware` takes
hold of the arm exactly when `arm_controller` claims its command interfaces,
dropping torque means deactivating the controller — the same switch `arm-jog`
uses. The latch then suppresses every goal until it is explicitly cleared, so
the arm cannot resume simply because input started arriving again.

**The tick loop runs on the main thread, not on a timer.** Taking hold of the
arm is a `switch_controller` service call, and a service call made from inside
an executor callback can never complete: the future is resolved by the executor
that the callback is currently blocking. `arm-jog` gets this right by driving
from its REPL thread; the mirror does the same with a plain loop while
`cli.spin_background` spins the node.
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from mote_arm import cli, config
from mote_arm.control import ArmControl
from mote_arm.teleop import ESTOPPED, HOLDING, TRACKING, LeaderMirror, MirrorLimits


def latched(depth: int = 1) -> QoSProfile:
    """Transient-local QoS for the e-stop flag.

    The latch has to outlive the process that set it: a mirror restarted while
    the arm is e-stopped must come up e-stopped, not come up following.
    """
    qos = QoSProfile(depth=depth)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return qos


class Diagnostics:
    """Where the motion is being lost, measured rather than reasoned about.

    Teleop that stutters or falls short of its range has three candidate causes
    and they are indistinguishable from the outside: the mirror not ticking at
    the rate it claims, leader poses arriving in gaps, or the arm not achieving
    the velocity it is being asked for. Each is a number, so each is printed:

      tick   how fast this loop really runs, and its worst period
      leader arrival rate of leader/joint_states, and the worst gap between two
      cmd    rate the commanded pose advances -- what the mirror is asking for
      arm    rate the measured pose advances -- what the arm actually did
      lag    how far the arm trails the command right now

    cmd at the rate limit with arm well below it, and a lag that grows, is the
    arm failing to follow. Both low is the mirror. A leader gap over the deadman
    is the input path. Reported for the joint that moved most over the window,
    since that is the one being driven.
    """

    def __init__(self, node: "ArmMirror", period: float = 0.5):
        self._node = node
        self._period = period
        self._reset(time.monotonic())
        self.leader_stamps: list[float] = []

    def _reset(self, now: float) -> None:
        self._start = now
        self._ticks = 0
        self._dt_max = 0.0
        self._last_tick = now
        self._commanded0 = self._node.mirror.commanded
        self._measured0 = self._node.mirror.measured
        self.leader_stamps = []

    def on_leader(self, now: float) -> None:
        self.leader_stamps.append(now)

    def tick(self, now: float) -> None:
        self._ticks += 1
        self._dt_max = max(self._dt_max, now - self._last_tick)
        self._last_tick = now
        elapsed = now - self._start
        if elapsed < self._period:
            return

        commanded = self._node.mirror.commanded
        measured = self._node.mirror.measured
        moved = {n: abs(v - self._commanded0.get(n, v)) for n, v in commanded.items()}
        joint = max(moved, key=moved.get, default=None)

        gaps = [b - a for a, b in zip(self.leader_stamps, self.leader_stamps[1:])]
        parts = [
            f"tick {self._ticks / elapsed:4.1f}Hz worst {self._dt_max * 1e3:5.1f}ms",
            f"leader {len(self.leader_stamps) / elapsed:4.1f}Hz "
            f"worst gap {max(gaps, default=0.0) * 1e3:5.1f}ms",
        ]
        if joint is not None:
            cmd_rate = moved[joint] / elapsed
            arm_rate = (
                abs(measured.get(joint, 0.0) - self._measured0.get(joint, 0.0))
                / elapsed
            )
            lag = abs(commanded[joint] - measured.get(joint, commanded[joint]))
            parts.append(
                f"{joint} cmd {cmd_rate:.3f} arm {arm_rate:.3f} rad/s  lag {lag:+.3f} rad"
            )
        parts.append(self._node.mirror.state)
        self._node.get_logger().info("diag  " + "  |  ".join(parts))
        self._reset(now)


class ArmMirror(Node):
    def __init__(self):
        super().__init__("arm_mirror")
        self.declare_parameter("robot_yaml", "")
        self.declare_parameter("rate", 20.0)
        self.declare_parameter("max_velocity", MirrorLimits.max_velocity)
        self.declare_parameter("deadman_timeout", MirrorLimits.deadman_timeout)
        # `pixi run arm-mirror --ros-args -p diagnose:=true`
        self.declare_parameter("diagnose", False)

        path = self.get_parameter("robot_yaml").get_parameter_value().string_value
        self.cfg = config.ArmConfig.from_yaml_file(path) if path else config.load()

        self.mirror = LeaderMirror(
            self.cfg.joints,
            MirrorLimits(
                max_velocity=self.get_parameter("max_velocity").value,
                deadman_timeout=self.get_parameter("deadman_timeout").value,
            ),
        )

        self.arm = ArmControl(self)
        self.create_subscription(JointState, "leader/joint_states", self._on_leader, 10)
        self.create_subscription(JointState, "joint_states", self._on_states, 10)
        self.create_subscription(Bool, "teleop/estop", self._on_estop, latched())

        self._reported = None
        self.diagnostics = (
            Diagnostics(self) if self.get_parameter("diagnose").value else None
        )
        self._estop_requested = False
        self.period = 1.0 / max(1.0, self.get_parameter("rate").value)

        limits = self.mirror.limits
        self.get_logger().info(
            f"arm_mirror up: max {limits.max_velocity:.2f} rad/s, deadman "
            f"{limits.deadman_timeout:.2f} s — leader/joint_states -> arm_controller"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_leader(self, msg: JointState) -> None:
        if self.diagnostics is not None:
            self.diagnostics.on_leader(time.monotonic())
        self.mirror.on_leader(dict(zip(msg.name, msg.position)), self._now())

    def _on_states(self, msg: JointState) -> None:
        self.mirror.on_measured(dict(zip(msg.name, msg.position)))

    def _on_estop(self, msg: Bool) -> None:
        # Recorded here, acted on in the loop: dropping torque is a service
        # call, which cannot complete from inside this callback.
        self._estop_requested = msg.data

    def _apply_estop(self) -> None:
        if self._estop_requested == self.mirror.estopped:
            return
        self.mirror.set_estop(self._estop_requested, self._now())
        if self._estop_requested:
            self.get_logger().warn("PANIC: dropping torque and refusing goals")
            if not self.arm.set_holding(False):
                self.get_logger().error(
                    "could not deactivate arm_controller — the arm may still be "
                    "holding; stop the control stack or cut power"
                )
        else:
            self.get_logger().info("panic cleared; following again")

    def tick(self) -> None:
        if self.diagnostics is not None:
            self.diagnostics.tick(time.monotonic())
        self._apply_estop()
        goal = self.mirror.update(self._now(), self.period)
        if goal:
            # One period to reach the point: the mirror has already rate-limited
            # the step to what that allows, and a trajectory the arm cannot
            # finish in time just runs ahead of the hardware.
            self.arm.send(goal, self.period)

        if self.mirror.state != self._reported:
            self._reported = self.mirror.state
            if self.mirror.state == HOLDING:
                self.get_logger().info("deadman: no leader input, holding position")
            elif self.mirror.state == TRACKING:
                self.get_logger().info("following the leader")
            elif self.mirror.state == ESTOPPED:
                self.get_logger().warn("e-stopped")

    def run(self) -> None:
        while rclpy.ok():
            self.tick()
            time.sleep(self.period)


def main() -> None:
    rclpy.init()
    node = ArmMirror()
    spinner = cli.spin_background(node)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        # Leave the arm limp: the mirror took hold of it, so the mirror gives it
        # back rather than leaving a torqued arm behind an exited process.
        node.arm.set_holding(False)
        cli.shutdown(node, spinner)


if __name__ == "__main__":
    main()
