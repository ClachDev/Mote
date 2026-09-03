"""Interactive per-joint jog CLI for the SO-101 follower arm.

A client of the ros2_control stack, not of the bus: it publishes single-point
trajectories to ``arm_controller/joint_trajectory`` and reads ``/joint_states``
from the joint_state_broadcaster, so it never opens the serial port and cannot
contend with the wheels. Increments are clamped to the per-joint soft limits
from robot.yaml both here (for immediate feedback) and again in the hardware
(authoritative).

"Torque" is controller activation. ``arm_controller`` is spawned *inactive*, so
the arm starts limp; activating it makes MoteHardware take hold of the arm's
current pose, and deactivating it drops torque. Jogging therefore activates on
demand, and exiting leaves the arm limp again.

Start a stack that owns the bus first — ``pixi run arm`` on the bench, or
``pixi run robot`` / ``mapping`` during a mission — then ``pixi run arm-jog``.
"""

from __future__ import annotations

import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from mote_arm import cli, config
from mote_arm.config import JointSpec
from mote_arm.control import ArmControl

# Trajectory speed for a jog move. Deliberately under the servos' own
# `moving_speed` cap (robot.yaml: 500 steps/s is ~0.77 rad/s) — a trajectory
# asking for more than the servo delivers just runs ahead of the hardware.
JOG_SPEED_RAD_S = 0.5
MIN_MOVE_TIME_S = 0.5


def next_target(current: float, step: float, joint: JointSpec) -> float:
    """Advance ``current`` by ``step`` and clamp to the joint's soft limits."""
    return joint.clamp_rad(current + step)


def move_time(delta: float) -> float:
    """Seconds to allow for a jog of ``delta`` radians."""
    return max(MIN_MOVE_TIME_S, abs(delta) / JOG_SPEED_RAD_S)


class JogClient(Node):
    def __init__(self):
        super().__init__("arm_jog")
        self.declare_parameter("robot_yaml", "")
        path = self.get_parameter("robot_yaml").get_parameter_value().string_value
        self.cfg = config.ArmConfig.from_yaml_file(path) if path else config.load()

        self._measured: dict[str, float] = {}
        self._target: dict[str, float] = {}
        self._lock = threading.Lock()

        self.arm = ArmControl(self)
        self.create_subscription(JointState, "joint_states", self._on_states, 10)

    def _on_states(self, msg: JointState) -> None:
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                self._measured[name] = pos

    def measured(self, name: str) -> float | None:
        with self._lock:
            return self._measured.get(name)

    def base_for(self, joint: JointSpec) -> float:
        """Base angle for the next jog: the last commanded target if this joint
        has been jogged, otherwise the live measured position (so the first jog
        steps from where the arm actually is, not from an assumed zero)."""
        if joint.name in self._target:
            return self._target[joint.name]
        meas = self.measured(joint.name)
        return meas if meas is not None else 0.0

    def wait_for_states(self, timeout: float = 5.0) -> bool:
        """Block briefly until /joint_states carries an arm joint.

        The joint_state_broadcaster publishes the wheels too, so waiting for any
        message at all would succeed even with the arm absent from the stack.
        """
        wanted = set(self.cfg.names)
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                if wanted & self._measured.keys():
                    return True
            time.sleep(0.05)
        return False

    def send(self, joint: JointSpec, rad: float) -> None:
        """Command one joint, taking hold of the arm first if it is still limp."""
        measured = self.measured(joint.name)
        delta = rad - measured if measured is not None else rad
        if self.arm.send({joint.name: rad}, move_time(delta)):
            self._target[joint.name] = rad


HELP = """
Commands:
  <n>            select joint by number
  + / -          jog selected joint by +step / -step
  step <rad>     set jog step (default 0.05 rad)
  zero           move selected joint to 0 rad (mid-travel, NOT the rest pose)
  torque on|off  hold (activate arm_controller) / limp (deactivate it)
  status         print all joints
  help           show this help
  quit           limp the arm and exit
""".rstrip()


def _print_status(node: JogClient, selected: int, step: float) -> None:
    print(f"\nstep = {step:.3f} rad   arm is {'HOLDING' if node.arm.held else 'LIMP'}")
    for i, joint in enumerate(node.cfg.joints):
        meas = node.measured(joint.name)
        meas_s = f"{meas:+.3f}" if meas is not None else "  ?  "
        marker = "->" if i == selected else "  "
        print(
            f" {marker} [{i}] {joint.name:<14} meas={meas_s} rad "
            f"target={node.base_for(joint):+.3f}  "
            f"limits=[{joint.min_rad:+.2f}, {joint.max_rad:+.2f}]"
        )


def _repl(node: JogClient) -> None:
    selected = 0
    step = 0.05
    print("SO-101 arm jog. Type 'help' for commands. Arm starts LIMP.")
    if not node.wait_for_states():
        print(
            "warning: no arm joints on /joint_states — is a stack that owns the "
            "bus running (`pixi run arm`, or `pixi run robot`)?"
        )
    _print_status(node, selected, step)
    while True:
        try:
            line = input("jog> ").strip()
        except EOFError:
            break
        if not line:
            continue
        parts = line.split()
        cmd = parts[0].lower()
        joint = node.cfg.joints[selected]

        if cmd in ("q", "quit", "exit"):
            break
        elif cmd in ("help", "h", "?"):
            print(HELP)
        elif cmd.isdigit():
            idx = int(cmd)
            if 0 <= idx < len(node.cfg.joints):
                selected = idx
            else:
                print(f"no joint {idx}")
            _print_status(node, selected, step)
        elif cmd == "step" and len(parts) == 2:
            try:
                step = abs(float(parts[1]))
            except ValueError:
                print("bad step")
        elif cmd in ("+", "-"):
            delta = step if cmd == "+" else -step
            tgt = next_target(node.base_for(joint), delta, joint)
            node.send(joint, tgt)
            print(f"{joint.name} -> {tgt:+.3f} rad")
        elif cmd in ("zero", "home"):
            if cmd == "home":
                # "home" is the name of a taught rest pose; 0 rad is the middle
                # of the joint's travel, a different place. Renamed rather than
                # removed, so the old reflex still works and says so.
                print("note: 'home' is now 'zero' — 0 rad is mid-travel.")
            tgt = joint.clamp_rad(0.0)
            node.send(joint, tgt)
            print(f"{joint.name} -> {tgt:+.3f} rad (zero)")
        elif cmd == "torque" and len(parts) == 2:
            node.arm.set_holding(parts[1].lower() in ("on", "true", "1", "hold"))
        elif cmd == "status":
            _print_status(node, selected, step)
        else:
            print("unknown command; type 'help'")


def main() -> None:
    rclpy.init()
    node = JogClient()
    spinner = cli.spin_background(node)
    try:
        _repl(node)
    except KeyboardInterrupt:
        pass
    finally:
        print("\nlimping arm (deactivating arm_controller) and exiting...")
        node.arm.set_holding(False)
        cli.shutdown(node, spinner)


if __name__ == "__main__":
    main()
