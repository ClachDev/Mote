"""Interactive per-joint jog CLI for the SO-101 follower arm.

A thin client of ``arm_driver`` (it publishes ``arm/goal`` and calls
``arm/set_torque``), so it never touches the serial bus itself and there is no
contention with the driver. Increments are clamped to the per-joint soft limits
from robot.yaml both here (for immediate feedback) and again in the driver
(authoritative). On exit it commands the arm limp — torque off.

Run the driver first (``pixi run arm``), then ``pixi run arm-jog`` in another
terminal.
"""

from __future__ import annotations

import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

from mote_arm import config
from mote_arm.config import JointSpec


def next_target(current: float, step: float, joint: JointSpec) -> float:
    """Advance ``current`` by ``step`` and clamp to the joint's soft limits."""
    return joint.clamp_rad(current + step)


class JogClient(Node):
    def __init__(self):
        super().__init__("arm_jog")
        self.declare_parameter("robot_yaml", "")
        path = self.get_parameter("robot_yaml").get_parameter_value().string_value
        self.cfg = config.ArmConfig.from_yaml_file(path) if path else config.load()

        self._measured: dict[str, float] = {}
        self._target: dict[str, float] = {}
        self._lock = threading.Lock()

        self._pub = self.create_publisher(JointState, "arm/goal", 10)
        self.create_subscription(JointState, "joint_states", self._on_states, 10)
        self._torque_cli = self.create_client(SetBool, "arm/set_torque")

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

    def wait_for_states(self, timeout: float = 2.0) -> bool:
        """Block briefly until the first /joint_states arrives."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                if self._measured:
                    return True
            time.sleep(0.05)
        return False

    def send(self, joint: JointSpec, rad: float) -> None:
        self._target[joint.name] = rad
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [joint.name]
        msg.position = [rad]
        self._pub.publish(msg)

    def set_torque(self, enable: bool) -> None:
        if not self._torque_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                "arm/set_torque unavailable — is arm_driver running?"
            )
            return
        req = SetBool.Request()
        req.data = enable
        self._torque_cli.call_async(req)


HELP = """
Commands:
  <n>            select joint by number
  + / -          jog selected joint by +step / -step
  step <rad>     set jog step (default 0.05 rad)
  zero           move selected joint to 0 rad (mid-travel, NOT the rest pose)
  torque on|off  enable (hold) / disable (limp) torque
  status         print all joints
  help           show this help
  quit           limp the arm and exit
""".rstrip()


def _print_status(node: JogClient, selected: int, step: float) -> None:
    print(f"\nstep = {step:.3f} rad")
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
        print("warning: no /joint_states yet — is 'pixi run arm' running?")
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
            node.set_torque(parts[1].lower() in ("on", "true", "1", "hold"))
        elif cmd == "status":
            _print_status(node, selected, step)
        else:
            print("unknown command; type 'help'")


def main() -> None:
    rclpy.init()
    node = JogClient()

    def _spin() -> None:
        # SIGINT surfaces here as ExternalShutdownException; the REPL thread
        # owns the exit path, so this one just stops quietly.
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        except Exception:  # noqa: BLE001
            # Context torn down by SIGINT (see arm_driver.main); a real error
            # is one that happened while the context was still valid.
            if rclpy.ok():
                raise

    spin = threading.Thread(target=_spin, daemon=True)
    spin.start()
    try:
        _repl(node)
    except KeyboardInterrupt:
        pass
    finally:
        print("\nlimping arm (torque off) and exiting...")
        node.set_torque(False)
        # Give the async torque-off a moment to reach the driver.
        time.sleep(0.2)
        rclpy.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
