"""Teach and replay named arm poses.

A client of ``arm_driver`` (reads ``/joint_states``, publishes ``arm/goal``), so
it never opens the serial bus and cannot contend with the driver.

    pixi run arm-pose save <name>   # capture the arm's current pose
    pixi run arm-pose list          # show taught poses (and current offset)
    pixi run arm-pose go <name>     # move to a taught pose
    pixi run arm-pose delete <name>

``save`` is read-only — pose the limp arm by hand, then capture it. ``go`` is
the only command that moves the arm: it reports the distance each joint will
travel and requires confirmation unless ``--yes`` is given. Goals are clamped to
the robot.yaml soft limits here *and* in the driver.
"""

from __future__ import annotations

import argparse
import sys
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from mote_arm import config, poses


class PoseClient(Node):
    def __init__(self):
        super().__init__("arm_pose")
        self.cfg = config.load()
        self._measured: dict[str, float] = {}
        self._lock = threading.Lock()
        self.create_subscription(JointState, "joint_states", self._on_states, 10)
        self._pub = self.create_publisher(JointState, "arm/goal", 10)

    def _on_states(self, msg: JointState) -> None:
        arm_names = set(self.cfg.names)
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                if name in arm_names:
                    self._measured[name] = pos

    def current(self) -> dict[str, float]:
        with self._lock:
            return dict(self._measured)

    def wait_for_states(self, timeout: float = 5.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if len(self.current()) == len(self.cfg.names):
                return True
            time.sleep(0.05)
        return False

    def send(self, joints: dict[str, float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(joints)
        msg.position = [joints[n] for n in msg.name]
        self._pub.publish(msg)


def _require_states(node: PoseClient) -> dict[str, float]:
    if not node.wait_for_states():
        raise SystemExit(
            "no /joint_states for all arm joints — is `pixi run arm` running?"
        )
    return node.current()


def _cmd_save(node: PoseClient, args) -> None:
    current = _require_states(node)
    path = poses.save_pose(args.name, current)
    print(f"saved pose {args.name!r} to {path}")
    for name in node.cfg.names:
        print(f"  {name:<14} {current[name]:+.4f} rad")


def _cmd_list(node: PoseClient, args) -> None:
    taught = poses.load_poses()
    if not taught:
        print(f"no poses taught yet (would live in {poses.poses_path()})")
        return
    current = node.current() if node.wait_for_states(timeout=2.0) else {}
    for name, joints in sorted(taught.items()):
        print(f"\n{name}:")
        for joint, value in joints.items():
            if joint in current:
                delta = current[joint] - value
                print(f"  {joint:<14} {value:+.4f} rad   (now {delta:+.4f} away)")
            else:
                print(f"  {joint:<14} {value:+.4f} rad")


def _cmd_delete(node: PoseClient, args) -> None:
    if poses.delete_pose(args.name):
        print(f"deleted pose {args.name!r}")
    else:
        raise SystemExit(f"no pose named {args.name!r}")


def _cmd_limits(node: PoseClient, args) -> None:
    """Emit robot.yaml soft limits spanning every taught pose."""
    taught = poses.load_poses()
    if not taught:
        raise SystemExit("no poses taught yet — capture some with `arm-pose save`")

    band = poses.envelope(taught, margin=args.margin)
    print(
        f"# soft limits spanning {len(taught)} taught pose(s): "
        f"{', '.join(sorted(taught))}"
    )
    print(
        f"# margin {args.margin:+.3f} rad; joints with no taught position keep "
        "their current limits"
    )
    print("  joints:")
    for joint in node.cfg.joints:
        if joint.name in band:
            lo, hi = band[joint.name]
            note = ""
        else:
            lo, hi = joint.min_rad, joint.max_rad
            note = "  # unchanged (not in any taught pose)"
        print(
            f"    - {{name: {joint.name + ',':<16} id: {joint.id}, "
            f"min: {lo:>7.3f}, max: {hi:>7.3f}, "
            f"home: {joint.home_counts:>4}, invert: {str(joint.invert).lower()}}}{note}"
        )

    print("\n# sanity check against the taught poses:")
    for pose_name, joints in sorted(taught.items()):
        outside = [
            n
            for n, v in joints.items()
            if n in band and not (band[n][0] <= v <= band[n][1])
        ]
        status = "OK" if not outside else f"OUTSIDE for {outside}"
        print(f"#   {pose_name}: {status}")


def _cmd_go(node: PoseClient, args) -> None:
    taught = poses.load_poses()
    if args.name not in taught:
        raise SystemExit(
            f"no pose named {args.name!r} (have: {sorted(taught) or 'none'})"
        )
    target = taught[args.name]
    current = _require_states(node)

    goals: dict[str, float] = {}
    print(f"move to pose {args.name!r}:")
    largest = 0.0
    for joint_name, value in target.items():
        try:
            joint = node.cfg.joint(joint_name)
        except KeyError:
            print(f"  {joint_name:<14} SKIPPED (not an arm joint any more)")
            continue
        clamped = joint.clamp_rad(value)
        now = current.get(joint_name)
        travel = abs(clamped - now) if now is not None else float("nan")
        largest = max(largest, travel if travel == travel else 0.0)
        note = "  (CLAMPED)" if clamped != value else ""
        print(
            f"  {joint_name:<14} {now:+.4f} -> {clamped:+.4f} rad "
            f"(travel {travel:.4f}){note}"
        )
        goals[joint_name] = clamped

    print(f"largest single-joint travel: {largest:.4f} rad")
    if largest > args.max_travel:
        raise SystemExit(
            f"refusing: travel {largest:.4f} rad exceeds --max-travel "
            f"{args.max_travel:.4f}. Re-run with a larger --max-travel if that "
            "is genuinely intended."
        )

    if not args.yes:
        reply = input("proceed? [y/N] ").strip().lower()
        if reply not in ("y", "yes"):
            print("aborted; nothing sent")
            return

    node.send(goals)
    print("goal sent; watching for settle...")
    deadline = time.time() + args.timeout
    while time.time() < deadline:
        time.sleep(0.25)
        now = node.current()
        worst = max((abs(now.get(n, goals[n]) - goals[n]) for n in goals), default=0.0)
        if worst < 0.02:
            print(f"settled (max error {worst:.4f} rad)")
            break
    else:
        now = node.current()
        worst = max((abs(now.get(n, goals[n]) - goals[n]) for n in goals), default=0.0)
        print(f"did NOT settle within {args.timeout}s (max error {worst:.4f} rad)")
    for name in node.cfg.names:
        if name in goals:
            print(f"  {name:<14} {node.current().get(name, float('nan')):+.4f} rad")


def main() -> None:
    parser = argparse.ArgumentParser(description="Teach and replay arm poses")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_save = sub.add_parser("save", help="capture the current pose")
    p_save.add_argument("name")
    p_save.set_defaults(func=_cmd_save)

    p_list = sub.add_parser("list", help="list taught poses")
    p_list.set_defaults(func=_cmd_list)

    p_del = sub.add_parser("delete", help="remove a pose")
    p_del.add_argument("name")
    p_del.set_defaults(func=_cmd_delete)

    p_lim = sub.add_parser(
        "limits", help="emit robot.yaml soft limits spanning the taught poses"
    )
    p_lim.add_argument(
        "--margin",
        type=float,
        default=0.10,
        help="radians of headroom beyond the taught extremes (default 0.10)",
    )
    p_lim.set_defaults(func=_cmd_limits)

    p_go = sub.add_parser("go", help="move to a taught pose")
    p_go.add_argument("name")
    p_go.add_argument("--yes", action="store_true", help="skip confirmation")
    p_go.add_argument(
        "--max-travel",
        type=float,
        default=0.35,
        help="refuse if any joint would move more than this many rad (default 0.35)",
    )
    p_go.add_argument("--timeout", type=float, default=10.0)
    p_go.set_defaults(func=_cmd_go)

    args = parser.parse_args()

    rclpy.init()
    node = PoseClient()

    def _spin() -> None:
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        except Exception:  # noqa: BLE001 - see arm_driver.main
            if rclpy.ok():
                raise

    threading.Thread(target=_spin, daemon=True).start()
    try:
        args.func(node, args)
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
