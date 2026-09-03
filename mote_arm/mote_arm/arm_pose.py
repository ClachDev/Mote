"""Teach and replay named arm poses.

A client of the ros2_control stack (reads ``/joint_states``, commands
``arm_controller``), so it never opens the serial bus and cannot contend with
the wheels — see ``mote_arm/control.py``.

    pixi run arm-pose save <name>   # capture the arm's current pose
    pixi run arm-pose list          # show taught poses (and current offset)
    pixi run arm-pose go <name>     # move to a taught pose
    pixi run arm-pose delete <name>

``save`` moves nothing — pose the limp arm by hand, then capture it. It stores
each joint clamped into its soft band and says which ones it held there: posing
by hand means posing against the mechanical stops, and the soft limits sit a
margin inside those, so a raw capture is routinely a fraction outside the band
and could never be replayed. ``go`` is the only command that moves the arm, and
it leaves the arm *holding* the pose it reached (deactivate ``arm_controller``,
or run ``arm-jog`` and ``torque off``, to make it limp again): it reports the
distance each joint will travel and then moves. There is no confirmation: the
move is bounded by ``--speed`` and supervised by ``--max-lag``, the destination
is a pose the operator taught and `save` already clamped into the soft limits,
and a prompt on every bench move is a keypress that buys none of that. Goals are
clamped to the soft limits here *and* in the driver.

There is no ceiling on how far a `go` may move. Distance is not what makes a
move risky once setpoints are streamed: the arm advances at ``--speed``
whatever the distance, so a long move is a slow move rather than a violent one,
and ``--max-lag`` stops it if the arm falls behind. A distance limit was a proxy
for the lurch that streaming removed, and with real calibrated joints it refused
the ordinary case — teach a pose, let go, watch the limp arm fall to rest,
replay. ``episode-replay`` keeps one for a different job: there a long approach
means the arm is not where the recording starts, so the replay will not
reproduce.

The arm is limp whenever no controller holds it, so it falls to rest the moment
you let go of it. A `go` straight after a `save` therefore starts from the rest
position and not from the pose just taught, and the travel it reports is real.

``go`` streams setpoints at a fixed rate rather than commanding the destination
in one jump, so the arm moves continuously at ``--speed`` instead of lurching,
and it stops if the arm falls behind its setpoint by more than ``--max-lag``.
The lag is measured against ``/joint_states``, which the hardware refreshes one
arm joint per control cycle (~8 Hz per joint) to stay inside the bus budget it
shares with the wheels, so lag is detected within a few setpoints rather than
instantly.
"""

from __future__ import annotations

import argparse
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from mote_arm import cli, config, poses
from mote_arm.calibrate import DEFAULT_MARGIN
from mote_arm.control import ArmControl
from mote_arm.motion import LagSupervisor, lag_of


class PoseClient(Node):
    def __init__(self):
        super().__init__("arm_pose")
        self.cfg = config.load()
        self._measured: dict[str, float] = {}
        self._lock = threading.Lock()
        self.create_subscription(JointState, "joint_states", self._on_states, 10)
        self.arm = ArmControl(self)

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

    def send(self, joints: dict[str, float], seconds: float) -> bool:
        """Command one streamed setpoint, taking hold of the arm if it is limp."""
        return self.arm.send(joints, seconds)


def _require_states(node: PoseClient) -> dict[str, float]:
    if not node.wait_for_states():
        raise SystemExit(
            "no /joint_states for all arm joints — is a stack that owns the "
            "servo bus running (`pixi run arm`, or `pixi run robot`)?"
        )
    return node.current()


def _cmd_save(node: PoseClient, args) -> None:
    """Capture the arm's pose, clamped into the band it can actually be sent to.

    Posing by hand means posing a limp arm against its mechanical stops, and the
    soft limits sit a margin (0.05 rad) inside those, so a captured position is
    routinely a fraction past the band. Stored raw, that pose can never be
    reached: every `go` clamps it and says so, which is a warning the operator
    receives minutes later and cannot act on. Stored clamped, it is reachable by
    construction and differs from what was posed by at most the margin.

    A joint further out than the margin is a different matter -- it means the
    arm and arm.yaml disagree about where that joint's limits are -- so it is
    called out separately rather than folded into the same quiet clamp.
    """
    current = _require_states(node)
    clamped: dict[str, float] = {}
    held: dict[str, float] = {}
    suspect: list[str] = []
    for name, value in current.items():
        try:
            joint = node.cfg.joint(name)
        except KeyError:
            clamped[name] = value
            continue
        clamped[name] = joint.clamp_rad(value)
        if clamped[name] != value:
            held[name] = value
            # DEFAULT_MARGIN rather than the joint's own: arm.yaml records the
            # margin per joint but JointSpec does not carry it, and the two
            # differ only if someone passed --margin to arm-setup calibrate.
            if abs(clamped[name] - value) > DEFAULT_MARGIN:
                suspect.append(name)

    path = poses.save_pose(args.name, clamped)
    print(f"saved pose {args.name!r} to {path}")
    for name in node.cfg.names:
        note = (
            f"  <- held at the soft limit, from {held[name]:+.4f}"
            if name in held
            else ""
        )
        print(f"  {name:<14} {clamped[name]:+.4f} rad{note}")
    if held:
        print(
            f"\n{len(held)} joint(s) were posed past their soft limits and are "
            "stored at the limit, so this pose is reachable as saved. Posing a "
            "limp arm against its stops does this: the soft limits sit a margin "
            "inside them."
        )
    if suspect:
        print(
            f"\n{', '.join(suspect)}: further out than the calibration margin, "
            "so the arm and $MOTE_HOME/arm.yaml disagree about this joint's "
            "limits. `pixi run arm-setup calibrate` re-measures them."
        )


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
            f"zero: {joint.zero_counts:>4}, invert: {str(joint.invert).lower()}}}{note}"
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

    _stream(node, current, goals, args)

    final = node.current()
    print("\nfinal pose:")
    for name in node.cfg.names:
        if name in goals:
            err = final.get(name, float("nan")) - goals[name]
            print(
                f"  {name:<14} {final.get(name, float('nan')):+.4f} rad "
                f"(target {goals[name]:+.4f}, err {err:+.4f})"
            )


SETPOINT_RATE_HZ = 20.0


def _stream(node: PoseClient, start: dict, goals: dict, args) -> None:
    """Walk to ``goals`` by streaming setpoints at a fixed rate.

    Waiting for each waypoint to *settle* before issuing the next makes the arm
    move in visible stop-start hops. Streaming a fresh setpoint every tick keeps
    the servo's target always just ahead of where it is, so the motion is
    continuous and its speed is set by ``--speed`` rather than by how quickly
    each hop happens to converge.

    Supervision is by *lag*: how far the arm trails the setpoint it was given.
    Lag that stays above ``--max-lag`` means the arm is no longer keeping up —
    the same stall condition as before, detected without pausing.
    """
    period = 1.0 / SETPOINT_RATE_HZ
    stream = poses.interpolate(start, goals, max(1e-4, args.speed / SETPOINT_RATE_HZ))
    print(
        f"\nstreaming {len(stream)} setpoints at {SETPOINT_RATE_HZ:.0f} Hz "
        f"({args.speed:.2f} rad/s), stopping if lag exceeds {args.max_lag:.2f} rad"
    )

    supervisor = LagSupervisor(args.max_lag, args.stall_time)
    report_every = max(1, len(stream) // 8)
    for i, setpoint in enumerate(stream, 1):
        node.send(setpoint, period)
        time.sleep(period)
        now = node.current()
        lag = lag_of(setpoint, now)
        if not supervisor.update(lag, period):
            print(
                f"\nSTOPPED at setpoint {i}/{len(stream)}: the arm trailed by "
                f"{lag:.3f} rad for {args.stall_time:.1f}s. Holding here rather "
                "than driving against a load it is not overcoming."
            )
            return
        if i % report_every == 0 or i == len(stream):
            print(
                f"  {i:>4}/{len(stream)}  lag {lag:.4f} rad   "
                + " ".join(
                    f"{n.split('_')[0]}={now.get(n, float('nan')):+.3f}"
                    for n in sorted(setpoint)
                )
            )

    # The stream ends exactly on target; give the servo a moment to close the
    # remaining proportional droop before reporting the final pose.
    deadline = time.time() + args.timeout
    while time.time() < deadline:
        time.sleep(0.1)
        now = node.current()
        err = max((abs(now.get(n, goals[n]) - goals[n]) for n in goals), default=0.0)
        if err < args.tolerance:
            print(f"  settled within {err:.4f} rad")
            return
    print(f"  holding at {err:.4f} rad of residual droop")


def build_parser() -> argparse.ArgumentParser:
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
    p_go.add_argument(
        "--speed",
        type=float,
        default=0.5,
        help="radians per second the streamed setpoint advances (default 0.5)",
    )
    p_go.add_argument(
        "--max-lag",
        type=float,
        default=0.15,
        help="radians the arm may trail its setpoint before it counts as not "
        "keeping up (default 0.15)",
    )
    p_go.add_argument(
        "--stall-time",
        type=float,
        default=1.5,
        help="seconds of sustained lag before stopping (default 1.5)",
    )
    p_go.add_argument(
        "--tolerance",
        type=float,
        default=0.06,
        help="radians of residual error treated as settled (default 0.06)",
    )
    p_go.add_argument("--timeout", type=float, default=5.0)
    p_go.set_defaults(func=_cmd_go)
    return parser


def main() -> None:
    args = cli.parse(build_parser())

    rclpy.init()
    node = PoseClient()
    spinner = cli.spin_background(node)
    try:
        args.func(node, args)
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
    finally:
        cli.shutdown(node, spinner)


if __name__ == "__main__":
    main()
