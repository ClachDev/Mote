"""Replay a recorded episode on the arm.

Replay is the honest test of a recording: if the stored actions put the arm back
through the demonstrated motion, the episode really does contain what a policy
would need to learn from. It is also the first thing that will run a *policy's*
output, so it is built to be the safe version of that path from the start.

Three gates, in order:

1. **Reduced speed.** Actions are issued at ``fps * --speed-scale`` (a quarter
   of the recorded rate by default), so a replay is slow enough to watch and to
   interrupt. It replays the same *path*, not the same dynamics.
2. **Approach, then replay.** The arm is walked to the episode's first pose
   before anything is replayed, at a bounded speed and only after the operator
   has seen how far that is. Replaying from wherever the arm happens to be
   parked would put the first action a long way from it.
3. **Lag supervision.** The same rule that guards ``arm-pose go``: if the arm
   trails its setpoint for ``--stall-time``, the replay stops where it is rather
   than driving on against whatever is holding it.

Every action is clamped to the robot.yaml soft limits here and again in the
hardware, so an episode recorded before a limit was tightened cannot replay
outside the current envelope.

Stop the virtual leader before replaying — two things commanding
``arm_controller`` would fight over the arm.
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from mote_arm import cli, config, poses
from mote_arm.control import ArmControl
from mote_arm.episode import Episode, list_episodes, load_dataset_spec, load_episode
from mote_arm.motion import LagSupervisor, lag_of


class ReplayClient(Node):
    """A client of arm_controller — commands the arm, never touches the bus."""

    def __init__(self):
        super().__init__("episode_replay")
        self.declare_parameter("robot_yaml", "")
        path = self.get_parameter("robot_yaml").get_parameter_value().string_value
        self.cfg = config.ArmConfig.from_yaml_file(path) if path else config.load()
        self._lock = threading.Lock()
        self._measured: dict[str, float] = {}
        self.create_subscription(JointState, "joint_states", self._on_states, 10)
        self.arm = ArmControl(self)

    def _on_states(self, msg: JointState) -> None:
        names = set(self.cfg.names)
        with self._lock:
            for name, position in zip(msg.name, msg.position):
                if name in names:
                    self._measured[name] = position

    def measured(self) -> dict[str, float]:
        with self._lock:
            return dict(self._measured)

    def wait_for_states(self, timeout: float = 5.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if len(self.measured()) == len(self.cfg.names):
                return True
            time.sleep(0.05)
        return False

    def send(self, pose: dict[str, float], seconds: float) -> bool:
        """Command one setpoint, taking hold of the arm first if it is limp."""
        return self.arm.send(pose, seconds)


def _resolve(capture: Path, index: int | None) -> Path:
    """The episode directory to replay: named by index, or the only one there."""
    available = list_episodes(capture)
    if not available:
        raise SystemExit(f"no episodes in {capture}")
    if index is None:
        if len(available) > 1:
            names = ", ".join(p.name for p in available)
            raise SystemExit(
                f"{capture} holds several episodes ({names}) — pass --episode N"
            )
        return available[0]
    match = capture / f"episode_{index:03d}"
    if not match.exists():
        raise SystemExit(f"no episode {index} in {capture}")
    return match


def _pose_of(joints: tuple[str, ...], values) -> dict[str, float]:
    return dict(zip(joints, values))


def _stream(
    node: ReplayClient,
    setpoints: list[dict[str, float]],
    period: float,
    supervisor: LagSupervisor,
    label: str,
) -> bool:
    """Issue setpoints on a fixed period; False if the arm stopped keeping up."""
    report_every = max(1, len(setpoints) // 8)
    for i, setpoint in enumerate(setpoints, 1):
        if not node.send(setpoint, period):
            print(
                f"\nSTOPPED during {label} at {i}/{len(setpoints)}: could not take "
                "hold of the arm, so nothing further was commanded."
            )
            return False
        time.sleep(period)
        lag = lag_of(setpoint, node.measured())
        if not supervisor.update(lag, period):
            print(
                f"\nSTOPPED during {label} at {i}/{len(setpoints)}: the arm trailed "
                f"by {lag:.3f} rad for {supervisor.stall_time:.1f}s. Holding here "
                "rather than driving against a load it is not overcoming."
            )
            return False
        if i % report_every == 0 or i == len(setpoints):
            print(f"  {label} {i:>5}/{len(setpoints)}  lag {lag:.4f} rad")
    return True


def _summarise(episode: Episode, joints: tuple[str, ...], cfg) -> dict[str, float]:
    """Print what the episode will do and return its first (clamped) pose."""
    actions = [_pose_of(joints, frame.action) for frame in episode.frames]
    print(
        f"\nepisode {episode.index}: {len(episode.frames)} frames, {episode.duration:.1f}s"
    )
    print(f"task: {episode.task!r}")
    for name in joints:
        values = [pose[name] for pose in actions]
        joint = cfg.joint(name)
        clamped = any(joint.clamp_rad(v) != v for v in values)
        print(
            f"  {name:<14} {min(values):+.3f} .. {max(values):+.3f} rad"
            + ("   (CLAMPED to limits on replay)" if clamped else "")
        )
    return {n: cfg.joint(n).clamp_rad(v) for n, v in actions[0].items()}


def _run(node: ReplayClient, args) -> None:
    capture = Path(args.capture)
    spec = load_dataset_spec(capture)
    episode = load_episode(_resolve(capture, args.episode))
    if not episode.frames:
        raise SystemExit(f"episode {episode.path} has no frames")

    unknown = [n for n in spec.joints if n not in set(node.cfg.names)]
    if unknown:
        raise SystemExit(
            f"episode was recorded with joints {unknown} that this arm does not "
            "have — it belongs to a different robot.yaml"
        )

    if not node.wait_for_states():
        raise SystemExit(
            "no /joint_states for all arm joints — is a stack that owns the servo "
            "bus running (`pixi run arm`, or `pixi run robot`)?"
        )
    current = node.measured()
    start = _summarise(episode, spec.joints, node.cfg)

    approach = max(abs(start[n] - current[n]) for n in start if n in current)
    print(f"\napproach to the first pose: {approach:.3f} rad of travel")
    if approach > args.max_travel:
        raise SystemExit(
            f"refusing: the arm is {approach:.3f} rad from the episode's start, over "
            f"--max-travel {args.max_travel:.3f}. Move it closer, or raise the limit "
            "deliberately."
        )

    period = 1.0 / (spec.fps * args.speed_scale)
    print(
        f"replay at {args.speed_scale:.0%} of {spec.fps} fps "
        f"({1 / period:.1f} setpoints/s), stopping if lag exceeds {args.max_lag:.2f} rad"
    )
    if not args.yes and input("proceed? [y/N] ").strip().lower() not in ("y", "yes"):
        print("aborted; nothing sent")
        return

    supervisor = LagSupervisor(args.max_lag, args.stall_time)
    walk = poses.interpolate(current, start, max(1e-4, args.approach_speed / spec.fps))
    if walk and not _stream(node, walk, 1.0 / spec.fps, supervisor, "approach"):
        return

    setpoints = [
        {
            n: node.cfg.joint(n).clamp_rad(v)
            for n, v in _pose_of(spec.joints, frame.action).items()
        }
        for frame in episode.frames
    ]
    if not _stream(node, setpoints, period, supervisor, "replay"):
        return

    final = node.measured()
    print("\nfinal pose vs the episode's last action:")
    for name in spec.joints:
        target = setpoints[-1][name]
        print(
            f"  {name:<14} {final.get(name, float('nan')):+.4f} rad "
            f"(target {target:+.4f}, err {final.get(name, float('nan')) - target:+.4f})"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="Replay a recorded episode on the arm")
    parser.add_argument(
        "capture", help="capture directory ($MOTE_HOME/episodes/<dataset>)"
    )
    parser.add_argument("--episode", type=int, default=None, help="episode index")
    parser.add_argument("--yes", action="store_true", help="skip the confirmation")
    parser.add_argument(
        "--speed-scale",
        type=float,
        default=0.25,
        help="fraction of the recorded rate to replay at (default 0.25)",
    )
    parser.add_argument(
        "--approach-speed",
        type=float,
        default=0.3,
        help="rad/s for the walk to the episode's first pose (default 0.3)",
    )
    parser.add_argument(
        "--max-travel",
        type=float,
        default=0.35,
        help="refuse if the arm starts further than this from the episode's "
        "first pose (default 0.35). Not a limit on how fast or how far the arm "
        "may move -- --speed and --max-lag govern that -- but a check that the "
        "arm is where the recording began, since a replay from somewhere else "
        "will not reproduce it.",
    )
    parser.add_argument("--max-lag", type=float, default=0.15)
    parser.add_argument("--stall-time", type=float, default=1.5)
    args = cli.parse(parser)

    if not 0 < args.speed_scale <= 1.0:
        raise SystemExit(
            "--speed-scale must be in (0, 1]; replay never speeds an episode up"
        )

    rclpy.init()
    node = ReplayClient()

    spinner = cli.spin_background(node)
    try:
        _run(node, args)
    except KeyboardInterrupt:
        print("\ninterrupted — the arm holds its last setpoint", file=sys.stderr)
    finally:
        # Replay took hold of the arm, so replay gives it back: leaving a
        # torqued arm behind an exited process is how a bench session ends with
        # the arm holding a pose nobody is watching.
        node.arm.set_holding(False)
        cli.shutdown(node, spinner)


if __name__ == "__main__":
    main()
