"""Record teleoperated episodes: what the arm saw, where it was, what it was told.

An episode is a demonstration — the raw material a policy is later learned from —
so what gets stored is fixed by what a policy needs at inference time:

    observation.state           the arm's measured joint positions
    observation.images.<key>    the camera frame, stored exactly as published
    action                      the joint positions it was commanded to reach

The action is what reached ``arm_controller`` — the mirror's output, not the
leader's pose: a policy replaces the thing that produces goals, so the goals are
the thing to imitate. It is read off the trajectory topic rather than from the
mirror, so a session driven by ``arm-jog`` or by anything else records just as
well. Before the first goal of an episode arrives the action is the measured
state — "stay where you are" is what the arm was, in fact, being told.

Sampling is timer-driven at the dataset's fps and takes the most recent value of
each input, so a 10 Hz camera under a 20 Hz recording repeats frames rather than
leaving holes. That is the same thing a real teleop rig does, and it keeps every
row complete.

Interactive by default — ENTER starts and stops an episode, so both hands are
free for the arm between takes. ``--duration`` records fixed-length episodes
without a human, which is how the mock-arm tests and the bench script drive it.
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, JointState
from trajectory_msgs.msg import JointTrajectory

from mote_arm import cli, config
from mote_arm.control import TRAJECTORY_TOPIC
from mote_arm.episode import CameraSpec, DatasetSpec, EpisodeWriter, episodes_root

DEFAULT_CAMERA_TOPIC = "/image_raw/compressed"
DEFAULT_CAMERA_KEY = "front"


def encoding_of(image_format: str) -> str:
    """The file extension for a CompressedImage, read off its format field.

    image_transport publishes ``"<pixel format>; <codec> compressed <...>"``. We
    store the bytes untouched, so all we need from that is what to call the file.
    """
    lowered = image_format.lower()
    for codec in ("jpeg", "jpg", "png", "webp", "tiff"):
        if codec in lowered:
            return "jpeg" if codec == "jpg" else codec
    return "bin"


class EpisodeRecorder(Node):
    def __init__(self, args):
        super().__init__("episode_record")
        self.declare_parameter("robot_yaml", "")
        path = self.get_parameter("robot_yaml").get_parameter_value().string_value
        self.cfg = config.ArmConfig.from_yaml_file(path) if path else config.load()

        self.args = args
        self.root = Path(args.root) if args.root else episodes_root() / args.dataset
        self._lock = threading.Lock()
        self._state: dict[str, float] = {}
        self._action: dict[str, float] = {}
        self._image: bytes | None = None
        self._image_encoding: str | None = None
        self.writer: EpisodeWriter | None = None
        self.dropped = 0

        self.create_subscription(JointState, "joint_states", self._on_states, 10)
        self.create_subscription(JointTrajectory, TRAJECTORY_TOPIC, self._on_goal, 10)
        if args.camera:
            self.create_subscription(
                CompressedImage, args.camera_topic, self._on_image, 5
            )

        self.create_timer(1.0 / args.fps, self._tick)

    def _on_states(self, msg: JointState) -> None:
        names = set(self.cfg.names)
        with self._lock:
            for name, position in zip(msg.name, msg.position):
                if name in names:
                    self._state[name] = position

    def _on_goal(self, msg: JointTrajectory) -> None:
        """Record where a commanded trajectory ends up.

        Only the final point matters: the action a policy learns is the pose the
        arm was asked to be in, not the interpolation used to get there.
        """
        if not msg.points:
            return
        names = set(self.cfg.names)
        with self._lock:
            for name, position in zip(msg.joint_names, msg.points[-1].positions):
                if name in names:
                    self._action[name] = position

    def _on_image(self, msg: CompressedImage) -> None:
        with self._lock:
            self._image = bytes(msg.data)
            self._image_encoding = encoding_of(msg.format)

    def sample(self) -> tuple[list[float], list[float], bytes | None] | None:
        """The current row, or None if the arm is not fully reporting."""
        with self._lock:
            if any(name not in self._state for name in self.cfg.names):
                return None
            state = [self._state[n] for n in self.cfg.names]
            # No goal yet: the arm is being told to hold where it is.
            action = [self._action.get(n, self._state[n]) for n in self.cfg.names]
            return state, action, self._image

    def ready(self, timeout: float = 10.0) -> str | None:
        """Block until every input has been seen; returns a reason if it hasn't."""
        deadline = time.time() + timeout
        have_state = False
        while time.time() < deadline:
            with self._lock:
                have_state = all(n in self._state for n in self.cfg.names)
                have_image = self._image is not None
            if have_state and (have_image or not self.args.camera):
                return None
            time.sleep(0.1)
        if not have_state:
            missing = sorted(set(self.cfg.names) - set(self._state))
            return (
                f"no joint_states for {missing} — is a stack that owns the servo "
                "bus running (`pixi run arm`, or `pixi run robot`)?"
            )
        return (
            f"no frames on {self.args.camera_topic} — start the camera, or record "
            "state-only with --no-camera"
        )

    def start(self, task: str) -> EpisodeWriter:
        with self._lock:
            encoding = self._image_encoding or "jpeg"
        camera = (
            CameraSpec(
                key=self.args.camera_key,
                topic=self.args.camera_topic,
                encoding=encoding,
            )
            if self.args.camera
            else None
        )
        spec = DatasetSpec(
            name=self.args.dataset,
            fps=self.args.fps,
            joints=tuple(self.cfg.names),
            camera=camera,
        )
        self.dropped = 0
        writer = EpisodeWriter(self.root, spec, task)
        self.writer = writer
        return writer

    def stop(self, keep: bool = True) -> Path | None:
        writer, self.writer = self.writer, None
        if writer is None:
            return None
        if keep:
            return writer.close()
        writer.discard()
        return None

    def _tick(self) -> None:
        writer = self.writer
        if writer is None:
            return
        row = self.sample()
        if row is None:
            # A gap in joint_states must not become a silently wrong frame; drop
            # the tick and account for it, so a sparse episode is visible.
            self.dropped += 1
            return
        state, action, image = row
        writer.add(time.monotonic(), state, action, image if self.args.camera else None)


def _record_one(
    node: EpisodeRecorder, task: str, duration: float | None
) -> Path | None:
    writer = node.start(task)
    print(f"recording episode {writer.index} ...", flush=True)
    if duration is not None:
        time.sleep(duration)
        keep = True
    else:
        reply = input("ENTER to stop and keep, 'r' ENTER to discard: ").strip().lower()
        keep = reply != "r"
    path = node.stop(keep=keep)
    if path is None:
        print("discarded")
        return None
    frames = writer.count
    print(
        f"saved {path} — {frames} frames"
        + (f", {node.dropped} ticks dropped (no joint_states)" if node.dropped else "")
    )
    if frames == 0:
        print("warning: the episode is empty and will export as nothing")
    return path


def _session(node: EpisodeRecorder, args) -> None:
    camera = f", {args.camera_topic}" if args.camera else " (no camera)"
    print(f"dataset: {node.root}")
    print(f"task:    {args.task!r}")
    print(f"inputs:  joint_states, {TRAJECTORY_TOPIC}{camera}")
    problem = node.ready()
    if problem:
        raise SystemExit(problem)

    recorded = 0
    while args.episodes is None or recorded < args.episodes:
        if args.duration is None:
            reply = input("\nENTER to record episode, 'q' to finish: ").strip().lower()
            if reply == "q":
                break
        if _record_one(node, args.task, args.duration) is not None:
            recorded += 1
        if args.duration is not None and (
            args.episodes is None or recorded >= args.episodes
        ):
            break
    print(f"\n{recorded} episode(s) in {node.root}")
    print(f"export with:  pixi run -e lerobot arm-export -- --capture {node.root}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Record teleoperated arm episodes")
    parser.add_argument("--task", required=True, help="what the demonstration shows")
    parser.add_argument(
        "--dataset", default="teleop", help="capture name (default teleop)"
    )
    parser.add_argument(
        "--root",
        default=None,
        help="capture directory (default $MOTE_HOME/episodes/<dataset>)",
    )
    parser.add_argument(
        "--fps", type=int, default=20, help="sampling rate (default 20)"
    )
    parser.add_argument(
        "--episodes", type=int, default=None, help="stop after N episodes"
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="record fixed-length episodes without prompting (seconds)",
    )
    parser.add_argument("--camera-topic", default=DEFAULT_CAMERA_TOPIC)
    parser.add_argument(
        "--camera-key", default=DEFAULT_CAMERA_KEY, help="LeRobot feature suffix"
    )
    parser.add_argument(
        "--no-camera",
        dest="camera",
        action="store_false",
        help="record state and action only (the camera does not fit with the arm attached)",
    )
    args = cli.parse(parser)

    rclpy.init()
    node = EpisodeRecorder(args)

    spinner = cli.spin_background(node)
    try:
        _session(node, args)
    except (KeyboardInterrupt, EOFError):
        print("\ninterrupted", file=sys.stderr)
    finally:
        # An interrupted episode is still data: close it rather than lose it.
        path = node.stop(keep=True)
        if path is not None:
            print(f"closed in-progress episode: {path}")
        cli.shutdown(node, spinner)


if __name__ == "__main__":
    main()
