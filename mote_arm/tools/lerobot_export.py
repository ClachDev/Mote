"""Turn on-robot captures into a LeRobot dataset. Runs off-board, not on the Pi.

The robot records a capture (``mote_arm/episode.py``): JSON lines plus the
camera's compressed frames, written with nothing but the standard library. This
converts that into a real ``LeRobotDataset`` — parquet shards, MP4 video, the
metadata LeRobot's loaders and viewers expect.

**It writes the dataset through LeRobot's own API rather than emitting the files
itself.** The format has already moved once (v2.1's file-per-episode became
v3.0's aggregated shards) and will move again; a hand-rolled writer would be a
second implementation of someone else's schema, silently wrong the first time it
changed. Using ``LeRobotDataset.create`` / ``add_frame`` / ``save_episode`` /
``finalize`` means "valid" is whatever the installed LeRobot says it is.

That API brings torch, ffmpeg and the HuggingFace stack with it, which is
exactly what the Pi does not carry — hence its own pixi environment, the same
split ``mote_perception`` makes for GPU inference::

    pixi run -e lerobot arm-export -- --capture ~/.mote/episodes/teleop \\
        --repo-id mote/teleop-demo

Then inspect it with LeRobot's own tooling::

    pixi run -e lerobot -- lerobot-dataset-viz --repo-id mote/teleop-demo \\
        --root <out> --episode-index 0

``--dry-run`` needs none of that: it reports the schema and the resampled frame
counts using only the capture, which is how the conversion is checked on a
machine that has no LeRobot.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# The capture format is defined once, in the ROS package. The exporter is
# ROS-free and runs in an environment with no ROS at all, so it imports that one
# module by path rather than keeping a second copy of the layout in step.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mote_arm.episode import (  # noqa: E402
    DatasetSpec,
    Episode,
    list_episodes,
    load_dataset_spec,
    load_episode,
    resample,
)

STATE_KEY = "observation.state"
ACTION_KEY = "action"


def features_for(spec: DatasetSpec, image_shape: tuple[int, int, int] | None) -> dict:
    """The LeRobot feature schema for a capture.

    Joint names go in ``names`` so a dataset stays self-describing: which column
    is the elbow is otherwise only recoverable from the robot.yaml of the day.
    """
    joints = list(spec.joints)
    features = {
        STATE_KEY: {"dtype": "float32", "shape": (len(joints),), "names": joints},
        ACTION_KEY: {"dtype": "float32", "shape": (len(joints),), "names": joints},
    }
    if spec.camera is not None and image_shape is not None:
        features[f"observation.images.{spec.camera.key}"] = {
            "dtype": "video",
            "shape": image_shape,
            "names": ["height", "width", "channels"],
        }
    return features


def _image_shape(episode: Episode) -> tuple[int, int, int] | None:
    """Read one frame to learn the camera's resolution."""
    from PIL import Image

    for frame in episode.frames:
        path = episode.image_path(frame)
        if path is not None and path.exists():
            with Image.open(path) as img:
                width, height = img.size
            return (height, width, 3)
    return None


def _load_rgb(path: Path):
    import numpy as np
    from PIL import Image

    with Image.open(path) as img:
        return np.asarray(img.convert("RGB"), dtype=np.uint8)


def plan(
    capture: Path, fps: int | None
) -> tuple[DatasetSpec, list[tuple[Path, int, int]]]:
    """What the export will do: the spec, and each episode's raw/resampled counts."""
    spec = load_dataset_spec(capture)
    if fps is not None:
        spec = DatasetSpec(
            name=spec.name,
            fps=fps,
            joints=spec.joints,
            robot_type=spec.robot_type,
            camera=spec.camera,
        )
    rows = []
    for path in list_episodes(capture):
        episode = load_episode(path)
        rows.append(
            (path, len(episode.frames), len(resample(episode.frames, spec.fps)))
        )
    return spec, rows


def export(args) -> Path:
    import numpy as np
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    capture = Path(args.capture)
    spec, planned = plan(capture, args.fps)
    episode_paths = [path for path, _, _ in planned]
    if not episode_paths:
        raise SystemExit(f"no episodes in {capture}")

    image_shape = None
    if spec.camera is not None:
        image_shape = _image_shape(load_episode(episode_paths[0]))
        if image_shape is None:
            raise SystemExit(
                f"{capture} declares camera {spec.camera.key!r} but holds no frames "
                "— re-record, or export a state-only capture"
            )

    features = features_for(spec, image_shape)
    image_key = f"observation.images.{spec.camera.key}" if image_shape else None
    root = Path(args.output) if args.output else capture / "lerobot"
    if root.exists() and any(root.iterdir()):
        raise SystemExit(
            f"{root} already exists and is not empty — pass --output elsewhere"
        )

    dataset = LeRobotDataset.create(
        repo_id=args.repo_id,
        fps=spec.fps,
        features=features,
        root=root,
        robot_type=spec.robot_type,
        use_videos=not args.images,
    )
    try:
        for path in episode_paths:
            episode = load_episode(path)
            frames = resample(episode.frames, spec.fps)
            if not frames:
                print(f"skipping {path.name}: no frames")
                continue
            task = episode.task or args.task
            if not task:
                raise SystemExit(
                    f"{path.name} has no task string — pass --task to supply one"
                )
            for frame in frames:
                row = {
                    STATE_KEY: np.asarray(frame.state, dtype=np.float32),
                    ACTION_KEY: np.asarray(frame.action, dtype=np.float32),
                    "task": task,
                }
                if image_key is not None:
                    image = episode.image_path(frame)
                    if image is None or not image.exists():
                        raise SystemExit(
                            f"{path.name} frame at t={frame.t:.3f} has no image, but the "
                            "capture declares a camera"
                        )
                    row[image_key] = _load_rgb(image)
                dataset.add_frame(row)
            dataset.save_episode()
            print(f"  {path.name}: {len(frames)} frames, task {task!r}")
    finally:
        # Without finalize the parquet footers are never written and the dataset
        # will not load — including after a failure partway through.
        dataset.finalize()
    return root


def verify(repo_id: str, root: Path) -> None:
    """Load the dataset back through LeRobot and report what it holds.

    Writing through LeRobot's API is not by itself proof the result loads: the
    footers are written at ``finalize`` and the video shards are encoded
    afterwards. Reading one sample back is the check that costs a second and
    catches that, so "valid dataset" is something the tool demonstrates rather
    than asserts.
    """
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    dataset = LeRobotDataset(repo_id, root=root)
    meta = dataset.meta
    print(
        f"\nverified: {meta.total_episodes} episode(s), {meta.total_frames} frames "
        f"at {meta.fps} fps, robot {meta.robot_type}"
    )
    sample = dataset[0]
    for key in sorted(meta.features):
        value = sample.get(key)
        shape = tuple(value.shape) if hasattr(value, "shape") else type(value).__name__
        print(f"  {key:<32} {shape}")
    print(f"  {'task':<32} {sample.get('task')!r}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Export arm captures to a LeRobot dataset"
    )
    parser.add_argument(
        "--capture", required=True, help="$MOTE_HOME/episodes/<dataset>"
    )
    parser.add_argument(
        "--repo-id", default=None, help="LeRobot repo id (default mote/<capture name>)"
    )
    parser.add_argument(
        "--output", default=None, help="dataset root (default <capture>/lerobot)"
    )
    parser.add_argument(
        "--fps", type=int, default=None, help="override the capture's fps"
    )
    parser.add_argument(
        "--task", default=None, help="task string for episodes recorded without one"
    )
    parser.add_argument(
        "--images",
        action="store_true",
        help="store frames as images instead of encoding video (no ffmpeg needed)",
    )
    parser.add_argument(
        "--dry-run", action="store_true", help="report the plan, import nothing"
    )
    parser.add_argument(
        "--no-verify",
        dest="verify",
        action="store_false",
        help="skip loading the dataset back after writing it",
    )
    args = parser.parse_args()

    capture = Path(args.capture)
    spec, planned = plan(capture, args.fps)
    if args.repo_id is None:
        args.repo_id = f"mote/{spec.name}"

    print(f"capture:  {capture}")
    print(f"repo id:  {args.repo_id}")
    print(f"fps:      {spec.fps}    robot: {spec.robot_type}")
    print(f"joints:   {', '.join(spec.joints)}")
    print(f"camera:   {spec.camera.key if spec.camera else 'none'}")
    for path, raw, gridded in planned:
        note = "" if raw == gridded else f"  (resampled from {raw})"
        print(f"  {path.name}: {gridded} frames{note}")

    if args.dry_run:
        print("\ndry run — nothing written")
        return

    root = export(args)
    print(f"\nwrote {root}")
    if args.verify:
        verify(args.repo_id, root)
    print(
        f"inspect it:  pixi run -e lerobot -- lerobot-dataset-viz "
        f"--repo-id {args.repo_id} --root {root} --episode-index 0"
    )


if __name__ == "__main__":
    main()
