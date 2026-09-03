"""The on-robot capture format for teleoperated episodes, and how to read it.

A recorded episode has to survive two very different consumers: a policy-learning
stack off-board, which wants a LeRobot dataset, and the robot itself, which wants
to replay the episode on the arm. Writing LeRobot's format directly on the robot
would put parquet, ffmpeg and (transitively) a large ML stack on a Pi that
deliberately carries none of it — the same reason inference runs off-board.

So the robot writes a **capture**: a plain directory of JSON lines plus the
camera's already-compressed frames, stored byte-for-byte as they were published.
Nothing decodes, re-encodes, or is even aware of an image format. The capture is
the replay source of truth, and ``mote_arm/tools/lerobot_export.py`` turns it
into a real LeRobot dataset off-board, using LeRobot's own writer so validity is
never our reimplementation of someone else's schema.

Layout, under ``$MOTE_HOME/episodes/<dataset>/``::

    dataset.json              fps, joint order, robot type, camera key/topic
    episode_000/
        episode.json          task string, frame count, duration, wall clock
        frames.jsonl          one JSON object per timestep
        frames/000000.jpg     the compressed image for that timestep

``frames.jsonl`` is append-only and flushed per row, so a recorder that is
killed mid-episode leaves a readable prefix rather than a corrupt file — the
episode is closed by reading back what actually landed.

ROS-free: the recorder, the replayer and the off-board exporter all share these
definitions, and the format's tests need no hardware.
"""

from __future__ import annotations

import json
from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path

CAPTURE_VERSION = 1

# LeRobot's convention for a single-arm follower; recorded so the exported
# dataset says what the data came from.
DEFAULT_ROBOT_TYPE = "so101_follower"


def episodes_root() -> Path:
    """Where captures live: per-robot state, like maps, zones and arm poses.

    ``mote_arm`` deliberately keeps its own reading of ``MOTE_HOME`` rather than
    importing ``mote_bringup``'s: the dependency runs `mote_bringup -> mote_arm`
    (the base launch resolves this arm's calibration into the URDF) and must not
    run back, or the two packages cannot be ordered. Imported inside the
    function because the off-board exporter reads captures by explicit path, in
    an environment with no ROS packages on it at all.
    """
    from mote_arm.poses import mote_home

    return mote_home() / "episodes"


@dataclass(frozen=True)
class CameraSpec:
    """One camera stream in a capture.

    ``key`` becomes the LeRobot feature suffix (``observation.images.<key>``).
    """

    key: str
    topic: str
    encoding: str = "jpeg"


@dataclass(frozen=True)
class DatasetSpec:
    """What every episode in one capture directory has in common."""

    name: str
    fps: int
    joints: tuple[str, ...]
    robot_type: str = DEFAULT_ROBOT_TYPE
    camera: CameraSpec | None = None
    version: int = CAPTURE_VERSION

    def to_dict(self) -> dict:
        out = {
            "version": self.version,
            "name": self.name,
            "fps": self.fps,
            "joints": list(self.joints),
            "robot_type": self.robot_type,
            "camera": None,
        }
        if self.camera is not None:
            out["camera"] = {
                "key": self.camera.key,
                "topic": self.camera.topic,
                "encoding": self.camera.encoding,
            }
        return out

    @staticmethod
    def from_dict(data: dict) -> "DatasetSpec":
        version = int(data.get("version", CAPTURE_VERSION))
        if version != CAPTURE_VERSION:
            raise ValueError(
                f"capture format version {version} is not {CAPTURE_VERSION} — "
                "recorded by a different version of mote_arm"
            )
        cam = data.get("camera")
        return DatasetSpec(
            name=str(data["name"]),
            fps=int(data["fps"]),
            joints=tuple(str(j) for j in data["joints"]),
            robot_type=str(data.get("robot_type", DEFAULT_ROBOT_TYPE)),
            camera=(
                CameraSpec(
                    key=str(cam["key"]),
                    topic=str(cam["topic"]),
                    encoding=str(cam.get("encoding", "jpeg")),
                )
                if cam
                else None
            ),
            version=version,
        )


@dataclass(frozen=True)
class Frame:
    """One timestep: where the arm was, what it was told, what it saw."""

    t: float
    state: tuple[float, ...]
    action: tuple[float, ...]
    image: str | None = None


@dataclass
class Episode:
    path: Path
    index: int
    task: str
    frames: list[Frame] = field(default_factory=list)

    @property
    def duration(self) -> float:
        return self.frames[-1].t - self.frames[0].t if len(self.frames) > 1 else 0.0

    def image_path(self, frame: Frame) -> Path | None:
        return self.path / frame.image if frame.image else None


def _episode_dir(root: Path, index: int) -> Path:
    return root / f"episode_{index:03d}"


def next_episode_index(root: Path) -> int:
    """One past the highest episode already in the directory.

    Indices are never reused: a discarded episode leaves a gap rather than
    letting a later recording quietly take over a number that appears in
    someone's notes.
    """
    if not root.exists():
        return 0
    used = [
        int(p.name.removeprefix("episode_"))
        for p in root.iterdir()
        if p.is_dir() and p.name.startswith("episode_") and p.name[8:].isdigit()
    ]
    return max(used) + 1 if used else 0


class EpisodeWriter:
    """Appends one episode's frames to disk as they are recorded."""

    def __init__(
        self, root: Path, spec: DatasetSpec, task: str, index: int | None = None
    ):
        if not task:
            raise ValueError("an episode needs a task description")
        self.root = Path(root)
        self.spec = spec
        self.task = task
        self.root.mkdir(parents=True, exist_ok=True)
        write_dataset_spec(self.root, spec)

        self.index = next_episode_index(self.root) if index is None else index
        self.path = _episode_dir(self.root, self.index)
        self.path.mkdir(parents=True, exist_ok=False)
        if spec.camera is not None:
            (self.path / "frames").mkdir()
        self._rows = (self.path / "frames.jsonl").open("w")
        self.count = 0
        self._first_t: float | None = None
        self._last_t = 0.0

    def add(
        self,
        stamp: float,
        state: Sequence[float],
        action: Sequence[float],
        image: bytes | None = None,
    ) -> None:
        """Record one timestep. ``stamp`` is absolute; ``t`` is made relative."""
        if self._first_t is None:
            self._first_t = stamp
        t = stamp - self._first_t
        row: dict = {
            "t": round(t, 6),
            "state": [float(v) for v in state],
            "action": [float(v) for v in action],
        }
        if image is not None:
            if self.spec.camera is None:
                raise ValueError("capture has no camera, but an image was recorded")
            name = f"frames/{self.count:06d}.{self.spec.camera.encoding}"
            (self.path / name).write_bytes(image)
            row["image"] = name
        self._rows.write(json.dumps(row) + "\n")
        # Flushed per row so a killed recorder loses at most the frame in hand.
        self._rows.flush()
        self.count += 1
        self._last_t = t

    def close(self) -> Path:
        """Write the episode's metadata and return its directory."""
        self._rows.close()
        (self.path / "episode.json").write_text(
            json.dumps(
                {
                    "index": self.index,
                    "task": self.task,
                    "frames": self.count,
                    "duration_s": round(self._last_t, 3),
                    "fps": self.spec.fps,
                },
                indent=2,
            )
            + "\n"
        )
        return self.path

    def discard(self) -> None:
        """Delete a recording that is not worth keeping."""
        self._rows.close()
        for child in sorted(self.path.rglob("*"), reverse=True):
            child.unlink() if child.is_file() else child.rmdir()
        self.path.rmdir()


def write_dataset_spec(root: Path, spec: DatasetSpec) -> None:
    (Path(root) / "dataset.json").write_text(
        json.dumps(spec.to_dict(), indent=2) + "\n"
    )


def load_dataset_spec(root: Path | str) -> DatasetSpec:
    path = Path(root) / "dataset.json"
    if not path.exists():
        raise FileNotFoundError(f"{root} is not a capture directory (no dataset.json)")
    return DatasetSpec.from_dict(json.loads(path.read_text()))


def load_episode(path: Path | str) -> Episode:
    """Read one episode directory, tolerating a truncated final row.

    A recorder killed mid-write leaves a partial last line; the frames before it
    are perfectly good data, and refusing to load them would throw away a whole
    session over the last 50 ms of it.
    """
    path = Path(path)
    meta_path = path / "episode.json"
    meta = json.loads(meta_path.read_text()) if meta_path.exists() else {}

    frames: list[Frame] = []
    for line in (path / "frames.jsonl").read_text().splitlines():
        if not line.strip():
            continue
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            break
        frames.append(
            Frame(
                t=float(row["t"]),
                state=tuple(float(v) for v in row["state"]),
                action=tuple(float(v) for v in row["action"]),
                image=row.get("image"),
            )
        )

    index = int(meta.get("index", int(path.name.removeprefix("episode_") or 0)))
    return Episode(
        path=path,
        index=index,
        task=str(meta.get("task", "")),
        frames=frames,
    )


def list_episodes(root: Path | str) -> list[Path]:
    root = Path(root)
    if not root.exists():
        return []
    return sorted(
        p for p in root.iterdir() if p.is_dir() and p.name.startswith("episode_")
    )


def resample(frames: Sequence[Frame], fps: int) -> list[Frame]:
    """Put frames on the exact 1/fps grid a LeRobot dataset assumes.

    LeRobot derives each frame's timestamp from its index and the dataset's fps —
    the recorded times are never stored, only implied. A capture whose timer
    slipped would therefore be exported as if it had not, silently stretching or
    compressing the motion. Resampling first makes the implied timeline the true
    one: each grid point takes the most recent frame at or before it, which is
    the causal choice (never showing an observation or action before it existed).
    """
    if fps <= 0:
        raise ValueError("fps must be positive")
    if not frames:
        return []

    period = 1.0 / fps
    duration = frames[-1].t - frames[0].t
    count = int(round(duration / period)) + 1
    base = frames[0].t

    out: list[Frame] = []
    cursor = 0
    for k in range(count):
        target = base + k * period
        while cursor + 1 < len(frames) and frames[cursor + 1].t <= target + 1e-9:
            cursor += 1
        src = frames[cursor]
        out.append(
            Frame(
                t=round(k * period, 6),
                state=src.state,
                action=src.action,
                image=src.image,
            )
        )
    return out
