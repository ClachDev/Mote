"""The capture format: what the robot writes, and what replay and export read."""

import json

import pytest

from mote_arm.episode import (
    CameraSpec,
    DatasetSpec,
    EpisodeWriter,
    Frame,
    episodes_root,
    list_episodes,
    load_dataset_spec,
    load_episode,
    next_episode_index,
    resample,
)

JOINTS = ("shoulder_pan", "elbow_flex")


def spec(camera: bool = True) -> DatasetSpec:
    return DatasetSpec(
        name="teleop",
        fps=20,
        joints=JOINTS,
        camera=CameraSpec(key="front", topic="/image_raw/compressed")
        if camera
        else None,
    )


def test_capture_round_trips(tmp_path):
    writer = EpisodeWriter(tmp_path, spec(), task="pick up the block")
    writer.add(100.0, [0.1, 0.2], [0.15, 0.25], image=b"\x89PNG-not-really")
    writer.add(100.05, [0.11, 0.21], [0.15, 0.25], image=b"second")
    path = writer.close()

    assert load_dataset_spec(tmp_path) == spec()
    episode = load_episode(path)
    assert episode.task == "pick up the block"
    assert episode.index == 0
    assert [f.t for f in episode.frames] == pytest.approx([0.0, 0.05])
    assert episode.frames[0].state == pytest.approx((0.1, 0.2))
    assert episode.frames[1].action == pytest.approx((0.15, 0.25))
    assert episode.image_path(episode.frames[1]).read_bytes() == b"second"


def test_timestamps_are_relative_to_the_first_frame(tmp_path):
    # The recorder samples on a monotonic clock, which starts wherever the
    # machine booted; an episode's timeline has to start at zero.
    writer = EpisodeWriter(tmp_path, spec(camera=False), task="t")
    writer.add(98765.5, [0.0, 0.0], [0.0, 0.0])
    writer.add(98766.0, [0.0, 0.0], [0.0, 0.0])
    episode = load_episode(writer.close())
    assert [f.t for f in episode.frames] == pytest.approx([0.0, 0.5])
    assert episode.duration == pytest.approx(0.5)


def test_episodes_accumulate_and_indices_are_never_reused(tmp_path):
    for _ in range(3):
        writer = EpisodeWriter(tmp_path, spec(camera=False), task="t")
        writer.add(0.0, [0.0, 0.0], [0.0, 0.0])
        writer.close()
    assert [p.name for p in list_episodes(tmp_path)] == [
        "episode_000",
        "episode_001",
        "episode_002",
    ]

    discarded = EpisodeWriter(tmp_path, spec(camera=False), task="t")
    discarded.add(0.0, [0.0, 0.0], [0.0, 0.0])
    discarded.discard()
    assert not discarded.path.exists()
    # The gap stays a gap: a discarded episode 3 must not be re-issued to a
    # later recording, or a number in someone's notes would name two takes.
    assert next_episode_index(tmp_path) == 3


def test_a_killed_recorder_leaves_readable_frames(tmp_path):
    writer = EpisodeWriter(tmp_path, spec(camera=False), task="t")
    writer.add(0.0, [0.1, 0.1], [0.1, 0.1])
    writer.add(0.05, [0.2, 0.2], [0.2, 0.2])
    # Rows are flushed as they are written, so a kill mid-write truncates the
    # last line and nothing else. No episode.json is ever written.
    with (writer.path / "frames.jsonl").open("a") as handle:
        handle.write('{"t": 0.1, "state": [0.3, 0.')

    episode = load_episode(writer.path)
    assert len(episode.frames) == 2
    assert episode.task == ""


def test_an_image_needs_a_camera_in_the_spec(tmp_path):
    writer = EpisodeWriter(tmp_path, spec(camera=False), task="t")
    with pytest.raises(ValueError, match="no camera"):
        writer.add(0.0, [0.0, 0.0], [0.0, 0.0], image=b"x")


def test_an_episode_needs_a_task(tmp_path):
    with pytest.raises(ValueError, match="task"):
        EpisodeWriter(tmp_path, spec(), task="")


def test_a_capture_from_another_format_version_is_refused(tmp_path):
    (tmp_path / "dataset.json").write_text(
        json.dumps({**spec().to_dict(), "version": 99})
    )
    with pytest.raises(ValueError, match="version 99"):
        load_dataset_spec(tmp_path)


def test_episodes_root_follows_mote_home(tmp_path, monkeypatch):
    monkeypatch.setenv("MOTE_HOME", str(tmp_path))
    assert episodes_root() == tmp_path / "episodes"


def frames(*times) -> list[Frame]:
    return [Frame(t=t, state=(t,), action=(t,)) for t in times]


def test_resample_is_a_no_op_on_an_exact_grid():
    original = frames(0.0, 0.1, 0.2)
    assert resample(original, fps=10) == original


def test_resample_holds_the_most_recent_sample():
    # LeRobot stores no timestamps — it derives them from the index and fps — so
    # a slipped capture would otherwise export as if its timing had been perfect.
    out = resample(frames(0.0, 0.07, 0.23), fps=10)
    assert [f.t for f in out] == pytest.approx([0.0, 0.1, 0.2])
    # Zero-order hold, never interpolation and never a peek ahead: 0.1 s takes
    # the 0.07 s sample, and 0.2 s still does, because 0.23 has not happened yet.
    assert [f.state[0] for f in out] == pytest.approx([0.0, 0.07, 0.07])


def test_resample_of_an_empty_or_single_frame_episode():
    assert resample([], fps=20) == []
    assert len(resample(frames(0.0), fps=20)) == 1


def test_resample_rejects_a_nonsense_rate():
    with pytest.raises(ValueError):
        resample(frames(0.0, 0.1), fps=0)
