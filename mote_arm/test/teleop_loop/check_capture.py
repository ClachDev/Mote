#!/usr/bin/env python3
"""Assert that a capture holds a real teleoperated motion, not just rows.

A recorder that ran but recorded nothing useful — a frozen arm, missing images,
actions that never differ from the state — still produces a well-formed capture.
These are the checks that tell the difference, so ``run_teleop_loop.sh`` fails
loudly instead of passing on an empty dataset.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from mote_arm.episode import list_episodes, load_dataset_spec, load_episode, resample


def main() -> int:
    capture = Path(sys.argv[1])
    # A recorder that was quit before its first episode leaves no dataset.json
    # at all, so this runs before anything reads one: "you recorded nothing" is
    # a verdict, and a traceback under it says only that the check crashed.
    if not (capture / "dataset.json").exists():
        print(
            f"nothing was recorded: {capture} holds no episodes.\n"
            "Re-run and press ENTER at the record prompt to capture one, "
            "driving the arm in the teleop terminal while it runs.",
            file=sys.stderr,
        )
        return 1
    spec = load_dataset_spec(capture)
    episodes = list_episodes(capture)
    if not episodes:
        print("no episodes recorded", file=sys.stderr)
        return 1

    problems = []
    for path in episodes:
        episode = load_episode(path)
        frames = episode.frames
        name = path.name
        if len(frames) < 20:
            problems.append(f"{name}: only {len(frames)} frames")
            continue

        span = [
            max(f.state[i] for f in frames) - min(f.state[i] for f in frames)
            for i in range(len(spec.joints))
        ]
        if max(span) < 0.02:
            problems.append(
                f"{name}: the arm never moved (widest joint span {max(span):.4f} rad)"
            )

        # The action is what reached arm_controller. If it never leads the
        # state, nothing was commanded and the episode records a coincidence.
        lead = max(
            abs(f.action[i] - f.state[i])
            for f in frames
            for i in range(len(spec.joints))
        )
        if lead < 1e-6:
            problems.append(
                f"{name}: action never differs from state — nothing was commanded"
            )

        if spec.camera is not None:
            # One reading of "this frame has an image", used by both checks: the
            # second used to re-test only that a filename was recorded, so a
            # frame naming a file that is not on disk was reported as missing
            # and then stat()ed anyway, raising through the whole report.
            present = [f for f in frames if f.image and (path / f.image).exists()]
            if len(present) < len(frames):
                problems.append(
                    f"{name}: {len(frames) - len(present)}/{len(frames)} "
                    "frames have no image"
                )
            sizes = {(path / f.image).stat().st_size for f in present}
            if present and len(sizes) < 2:
                problems.append(f"{name}: every camera frame is byte-identical")

        gridded = resample(frames, spec.fps)
        drift = abs(len(gridded) - len(frames))
        if drift > max(2, 0.05 * len(frames)):
            problems.append(
                f"{name}: recorded {len(frames)} frames but the timeline implies "
                f"{len(gridded)} at {spec.fps} fps — the recorder is not keeping its rate"
            )

        print(
            f"  {name}: {len(frames)} frames, {episode.duration:.1f}s, "
            f"widest joint span {max(span):.3f} rad, task {episode.task!r}"
        )

    for problem in problems:
        print(f"  PROBLEM {problem}", file=sys.stderr)
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
