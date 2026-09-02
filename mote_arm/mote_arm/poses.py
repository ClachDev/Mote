"""Named arm poses — teach a safe pose, then return to it.

The base layer captures a map position by driving there and running
``pixi run save-zone`` (see ``mote_tasks``/Sites); this is the arm's analogue:
pose the limp arm by hand, capture it, and later command that exact pose back.

Poses live in ``~/.mote/arm_poses.yaml`` (``MOTE_HOME`` overrides ``~/.mote``,
as elsewhere) — per-robot data, outside the repo, because a pose is only
meaningful for one physical arm and its calibration.

ROS-free so the file handling is unit-testable without hardware.
"""

from __future__ import annotations

import math
import os
from pathlib import Path

import yaml


def mote_home() -> Path:
    return Path(os.environ.get("MOTE_HOME", "~/.mote")).expanduser()


def poses_path() -> Path:
    return mote_home() / "arm_poses.yaml"


def load_poses(path: Path | str | None = None) -> dict[str, dict[str, float]]:
    """Return {pose_name: {joint_name: radians}}; empty if none taught yet."""
    p = Path(path) if path is not None else poses_path()
    if not p.exists():
        return {}
    data = yaml.safe_load(p.read_text()) or {}
    poses = data.get("poses") or {}
    return {
        str(name): {str(j): float(v) for j, v in (joints or {}).items()}
        for name, joints in poses.items()
    }


def save_pose(
    name: str,
    joints: dict[str, float],
    path: Path | str | None = None,
) -> Path:
    """Add or replace one named pose, leaving the others untouched."""
    if not name:
        raise ValueError("pose name must not be empty")
    if not joints:
        raise ValueError(f"pose {name!r} has no joint positions")

    p = Path(path) if path is not None else poses_path()
    poses = load_poses(p)
    poses[name] = {str(j): float(v) for j, v in joints.items()}
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(yaml.safe_dump({"poses": poses}, sort_keys=True))
    return p


def shift_poses(
    taught: dict[str, dict[str, float]],
    shifts: dict[str, float],
) -> dict[str, dict[str, float]]:
    """Re-express taught poses about a moved zero, preserving where they point.

    A pose is stored as radians from the joint's zero, so moving the zero
    silently changes which physical position each number names. The correction
    is exact and known — it is the same shift the calibration computed — so the
    poses can simply be rewritten rather than re-taught by hand, which would
    mean physically posing the arm again for no reason.

    Joints absent from ``shifts`` keep their stored value.
    """
    return {
        name: {joint: value + shifts.get(joint, 0.0) for joint, value in joints.items()}
        for name, joints in taught.items()
    }


def save_poses(
    taught: dict[str, dict[str, float]],
    path: Path | str | None = None,
) -> Path:
    """Replace the whole pose file, keeping a .bak of what was there."""
    p = Path(path) if path is not None else poses_path()
    p.parent.mkdir(parents=True, exist_ok=True)
    if p.exists():
        p.with_suffix(p.suffix + ".bak").write_text(p.read_text())
    p.write_text(yaml.safe_dump({"poses": taught}, sort_keys=True))
    return p


def envelope(
    taught: dict[str, dict[str, float]],
    margin: float = 0.0,
) -> dict[str, tuple[float, float]]:
    """Per-joint (min, max) spanning every taught pose, widened by ``margin``.

    Limits derived this way are safe by construction: every position inside the
    band lies between poses a human physically vetted. Joints that appear in no
    pose are absent from the result — the caller keeps their existing limits
    rather than inventing a band from nothing.
    """
    if margin < 0:
        raise ValueError("margin must not be negative")
    spans: dict[str, tuple[float, float]] = {}
    for joints in taught.values():
        for name, value in joints.items():
            lo, hi = spans.get(name, (value, value))
            spans[name] = (min(lo, value), max(hi, value))
    return {n: (lo - margin, hi + margin) for n, (lo, hi) in spans.items()}


def interpolate(
    start: dict[str, float],
    target: dict[str, float],
    step: float,
) -> list[dict[str, float]]:
    """Waypoints from ``start`` to ``target``, no joint moving > ``step`` per hop.

    Commanding a large move as one goal hands the whole trajectory to the servo
    and leaves nothing to supervise. Walking it in bounded increments keeps the
    caller in the loop, so a stall can be caught partway instead of being
    discovered at the end (or held indefinitely against a load).

    Only joints present in both dicts move. The final waypoint is exactly
    ``target``, so interpolation never changes the destination.
    """
    if step <= 0:
        raise ValueError("step must be positive")
    shared = [n for n in target if n in start]
    if not shared:
        return []
    largest = max(abs(target[n] - start[n]) for n in shared)
    hops = max(1, math.ceil(largest / step))
    return [
        {n: start[n] + (target[n] - start[n]) * (i / hops) for n in shared}
        for i in range(1, hops + 1)
    ]


def delete_pose(name: str, path: Path | str | None = None) -> bool:
    """Remove a pose. Returns False if it was not there."""
    p = Path(path) if path is not None else poses_path()
    poses = load_poses(p)
    if name not in poses:
        return False
    del poses[name]
    p.write_text(yaml.safe_dump({"poses": poses}, sort_keys=True))
    return True
