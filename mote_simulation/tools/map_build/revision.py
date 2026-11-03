"""A solved replay leg, turned into a site-bundle map revision.

``save-map`` writes a revision from a *running* mapping session: it asks
``map_saver_cli`` for the map pair and slam_toolbox for the posegraph, promotes
the decluttered image over the raw one, and stamps a ``meta.yaml``. An offline
build has the same two artifacts — the finished grid and the serialized graph —
and has to write the same layout, because the registry, ``bundle.validate`` and
every consumer downstream know exactly one shape of revision.

So this module writes the map pair and the meta, and the *cleaning* is
``sites.promote_cleaned`` itself rather than a copy of it: a build whose
declutter pass differed from the robot's would produce maps that cannot be
compared with the ones already published.

Nothing here imports ROS. The grid arrives as the ``map.npz`` the replay
harness wrote, so a revision can be assembled — and this module tested — on a
machine with no ROS on it at all.
"""

from __future__ import annotations

import hashlib
import time
from pathlib import Path

import numpy as np
import yaml

# Two scales, and confusing them writes a map whose unknown space reads back as
# free — which is to say, as somewhere Nav2 may plan straight through.
#
# GRID_* classify the incoming ``OccupancyGrid``: 0..100 probability, -1
# unknown. These are the values ``map_saver``, ``render.py`` and the benchmark's
# ``map_quality`` all split occupancy at, so a built map's pixels agree with
# every other reading of the same grid.
GRID_FREE_MAX = 25
GRID_OCC_MIN = 65

# YAML_* are what ``map.yaml`` declares, on ``map_server``'s ``p = (255 -
# shade) / 255`` scale: free below the first, occupied above the second,
# unknown in between. The three shades written below land at p = 0.004, 0.196
# and 1.0, so these sit with room on both sides.
#
# ``map_saver`` writes 0.196 for the free threshold, which is the unknown
# shade's own p value to five decimals — the read-back is then correct by
# 8e-5, and a grey pixel one shade lighter is free space. A build writes the
# same three shades with a threshold that is not deciding the map on a
# rounding, and picks the pair ``bundle.occupancy`` already counts at.
YAML_FREE_THRESH = 0.100
YAML_OCC_THRESH = 0.650

FREE_PX = 254
UNKNOWN_PX = 205
OCCUPIED_PX = 0


def grid_to_png_array(grid: np.ndarray) -> np.ndarray:
    """ROS occupancy grid -> the greyscale image ``map_saver`` would write.

    Occupied black, free white, unknown grey, north up: the grid's row 0 is the
    bottom of the world, an image's row 0 is the top.
    """
    image = np.full(grid.shape, UNKNOWN_PX, dtype=np.uint8)
    decided = grid >= 0
    image[decided & (grid <= GRID_FREE_MAX)] = FREE_PX
    image[grid >= GRID_OCC_MIN] = OCCUPIED_PX
    return np.flipud(image)


def png_to_grid(
    image: np.ndarray,
    negate: int = 0,
    free_thresh: float = YAML_FREE_THRESH,
    occupied_thresh: float = YAML_OCC_THRESH,
) -> np.ndarray:
    """The inverse, as ``map_server`` reads a saved map back.

    Wanted for the baseline side of the build's metric diff: the revision a
    candidate is compared against is on disk as pixels, and the metrics take
    grids. The thresholds come from that revision's own ``map.yaml``, because a
    map saved by a robot declares ``map_saver``'s and not these.
    """
    values = image.astype(np.float64) if negate else 255.0 - image.astype(np.float64)
    p = values / 255.0
    grid = np.full(image.shape, -1, dtype=np.int16)
    grid[p < free_thresh] = 0
    grid[p > occupied_thresh] = 100
    return np.flipud(grid)


def write_map_pair(npz_path, rev_dir) -> dict:
    """Write ``map.png`` + ``map.yaml`` from a replay leg's captured grid.

    Returns the frame — resolution, origin, size — for the build report.
    """
    import cv2

    rev_dir = Path(rev_dir)
    rev_dir.mkdir(parents=True, exist_ok=True)
    data = np.load(npz_path)
    grid = data["grid"]
    resolution = float(data["resolution"])
    ox, oy = (float(v) for v in data["origin"])
    # Harness output from before the origin yaw was recorded has none. It has
    # always been zero in practice, but a build must not assume it: a dropped
    # origin yaw moves every zone on the floor and leaves the map looking
    # perfectly good, so the absence is written into the report as well.
    yaw = float(data["origin_yaw"]) if "origin_yaw" in data.files else 0.0

    cv2.imwrite(str(rev_dir / "map.png"), grid_to_png_array(grid))
    (rev_dir / "map.yaml").write_text(
        "image: map.png\n"
        "mode: trinary\n"
        f"resolution: {resolution:.3f}\n"
        f"origin: [{ox:.3f}, {oy:.3f}, {yaw:.6f}]\n"
        "negate: 0\n"
        f"occupied_thresh: {YAML_OCC_THRESH}\n"
        f"free_thresh: {YAML_FREE_THRESH}\n"
    )
    return {
        "width": int(grid.shape[1]),
        "height": int(grid.shape[0]),
        "resolution": resolution,
        "origin": [ox, oy, yaw],
        "origin_yaw_recorded": "origin_yaw" in data.files,
    }


def copy_posegraph(set_dir, rev_dir) -> list[str]:
    """Put the leg's serialized graph in beside its map.

    A revision without it navigates and cannot be *extended* — the frame is
    lost, and with it every zone taught in it — so the build treats a missing
    graph as a failure rather than a warning, and this reports what it found.
    """
    set_dir, rev_dir = Path(set_dir), Path(rev_dir)
    copied = []
    for name in ("map.posegraph", "map.data"):
        source = set_dir / name
        if source.is_file():
            (rev_dir / name).write_bytes(source.read_bytes())
            copied.append(name)
    return copied


def new_revision_id() -> str:
    """The shape ``sites._new_revision_dir`` mints: revisions sort by name, and
    a build's has to sort beside a robot's."""
    return time.strftime("%Y%m%dT%H%M%S")


def digest_bag(bag_dir) -> dict:
    """A mapping bag's identity: every file, its size, and one digest over all.

    The bag is the build's source, so "which bag" has to mean the bytes and not
    a directory name anybody can re-use. Cheap enough to always do: a 184 MB
    bag hashes in under a second.
    """
    bag_dir = Path(bag_dir)
    files = sorted(p for p in bag_dir.iterdir() if p.is_file())
    whole = hashlib.sha256()
    members = []
    for path in files:
        each = hashlib.sha256()
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1 << 20), b""):
                each.update(chunk)
                whole.update(chunk)
        whole.update(path.name.encode())
        members.append(
            {
                "name": path.name,
                "bytes": path.stat().st_size,
                "sha256": each.hexdigest(),
            }
        )
    return {"name": bag_dir.name, "sha256": whole.hexdigest(), "files": members}


def digest_file(path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_meta(rev_dir, meta: dict) -> None:
    Path(rev_dir, "meta.yaml").write_text(yaml.safe_dump(meta, sort_keys=False))
