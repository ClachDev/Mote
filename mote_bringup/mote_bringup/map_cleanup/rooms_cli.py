"""CLI: propose named zones for the rooms of a saved map.

    pixi run segment-map [MAP.yaml] [--write] [--out DIR]

With no argument it segments the active site floor's current map (see
:mod:`mote_bringup.sites`); pass a ``map.yaml`` to segment any saved revision.
It always writes a ``<map>_rooms.yaml`` proposal plus a ``<map>_rooms.png``
overlay to look at, and with ``--write`` merges the proposal into the floor's
``zones.yaml``, where the generated ``room_NN`` names are meant to be renamed to
what the rooms are actually called.

Merging never overwrites: a candidate covering the pose of a zone that already
has a footprint is dropped as already-named, and an existing zone is never
touched. That makes re-running after teaching a few rooms by hand additive, and
running twice in a row a no-op.
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

from .room_segmentation import (
    MapGeometry,
    Room,
    RoomParams,
    SegmentationResult,
    polygon_contains,
    segment_rooms,
)

# Distinct fills for adjacent rooms; cycled, so neighbours rarely collide.
PALETTE = [
    (232, 128, 64),
    (80, 175, 76),
    (60, 100, 220),
    (200, 140, 200),
    (40, 190, 220),
    (150, 90, 40),
    (90, 200, 160),
    (190, 110, 180),
]


def load_map(map_yaml: Path) -> tuple[np.ndarray, MapGeometry]:
    """Read a nav2 map pair into an occupancy array plus its geometry."""
    spec = yaml.safe_load(map_yaml.read_text())
    image = Path(spec["image"])
    if not image.is_absolute():
        image = map_yaml.parent / image
    occ = cv2.imread(str(image), cv2.IMREAD_GRAYSCALE)
    if occ is None:
        raise SystemExit(f"could not read map image {image}")
    if int(spec.get("negate", 0)):
        occ = 255 - occ
    return occ, MapGeometry(spec["resolution"], spec["origin"][:2], occ.shape[0])


def zone_entry(room: Room) -> dict:
    """A room as a ``mote_tasks.zones`` entry: a pose plus a polygon footprint.

    ``kind: room`` is stated rather than left to default to ``area`` because
    this tool knows it: what it segments *are* rooms, carved out of free space
    by where the doorways are. That is the one piece of a candidate's
    vocabulary it can honestly fill in — the name is a placeholder for the
    operator to replace, and only they know the aliases.
    """
    return {
        "x": round(room.pose[0], 3),
        "y": round(room.pose[1], 3),
        "yaw": 0.0,
        "kind": "room",
        "polygon": [[round(x, 3), round(y, 3)] for x, y in room.polygon],
    }


def merge_into_zones(path: Path, rooms: list[Room]) -> tuple[list[str], list[str]]:
    """Add the rooms that are not already named to a zones file.

    Returns ``(added, skipped)`` names. A candidate is skipped when its outline
    contains the pose of an existing zone that already carries a footprint --
    that room has a name, and it is not this tool's to replace. Bare waypoints
    (a ``pickup`` standing in the middle of a hall) do not suppress anything:
    they name a spot, not the room around it.
    """
    data = yaml.safe_load(path.read_text()) if path.exists() else None
    data = data or {"frame_id": "map"}
    zones = data.setdefault("zones", {}) or {}
    data["zones"] = zones

    named = [
        (float(spec["x"]), float(spec["y"]))
        for spec in zones.values()
        if ("radius" in spec or "polygon" in spec) and "x" in spec and "y" in spec
    ]
    added, skipped = [], []
    for room in rooms:
        if any(polygon_contains(room.polygon, x, y) for x, y in named):
            skipped.append(room.name)
            continue
        name = room.name
        suffix = 2
        while name in zones:
            name = f"{room.name}_{suffix}"
            suffix += 1
        zones[name] = zone_entry(room)
        added.append(name)

    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.{os.getpid()}")
    tmp.write_text(yaml.safe_dump(data, sort_keys=False, default_flow_style=None))
    os.replace(tmp, path)
    return added, skipped


def make_overlay(
    occ: np.ndarray, geometry: MapGeometry, result: SegmentationResult
) -> np.ndarray:
    """The map with every candidate filled, outlined, posed and labelled."""
    scale = max(1, int(round(1400 / max(occ.shape))))
    base = cv2.cvtColor(occ, cv2.COLOR_GRAY2BGR)
    base = cv2.resize(
        base,
        (occ.shape[1] * scale, occ.shape[0] * scale),
        interpolation=cv2.INTER_NEAREST,
    )
    fills = base.copy()

    def pixels(points):
        return np.array(
            [np.array(geometry.to_pixel(x, y)) * scale for x, y in points], np.int32
        )

    for index, room in enumerate(result.rooms):
        colour = PALETTE[index % len(PALETTE)]
        outline = pixels(room.polygon)
        cv2.fillPoly(fills, [outline], colour)
        cv2.polylines(base, [outline], True, colour, max(1, scale // 2), cv2.LINE_AA)

    out = cv2.addWeighted(fills, 0.35, base, 0.65, 0.0)
    for index, room in enumerate(result.rooms):
        colour = PALETTE[index % len(PALETTE)]
        cx, cy = (np.array(geometry.to_pixel(*room.pose)) * scale).astype(int)
        cv2.circle(out, (cx, cy), max(2, scale), (20, 20, 20), -1)
        cv2.circle(out, (cx, cy), max(1, scale - 1), colour, -1)
        label = room.name.rsplit("_", 1)[-1]
        cv2.putText(
            out,
            label,
            (cx + 2 * scale, cy - scale),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.35 * scale,
            (0, 0, 0),
            max(1, scale),
            cv2.LINE_AA,
        )
    return out


def _resolve_map(argument: str | None) -> tuple[Path, Path]:
    """The map to segment and where its proposal and overlay should land.

    A map revision is immutable once published, so the active floor's outputs go
    beside its zones.yaml rather than inside ``maps/<rev>/``. An explicitly named
    map is the caller's own business and gets its outputs beside it.
    """
    if argument:
        explicit = Path(argument).expanduser()
        return explicit, explicit.parent
    from mote_bringup import sites

    resolved = sites.resolve_map()
    if not resolved:
        raise SystemExit(
            "no map for the active site floor (run: pixi run site info), "
            "or pass a map.yaml explicitly"
        )
    return Path(resolved), Path(sites.floor_dir(*sites.active()))


def _resolve_zones(argument: str | None) -> Path:
    if argument:
        return Path(argument).expanduser()
    from mote_bringup import sites

    return sites.zones_for_write()


def main(argv: list[str] | None = None) -> int:
    defaults = RoomParams()
    parser = argparse.ArgumentParser(prog="segment-map", description=__doc__)
    parser.add_argument("map", nargs="?", help="map.yaml (default: active floor's map)")
    parser.add_argument("--out", help="output directory (default: beside the map)")
    parser.add_argument(
        "--write",
        action="store_true",
        help="merge the candidates into the floor's zones.yaml",
    )
    parser.add_argument("--zones", help="merge into this zones file instead")
    parser.add_argument("--prefix", default="room", help="generated name prefix")
    parser.add_argument("--door", type=float, default=defaults.door_max_m)
    parser.add_argument("--min-area", type=float, default=defaults.min_room_area_m2)
    parser.add_argument("--wall-run", type=float, default=defaults.min_wall_run_m)
    parser.add_argument("--no-align", action="store_true", help="skip wall alignment")
    args = parser.parse_args(argv)

    map_yaml, beside = _resolve_map(args.map)
    occ, geometry = load_map(map_yaml)
    result = segment_rooms(
        occ,
        geometry,
        RoomParams(
            door_max_m=args.door,
            min_room_area_m2=args.min_area,
            min_wall_run_m=args.wall_run,
            align=not args.no_align,
        ),
        name_prefix=args.prefix,
    )

    out_dir = Path(args.out) if args.out else beside
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = map_yaml.stem
    overlay_path = out_dir / f"{stem}_rooms.png"
    proposal_path = out_dir / f"{stem}_rooms.yaml"
    cv2.imwrite(str(overlay_path), make_overlay(occ, geometry, result))
    proposal_path.write_text(
        yaml.safe_dump(
            {
                "frame_id": "map",
                "zones": {room.name: zone_entry(room) for room in result.rooms},
            },
            sort_keys=False,
            default_flow_style=None,
        )
    )

    print(
        f"{len(result.rooms)} rooms from {result.faces} faces "
        f"({len(result.cuts_x)}x{len(result.cuts_y)} cut lines, "
        f"rotation {result.rotation_deg:g} deg)"
    )
    for room in result.rooms:
        print(
            f"  {room.name:12s} {room.area_m2:7.1f} m^2  "
            f"pose ({room.pose[0]:7.2f}, {room.pose[1]:7.2f})  "
            f"clearance {room.clearance_m:.2f} m  {len(room.polygon)} vertices"
        )
    print(f"wrote {overlay_path}")
    print(f"wrote {proposal_path}")

    if args.write or args.zones:
        zones_path = _resolve_zones(args.zones)
        added, skipped = merge_into_zones(zones_path, result.rooms)
        print(
            f"merged into {zones_path}: {len(added)} added, {len(skipped)} already named"
        )
        if added:
            print("  added:   " + ", ".join(added))
        if skipped:
            print("  skipped: " + ", ".join(skipped))
        print("rename the generated zones to what the rooms are called")
    return 0


if __name__ == "__main__":
    sys.exit(main())
