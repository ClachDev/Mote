#!/usr/bin/env python3
"""Score map room segmentation against a sim world's ground-truth rooms.

    pixi run segment-eval [world_stem ...] [--out DIR] [-- --door 1.4 ...]

For each world on the ladder it segments that world's committed SLAM map
(``mote_simulation/sim_home/sites/<world>/floors/ground/map``) and compares the
candidates against ``mote_simulation/worlds/<world>.rooms.yaml`` -- the walkable
rectangle of every enclosed room, which for hospital_world its generator emits
and for the two hand-written worlds was read off the SDF.

**Only rooms the robot actually mapped are scored.** The sim maps come from a
timed autonomous exploration run, so parts of the hospital were never visited; a
room whose rectangle is mostly unobserved is not a segmentation failure and is
reported separately rather than counted against recall.

Scoring, per mapped ground-truth room, over observed free pixels:

    recovered      one candidate covers >= COVER of it, and that candidate does
                   not also cover >= MERGE of another mapped room
    split          no candidate covers >= COVER of it (it came out in pieces)
    merged         its best candidate spans it and another mapped room

Candidates that match no ground-truth room are corridor and other non-room free
space (the ladder's worlds have corridors, which are not rooms), and are counted
but not penalised -- a corridor network coming out as one candidate is the
right answer, and the operator deletes or renames it.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "mote_bringup"))

from mote_bringup.map_cleanup.room_segmentation import (  # noqa: E402
    MapGeometry,
    RoomParams,
    _rotate,
    polygon_contains,
    segment_rooms,
)
from mote_bringup.map_cleanup.rooms_cli import load_map, make_overlay  # noqa: E402
from mote_bringup.map_cleanup.structure_extraction import FREE  # noqa: E402

WORLDS = ["mote_world", "office_world", "hospital_world"]
MAPPED = 0.5  # a truth room needs this much of it observed free to be scored
COVER = 0.6  # a candidate covering this much of a truth room recovers it
MERGE = 0.35  # ... and covering this much of a second one has merged them


def truth_masks(rooms, geometry, shape) -> list[np.ndarray]:
    masks = []
    for x0, y0, x1, y1 in rooms:
        c0, r0 = geometry.to_pixel(x0, y1)  # +y is up in the map frame, down in rows
        c1, r1 = geometry.to_pixel(x1, y0)
        mask = np.zeros(shape, bool)
        mask[
            max(int(round(r0)), 0) : int(round(r1)),
            max(int(round(c0)), 0) : int(round(c1)),
        ] = True
        masks.append(mask)
    return masks


def polygon_masks(polygons, geometry, shape) -> list[np.ndarray]:
    masks = []
    for corners in polygons:
        mask = np.zeros(shape, np.uint8)
        outline = np.array(
            [np.round(geometry.to_pixel(x, y)) for x, y in corners], np.int32
        )
        cv2.fillPoly(mask, [outline], 1)
        masks.append(mask.astype(bool))
    return masks


def turn(occ, geometry, rooms, degrees: float):
    """Re-pose a map and its ground truth as if SLAM had started facing elsewhere.

    A map frame's axes are wherever the robot happened to start, so a real map
    is rarely axis-aligned; the sim ladder's maps all are. Rotating both the
    grid and the truth rectangles by the same angle exercises the alignment step
    against real SLAM data instead of a synthetic fixture.
    """
    turned, matrix = _rotate(occ, degrees)
    moved = MapGeometry(geometry.resolution, geometry.origin, turned.shape[0])

    def carry(x, y):
        px = np.array([[geometry.to_pixel(x, y)]], np.float64)
        qx, qy = cv2.transform(px, matrix).reshape(2)
        return moved.to_world(qx, qy)

    corners = [
        [carry(x, y) for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))]
        for x0, y0, x1, y1 in rooms
    ]
    return turned, moved, corners


def evaluate(
    map_yaml: Path,
    rooms_yaml: Path,
    params: RoomParams,
    out_dir: Path | None,
    rotate: float = 0.0,
):
    occ, geometry = load_map(map_yaml)
    truth = yaml.safe_load(rooms_yaml.read_text())["rooms"]
    if rotate:
        occ, geometry, polygons = turn(occ, geometry, truth, rotate)
        masks = polygon_masks(polygons, geometry, occ.shape)
    else:
        masks = truth_masks(truth, geometry, occ.shape)
    result = segment_rooms(occ, geometry, params)
    free = occ >= (FREE - 20)

    candidates = []
    for room in result.rooms:
        assert polygon_contains(room.polygon, *room.pose), (
            f"{room.name}'s pose is outside its own footprint -- "
            "'go to it' and 'am I in it' would disagree"
        )
        mask = np.zeros(occ.shape, np.uint8)
        outline = np.array(
            [np.round(geometry.to_pixel(x, y)) for x, y in room.polygon], np.int32
        )
        cv2.fillPoly(mask, [outline], 1)
        candidates.append(mask.astype(bool) & free)

    observed = [int((m & free).sum()) for m in masks]
    full = [int(m.sum()) for m in masks]
    mapped = [
        i for i in range(len(truth)) if full[i] and observed[i] / full[i] >= MAPPED
    ]

    # overlap[i][j] = fraction of mapped truth room i's observed free space that
    # candidate j covers.
    overlap = {
        i: [
            (candidates[j] & masks[i] & free).sum() / observed[i]
            for j in range(len(candidates))
        ]
        for i in mapped
    }

    recovered, split, merged, assignment = [], [], [], {}
    for i in mapped:
        if not overlap[i]:
            split.append(i)
            continue
        best = int(np.argmax(overlap[i]))
        if overlap[i][best] < COVER:
            split.append(i)
            continue
        others = [k for k in mapped if k != i and overlap[k][best] >= MERGE]
        (merged if others else recovered).append(i)
        assignment[i] = best

    matched = set(assignment.values())
    other = [j for j in range(len(candidates)) if j not in matched]

    if out_dir:
        out_dir.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(
            str(out_dir / f"{map_yaml.parent.parent.parent.parent.name}_rooms.png"),
            make_overlay(occ, geometry, result),
        )

    detail = [
        (
            "split" if i in split else "merged",
            truth[i],
            round(observed[i] / full[i], 2),
            round(max(overlap[i], default=0.0), 2),
        )
        for i in mapped
        if i in split or i in merged
    ]
    return {
        "detail": detail,
        "truth": len(truth),
        "mapped": len(mapped),
        "candidates": len(result.rooms),
        "recovered": len(recovered),
        "split": len(split),
        "merged": len(merged),
        "other": len(other),
        "rotation_deg": result.rotation_deg,
        "faces": result.faces,
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("worlds", nargs="*", default=None)
    parser.add_argument("--out", help="write an overlay per world here")
    parser.add_argument("-v", "--verbose", action="store_true", help="list failures")
    parser.add_argument(
        "--rotate",
        type=float,
        default=0.0,
        help="turn map and truth by this many degrees first (tests wall alignment)",
    )
    parser.add_argument("--door", type=float, default=RoomParams.door_max_m)
    parser.add_argument("--min-area", type=float, default=RoomParams.min_room_area_m2)
    parser.add_argument("--wall-run", type=float, default=RoomParams.min_wall_run_m)
    parser.add_argument(
        "--sim-home", default=str(REPO / "mote_simulation" / "sim_home")
    )
    args = parser.parse_args(argv)

    params = RoomParams(
        door_max_m=args.door,
        min_room_area_m2=args.min_area,
        min_wall_run_m=args.wall_run,
    )
    out_dir = Path(args.out) if args.out else None
    header = (
        f"{'world':16s} {'truth':>6s} {'mapped':>7s} {'cand':>5s} "
        f"{'recovered':>10s} {'split':>6s} {'merged':>7s} {'other':>6s}"
    )
    print(
        f"door<={params.door_max_m} m  wall-run>={params.min_wall_run_m} m  "
        f"min-area>={params.min_room_area_m2} m^2"
        + (f"  map turned {args.rotate:g} deg" if args.rotate else "")
    )
    print(header)
    failures = 0
    for world in args.worlds or WORLDS:
        map_yaml = (
            Path(args.sim_home)
            / "sites"
            / world
            / "floors"
            / "ground"
            / "map"
            / "map.yaml"
        )
        rooms_yaml = REPO / "mote_simulation" / "worlds" / f"{world}.rooms.yaml"
        if not map_yaml.exists() or not rooms_yaml.exists():
            print(f"{world:16s} skipped (no map or no rooms ground truth)")
            continue
        s = evaluate(map_yaml, rooms_yaml, params, out_dir, args.rotate)
        print(
            f"{world:16s} {s['truth']:6d} {s['mapped']:7d} {s['candidates']:5d} "
            f"{s['recovered']:10d} {s['split']:6d} {s['merged']:7d} {s['other']:6d}"
        )
        for kind, rect, seen, cover in s["detail"] if args.verbose else []:
            box = ", ".join(f"{v:g}" for v in rect)
            print(f"    {kind:7s} [{box}]  observed {seen:.0%}  best cover {cover:.0%}")
        failures += s["split"] + s["merged"]
    print(f"total split+merged: {failures}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
