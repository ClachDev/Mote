"""Segment the free space of a 2D occupancy grid into rooms.

This is the ROSE2 layer that sits on top of :mod:`structure_extraction`
(arXiv:2203.03519): once the dominant wall orientations are known, the walls
they describe are extended into full-width cut lines, those lines partition the
map into a grid of faces, and faces are merged back together wherever the
boundary between them is wide open. What survives the merge is a room.

The whole method rests on one physical observation: **a doorway is narrow**. A
room connects to the corridor through a ~0.9 m gap, while two faces of the same
room (or two stretches of the same corridor) are separated by nothing at all, or
by an opening far wider than a door. So the merge rule is a width threshold, not
a heuristic about shape or size, and the segmentation is stable across rooms of
wildly different sizes -- which a global distance-transform threshold is not.

    1. detect the dominant wall orientation (the FFT scan already written for
       decluttering) and rotate the map so those walls are axis-aligned,
    2. project vertically- and horizontally-extended wall pixels onto the two
       axes; the peaks are the map's wall lines,
    3. cut the map along every wall line -- including where that line runs
       through open space, which is what separates a room from the corridor
       stretch outside its door,
    4. merge neighbouring faces whose shared boundary has a contiguous opening
       wider than a door,
    5. keep the merged faces with enough observed free space in them; each is a
       room, outlined by a polygon and posed at its clearance maximum.

Manhattan *after rotation*: one dominant orientation and its perpendicular are
handled (including a map whose frame is rotated arbitrarily, which is the usual
case -- a map frame's axes are wherever SLAM happened to start). A building with
two wings at 30 degrees to each other is not, and will over-cut the off-axis
wing.

Nothing here touches ROS or the filesystem: it takes an occupancy array and
returns coordinates, so it is testable off the robot.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np

from .structure_extraction import (
    FREE,
    OCCUPIED,
    UNKNOWN,
    Params,
    _angular_energy,
    _pick_directions,
)


@dataclass
class RoomParams:
    """Everything the segmentation is allowed to assume about a building."""

    door_max_m: float = 1.4  # an opening this wide or narrower separates rooms
    min_room_area_m2: float = 1.5  # discard candidates with less free space
    min_wall_run_m: float = 1.5  # a cut line needs an unbroken wall this long
    line_merge_m: float = 0.4  # cut lines closer than this are one line
    wall_thickness_m: float = 0.15  # outlines are inset by half of this
    simplify_m: float = 0.1  # polygon simplification tolerance
    min_free_frac: float = 0.08  # a face with less observed free space is dead
    align: bool = True  # rotate to the dominant wall orientation first
    align_min_deg: float = 0.75  # below this, rotating only costs resampling


@dataclass
class Room:
    """One segmented room, in map-frame metres."""

    name: str
    polygon: list[tuple[float, float]]  # outline, counter-clockwise
    pose: tuple[float, float]  # clearance maximum: the open middle
    area_m2: float  # observed free space inside the outline
    clearance_m: float  # distance from the pose to the nearest obstacle
    mask: np.ndarray = field(default=None, repr=False)  # pixels, source grid


@dataclass
class SegmentationResult:
    rooms: list[Room]
    rotation_deg: float  # rotation applied to axis-align the walls
    cuts_x: list[int]  # cut-line positions, rotated pixel frame
    cuts_y: list[int]
    faces: int  # live faces before merging
    encircling: int  # regions dropped for wrapping around other rooms
    labels: np.ndarray = field(default=None, repr=False)  # rotated frame


class MapGeometry:
    """The map.yaml half of an occupancy grid: pixels <-> map-frame metres.

    Matches the transform the fleet dashboard draws with (``server/ui/map.mjs``)
    so a polygon written here lands where the operator sees it.
    """

    def __init__(self, resolution: float, origin: tuple[float, float], height: int):
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))
        self.height = int(height)

    def to_world(self, col: float, row: float) -> tuple[float, float]:
        return (
            col * self.resolution + self.origin[0],
            (self.height - row) * self.resolution + self.origin[1],
        )

    def to_pixel(self, x: float, y: float) -> tuple[float, float]:
        return (
            (x - self.origin[0]) / self.resolution,
            self.height - (y - self.origin[1]) / self.resolution,
        )


def dominant_rotation_deg(occ: np.ndarray, params: Params | None = None) -> float:
    """The rotation that axis-aligns the map's dominant wall direction.

    Wall orientations are only defined modulo 90 degrees for this purpose: a
    rectilinear building's two wall families are perpendicular, so aligning one
    aligns the other. The answer is in (-45, 45].

    The wall image is padded out to a square first. The angular scan measures
    orientation in *array index* space, and a frequency-domain index maps to a
    real frequency divided by that axis' length -- so on an oblong map the two
    axes carry different scales and every angle is skewed towards the long one.
    (The declutter pass is immune: it places its wedges in the same index space
    it found them in. Here the number is a rotation applied to the map itself,
    so it has to be a real angle -- on a 58 x 38 m map the skew is 8 degrees.)
    """
    params = params or Params()
    wall = occ <= (OCCUPIED + 20)
    if not wall.any():
        return 0.0
    side = max(wall.shape)
    square = np.zeros((side, side), np.float32)
    top, left = (side - wall.shape[0]) // 2, (side - wall.shape[1]) // 2
    square[top : top + wall.shape[0], left : left + wall.shape[1]] = wall
    mag = np.abs(np.fft.fftshift(np.fft.fft2(square)))
    angles, energy = _angular_energy(mag, params)
    directions = _pick_directions(angles, energy, params)
    if not directions:
        return 0.0
    # Strongest direction first -- _pick_directions returns them sorted by
    # angle, so re-rank by the energy at each.
    idx = [int(np.argmin(np.abs(angles - d))) for d in directions]
    best = directions[int(np.argmax([energy[i] for i in idx]))]
    rot = best % 90.0
    return rot - 90.0 if rot > 45.0 else rot


def _rotate(occ: np.ndarray, degrees: float) -> tuple[np.ndarray, np.ndarray]:
    """Rotate an occupancy grid about its centre, growing the canvas to fit.

    Returns the rotated grid and the 2x3 affine that produced it. Nearest
    neighbour keeps the three occupancy levels exact, and the new border is
    unknown rather than free so the rotation cannot invent open space.
    """
    h, w = occ.shape
    matrix = cv2.getRotationMatrix2D((w / 2.0, h / 2.0), degrees, 1.0)
    cos, sin = abs(matrix[0, 0]), abs(matrix[0, 1])
    out_w = int(round(h * sin + w * cos))
    out_h = int(round(h * cos + w * sin))
    matrix[0, 2] += out_w / 2.0 - w / 2.0
    matrix[1, 2] += out_h / 2.0 - h / 2.0
    rotated = cv2.warpAffine(
        occ,
        matrix,
        (out_w, out_h),
        flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=int(UNKNOWN),
    )
    return rotated, matrix


def _cut_lines(wall: np.ndarray, axis: int, run_px: int, nms_px: int) -> list[int]:
    """Positions along ``axis`` of the wall lines perpendicular to it.

    A wall only votes for a cut where it actually extends: eroding with a line
    kernel keeps the pixels sitting in an unbroken run of at least ``run_px``,
    so a long horizontal wall contributes nothing to the vertical-line scan and
    a lone speck of clutter contributes nothing to either.

    "Unbroken" is measured after closing pinholes along the same direction. A
    wall is about three pixels thick at 5 cm, and a scan that missed one cell of
    it -- or a rotation that resampled one away -- would otherwise cut a real
    wall into two runs too short to vote.
    """
    length = wall.shape[1 - axis]  # the axis the positions are measured along
    kernel = np.ones((run_px, 1) if axis == 0 else (1, run_px), np.uint8)
    bridge = np.ones((5, 1) if axis == 0 else (1, 5), np.uint8)
    solid = cv2.morphologyEx(wall.astype(np.uint8), cv2.MORPH_CLOSE, bridge)
    extended = cv2.erode(solid, kernel)
    score = extended.sum(axis=axis).astype(float)

    chosen: list[int] = []
    for idx in np.argsort(-score):
        if score[idx] < 1:
            break
        if all(abs(int(idx) - c) >= nms_px for c in chosen):
            chosen.append(int(idx))
    return sorted(set(chosen + [0, length - 1]))


def _is_opening(wall_band: np.ndarray, free_band: np.ndarray, door_px: float) -> bool:
    """Is the gap in this boundary band wider than a door?

    An opening is an unbroken stretch of the boundary with no wall across it,
    and the *longest* such stretch is what counts -- a room with two doors in
    one wall has two 0.9 m gaps, not one 1.8 m one.

    The stretch must also be observed free over more than a door's width.
    Requiring that (rather than free everywhere) is what lets a room whose
    middle is a lidar shadow still come out whole, while a boundary that is
    unknown from end to end -- two mapped pockets with unexplored space between
    them -- stays unmerged, because nothing was ever seen to connect them.
    """
    clear = ~wall_band.any(axis=1)
    seen = free_band.all(axis=1)
    run = free = 0
    for is_clear, is_free in zip(clear, seen):
        if not is_clear:
            run = free = 0
            continue
        run += 1
        free += bool(is_free)
        if run > door_px and free > door_px:
            return True
    return False


class _Union:
    def __init__(self, n: int):
        self.parent = list(range(n))

    def find(self, a: int) -> int:
        while self.parent[a] != a:
            self.parent[a] = self.parent[self.parent[a]]
            a = self.parent[a]
        return a

    def union(self, a: int, b: int):
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.parent[rb] = ra


def _inset(mask: np.ndarray, inset_px: int) -> np.ndarray:
    """Pull a face union back off the wall centrelines its cut lines sit on.

    Cut lines run down the middle of the walls they were found in, so a raw
    union claims half a wall thickness of solid on every side. A rectangular
    kernel takes that back uniformly and keeps the shape rectilinear.
    """
    if inset_px <= 0:
        return mask.astype(np.uint8)
    size = 2 * inset_px + 1
    return cv2.erode(
        mask.astype(np.uint8), cv2.getStructuringElement(cv2.MORPH_RECT, (size, size))
    )


def _polygon_from_mask(
    work: np.ndarray, simplify_px: float
) -> tuple[np.ndarray, float] | None:
    """The outline of an inset face union, with the area of its largest hole.

    A zones footprint is a single outline and cannot express a hole, so a region
    that wraps around something -- a corridor ring enclosing a block of wards --
    would silently claim everything it encircles. That area is the caller's
    warning.
    """
    contours, hierarchy = cv2.findContours(
        work, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE
    )
    if not contours:
        return None
    outer = [i for i in range(len(contours)) if hierarchy[0][i][3] < 0]
    best = max(outer, key=lambda i: cv2.contourArea(contours[i]))
    hole_px = max(
        (
            cv2.contourArea(contours[i])
            for i in range(len(contours))
            if hierarchy[0][i][3] == best
        ),
        default=0.0,
    )
    approx = cv2.approxPolyDP(contours[best], max(simplify_px, 0.01), True)
    if len(approx) < 3:
        return None
    return approx.reshape(-1, 2).astype(np.float64), float(hole_px)


def _interior(polygon_px: np.ndarray, shape: tuple[int, int]) -> np.ndarray:
    """Pixels strictly inside an outline, eroded so none sits on its boundary."""
    filled = np.zeros(shape, np.uint8)
    cv2.fillPoly(filled, [np.round(polygon_px).astype(np.int32)], 1)
    return cv2.erode(filled, np.ones((3, 3), np.uint8)).astype(bool)


def segment_rooms(
    occ: np.ndarray,
    geometry: MapGeometry,
    params: RoomParams | None = None,
    name_prefix: str = "room",
) -> SegmentationResult:
    """Partition the observed free space of ``occ`` into candidate rooms."""
    params = params or RoomParams()
    res = geometry.resolution

    rotation = dominant_rotation_deg(occ) if params.align else 0.0
    if abs(rotation) < params.align_min_deg:
        rotation = 0.0
    if rotation:
        grid, matrix = _rotate(occ, rotation)
        inverse = cv2.invertAffineTransform(matrix)
    else:
        grid, inverse = occ, None

    wall = grid <= (OCCUPIED + 20)
    free = grid >= (FREE - 20)

    def px(metres: float, minimum: int = 1) -> int:
        return max(minimum, int(round(metres / res)))

    cuts_x = _cut_lines(wall, 0, px(params.min_wall_run_m, 3), px(params.line_merge_m))
    cuts_y = _cut_lines(wall, 1, px(params.min_wall_run_m, 3), px(params.line_merge_m))

    band = px(params.wall_thickness_m, 1)  # half-width of a wall's pixel footprint
    door_px = params.door_max_m / res
    inset = max(1, int(round(params.wall_thickness_m / (2 * res))))

    cells: dict[tuple[int, int], int] = {}
    boxes: list[tuple[int, int, int, int]] = []  # y0, y1, x0, x1 (half-open)
    free_px: list[int] = []
    for j in range(len(cuts_y) - 1):
        for i in range(len(cuts_x) - 1):
            y0, y1 = cuts_y[j], cuts_y[j + 1]
            x0, x1 = cuts_x[i], cuts_x[i + 1]
            patch = free[y0:y1, x0:x1]
            if patch.size == 0:
                continue
            count = int(patch.sum())
            if count / patch.size < params.min_free_frac:
                continue
            cells[(i, j)] = len(boxes)
            boxes.append((y0, y1, x0, x1))
            free_px.append(count)

    union = _Union(len(boxes))
    for (i, j), cell in cells.items():
        y0, y1, x0, x1 = boxes[cell]
        right = cells.get((i + 1, j))
        if right is not None:
            cut = slice(max(cuts_x[i + 1] - band, 0), cuts_x[i + 1] + band + 1)
            if _is_opening(wall[y0:y1, cut], free[y0:y1, cut], door_px):
                union.union(cell, right)
        below = cells.get((i, j + 1))
        if below is not None:
            cut = slice(max(cuts_y[j + 1] - band, 0), cuts_y[j + 1] + band + 1)
            if _is_opening(wall[cut, x0:x1].T, free[cut, x0:x1].T, door_px):
                union.union(cell, below)

    groups: dict[int, list[int]] = {}
    for cell in range(len(boxes)):
        groups.setdefault(union.find(cell), []).append(cell)

    distance = cv2.distanceTransform(
        (~wall).astype(np.uint8), cv2.DIST_L2, 5
    )  # unknown counts as passable here: it is never wall

    candidates: list[dict] = []
    encircling = 0
    labels = np.zeros(grid.shape, np.int32)
    for members in groups.values():
        area = sum(free_px[c] for c in members) * res * res
        if area < params.min_room_area_m2:
            continue
        mask = np.zeros(grid.shape, bool)
        for cell in members:
            y0, y1, x0, x1 = boxes[cell]
            mask[y0:y1, x0:x1] = True
        outline = _inset(mask, inset)
        outlined = _polygon_from_mask(outline, params.simplify_m / res)
        if outlined is None:
            continue
        polygon_px, hole_px = outlined
        if hole_px * res * res >= params.min_room_area_m2:
            encircling += 1
            continue
        # Pose from inside the simplified outline itself, not merely the region
        # it came from: "go to the kitchen" and "am I in the kitchen" read the
        # same polygon, and simplification can cut a corner off the region.
        inside = _interior(polygon_px, grid.shape)
        interior = distance * (inside & free)
        if not interior.any():
            interior = distance * inside
        if not interior.any():
            continue
        row, col = np.unravel_index(int(np.argmax(interior)), grid.shape)
        candidates.append(
            {
                "polygon": _to_world(polygon_px, inverse, geometry),
                "pose": _to_world(
                    np.array([[col + 0.5, row + 0.5]]), inverse, geometry
                )[0],
                "area_m2": round(area, 2),
                "clearance_m": round(float(interior[row, col]) * res, 2),
                "mask": mask,
            }
        )

    rooms = []
    order = sorted(candidates, key=lambda c: -c["area_m2"])
    for rank, candidate in enumerate(order, start=1):
        rooms.append(Room(name=f"{name_prefix}_{rank:02d}", **candidate))
        labels[candidate["mask"]] = rank

    return SegmentationResult(
        rooms=rooms,
        rotation_deg=round(rotation, 2),
        cuts_x=cuts_x,
        cuts_y=cuts_y,
        faces=len(boxes),
        encircling=encircling,
        labels=labels,
    )


def _to_world(
    points_px: np.ndarray, inverse: np.ndarray | None, geometry: MapGeometry
) -> list[tuple[float, float]]:
    """Rotated-frame pixels -> source pixels -> map-frame metres."""
    pts = np.asarray(points_px, np.float64)
    if inverse is not None:
        pts = cv2.transform(pts.reshape(-1, 1, 2), inverse).reshape(-1, 2)
    return [geometry.to_world(float(x), float(y)) for x, y in pts]


def polygon_area(polygon) -> float:
    """Shoelace area of an outline, in the units of its vertices."""
    total = 0.0
    for (ax, ay), (bx, by) in zip(polygon, list(polygon[1:]) + [polygon[0]]):
        total += ax * by - bx * ay
    return abs(total) / 2.0


def polygon_contains(polygon, x: float, y: float) -> bool:
    """Ray-cast membership, mirroring ``mote_tasks.zones.Polygon.contains``."""
    inside = False
    for (ax, ay), (bx, by) in zip(polygon, list(polygon[1:]) + [polygon[0]]):
        if (ay > y) != (by > y):
            if x < ax + (y - ay) * (bx - ax) / (by - ay):
                inside = not inside
    return inside
