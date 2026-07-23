"""Render a replayed occupancy grid to a PNG, optionally with the trajectory.

Uses cv2 (already a project dependency — map_cleanup and sites.py save maps with
it), matching map_saver's convention: occupied black, free white, unknown grey,
north up (grid row 0 is the bottom, so the image is flipped vertically). The
estimator trajectory, if given, is drawn in the map frame over the grid.
"""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

OCC_THRESH = 65
FREE_THRESH = 25


def grid_to_bgr(grid: np.ndarray) -> np.ndarray:
    """int8 occupancy grid (0..100, -1 unknown) -> flipped BGR image."""
    img = np.full(grid.shape, 205, dtype=np.uint8)  # unknown grey (map_saver value)
    img[(grid >= 0) & (grid <= FREE_THRESH)] = 254  # free
    img[grid >= OCC_THRESH] = 0  # occupied
    img = np.flipud(img)  # ROS grid origin is bottom-left; image north is up
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def render_map(npz_path, out_png, traj=None) -> None:
    data = np.load(npz_path)
    grid = data["grid"]
    res = float(data["resolution"])
    ox, oy = (float(v) for v in data["origin"])
    h = grid.shape[0]
    img = grid_to_bgr(grid)

    if traj:
        pts = []
        for row in traj:
            _, x, y = row[0], row[1], row[2]
            px = int(round((x - ox) / res))
            py = int(round((y - oy) / res))
            py = h - 1 - py  # match the vertical flip
            pts.append([px, py])
        if len(pts) >= 2:
            cv2.polylines(img, [np.array(pts, dtype=np.int32)], False, (0, 128, 255), 1)
            cv2.circle(img, tuple(pts[0]), 3, (0, 200, 0), -1)  # start green
            cv2.circle(img, tuple(pts[-1]), 3, (0, 0, 255), -1)  # end red

    # Upscale small maps so features are legible in the report.
    scale = max(1, int(round(900 / max(img.shape[:2]))))
    if scale > 1:
        img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    Path(out_png).parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_png), img)
