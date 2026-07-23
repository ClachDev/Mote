#!/usr/bin/env python3
"""Cross-section the mount STLs to see which face each nut pocket opens on.

[hole_survey.py](hole_survey.py) counts hex nut pockets but cannot tell an
*open-top* pocket (nut retained by the component that sits on the mount) from a
*side-loaded* or *open-bottom* one (nut captive in the mount or held by the
plate). That distinction drives the fastening discussion, so this tool measures
it directly rather than inferring it from the wall normals.

Method: no topology guessing. For each cell of a section plane we cast a ray
along the plane normal and count triangle crossings; an odd count means the cell
is inside solid. Printing the occupancy as ASCII reveals whether a pocket is
capped above, capped below, or open. Run from the repo root:

    python3 design/research/pocket_section.py
"""
import struct
from pathlib import Path

import numpy as np

STL_DIR = Path(__file__).resolve().parent.parent / "stl"


def load(path):
    with open(path, "rb") as fh:
        fh.read(80)
        n = struct.unpack("<I", fh.read(4))[0]
        rec = np.fromfile(fh, dtype=np.uint8).reshape(n, 50)
    return rec[:, 12:48].copy().view("<f4").reshape(n, 3, 3)


def inside(verts, P, axis):
    """Ray-cast from each point in P along +axis; True where inside the solid."""
    o = [i for i in range(3) if i != axis]
    a, b, c = verts[:, 0, o], verts[:, 1, o], verts[:, 2, o]
    az, bz, cz = verts[:, 0, axis], verts[:, 1, axis], verts[:, 2, axis]
    out = np.zeros(len(P), bool)
    for k, p in enumerate(P[:, o]):
        d1 = (b[:, 0] - a[:, 0]) * (p[1] - a[:, 1]) - (b[:, 1] - a[:, 1]) * (p[0] - a[:, 0])
        d2 = (c[:, 0] - b[:, 0]) * (p[1] - b[:, 1]) - (c[:, 1] - b[:, 1]) * (p[0] - b[:, 0])
        d3 = (a[:, 0] - c[:, 0]) * (p[1] - c[:, 1]) - (a[:, 1] - c[:, 1]) * (p[0] - c[:, 0])
        hit = ((d1 >= 0) & (d2 >= 0) & (d3 >= 0)) | ((d1 <= 0) & (d2 <= 0) & (d3 <= 0))
        hit &= (np.abs(d1) + np.abs(d2) + np.abs(d3)) > 1e-9
        if not hit.any():
            continue
        w = np.stack([d2, d3, d1], 1)[hit]
        w = w / w.sum(1, keepdims=True)
        zc = w[:, 0] * az[hit] + w[:, 1] * bz[hit] + w[:, 2] * cz[hit]
        out[k] = np.count_nonzero(zc > P[k, axis] + 1e-6) % 2 == 1
    return out


def section(verts, slice_axis, frac, cols=68, rows=22):
    """Vertical section: slice_axis is held constant at `frac` of its extent."""
    mn, mx = verts.reshape(-1, 3).min(0), verts.reshape(-1, 3).max(0)
    o = [i for i in range(3) if i != slice_axis]
    vert = 2 if 2 in o else o[1]
    hor = o[0] if o[0] != vert else o[1]
    level = mn[slice_axis] + frac * (mx[slice_axis] - mn[slice_axis])
    u = np.linspace(mn[hor], mx[hor], cols)
    v = np.linspace(mx[vert], mn[vert], rows)
    UU, VV = np.meshgrid(u, v)
    P = np.zeros((cols * rows, 3))
    P[:, hor], P[:, vert], P[:, slice_axis] = UU.ravel(), VV.ravel(), level
    grid = inside(verts, P, slice_axis).reshape(rows, cols)
    print(f"  {'xyz'[hor]}-{'xyz'[vert]} section at {'xyz'[slice_axis]}={level:.1f} "
          f"({'xyz'[vert]}: {mx[vert]:.0f} top .. {mn[vert]:.0f} bottom)")
    for r in grid:
        print("    " + "".join("#" if x else "." for x in r))


# (part, slice_axis, fraction, one-line read of the nut-pocket opening)
VIEWS = [
    ("Battery Mount", 0, 0.5, "pockets open DOWN to the base plate (plate-retained)"),
    ("C1 Lidar Mount", 1, 0.5, "pockets open UP into the lidar tray (lidar retains the nut)"),
    ("Waveshare Mount", 1, 0.5, "pockets open UP under the board (board retains the nut)"),
    ("Motor Support", 1, 0.5, "servo-lug nuts sit in open air (no pocket) -> the joint that loosens"),
]

if __name__ == "__main__":
    for name, ax, frac, note in VIEWS:
        p = STL_DIR / f"{name}.stl"
        if not p.exists():
            continue
        print(f"\n=== {name} — {note} ===")
        section(load(p), ax, frac)
