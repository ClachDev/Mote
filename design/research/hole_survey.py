#!/usr/bin/env python3
"""Survey the fastening features of the printed parts from their STL meshes.

For every STL in design/stl/ this reports, per axis:
  - circular hole loops (diameter, count) — 3.5 mm loops are ORP-grid bolt holes
  - hexagonal pockets (M3 nut traps: a hex across-flats 5.5 / across-corners
    6.35 shows up here as a loop with mean diameter ~5.5-6 mm and a large
    radius spread, unlike a circle)
  - nearest-neighbour spacing between 3.5 mm holes (the mounting grid pitch)

Only numpy is required. Run from the repo root:

    python3 design/research/hole_survey.py
"""

import itertools
import struct
from collections import Counter
from pathlib import Path

import numpy as np

STL_DIR = Path(__file__).resolve().parent.parent / "stl"


def load(path):
    with open(path, "rb") as fh:
        fh.read(80)
        n = struct.unpack("<I", fh.read(4))[0]
        rec = np.fromfile(fh, dtype=np.uint8).reshape(n, 50)
    normals = rec[:, 0:12].copy().view("<f4").reshape(n, 3)
    verts = rec[:, 12:48].copy().view("<f4").reshape(n, 3, 3)
    return normals, verts


def cluster(pts, tol):
    """Group 2D points into connected clusters (union-find on a coarse grid)."""
    parent = list(range(len(pts)))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    cells = {}
    for i, g in enumerate((pts / tol).astype(int)):
        cells.setdefault(tuple(g), []).append(i)
    for key, idxs in cells.items():
        for off in itertools.product((-1, 0, 1), repeat=2):
            for j in cells.get((key[0] + off[0], key[1] + off[1]), []):
                for i in idxs:
                    if i < j and np.hypot(*(pts[i] - pts[j])) < tol:
                        ra, rb = find(i), find(j)
                        if ra != rb:
                            parent[ra] = rb
    groups = {}
    for i in range(len(pts)):
        groups.setdefault(find(i), []).append(i)
    return [pts[np.array(v)] for v in groups.values()]


def survey(path):
    normals, verts = load(path)
    dims = verts.reshape(-1, 3).max(0) - verts.reshape(-1, 3).min(0)
    print(f"\n=== {path.name}  ({dims[0]:.1f} x {dims[1]:.1f} x {dims[2]:.1f} mm) ===")
    for ax in range(3):
        wall = np.abs(normals[:, ax]) < 0.1
        if not wall.any():
            continue
        other = [i for i in range(3) if i != ax]
        pts = np.unique(np.round(verts[wall].reshape(-1, 3)[:, other], 3), axis=0)
        holes, hexes = [], []
        for loop in cluster(pts, 1.2):
            if len(loop) < 6:
                continue
            ctr = loop.mean(0)
            r = np.linalg.norm(loop - ctr, axis=1)
            d = r.mean() * 2
            if not 1.0 < d < 9.0:
                continue
            if (r.max() - r.min()) > 0.25 and 4.5 < d < 7.5:
                hexes.append(d)
            else:
                holes.append((ctr, d))
        if not holes and not hexes:
            continue
        dias = Counter(round(d, 1) for _, d in holes)
        line = f"  axis {'xyz'[ax]}: holes {dict(sorted(dias.items()))}"
        if hexes:
            line += f"  hex pockets: {sorted(round(d, 1) for d in hexes)}"
        print(line)
        grid = np.array([c for c, d in holes if 3.0 < d < 4.2])
        if len(grid) > 1:
            nn = []
            for i in range(len(grid)):
                dd = np.hypot(*(grid - grid[i]).T)
                dd[i] = np.inf
                nn.append(dd.min())
            pitches = Counter(round(x) for x in nn)
            print(f"    3.5mm-hole grid pitch (NN): {dict(sorted(pitches.items()))}")


if __name__ == "__main__":
    for stl in sorted(STL_DIR.glob("*.stl")):
        survey(stl)
