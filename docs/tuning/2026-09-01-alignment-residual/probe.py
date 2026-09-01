"""What the mapping pipeline's alignment step can and cannot assert.

Reads the seven maps banked by the 2026-08-25 build-params run (the sibling
directory), which are the same bag solved with and without a ``--frame 0 0
-3.0`` birth-alignment injection, and measures each one four ways:

* ``angular_stats.wall_rotation`` — the primitive the design's alignment step
  names;
* ``room_segmentation.dominant_rotation_deg`` — the other FFT estimator in the
  tree, square-padded;
* a projection-sharpness sweep — rotate the wall mask, score how concentrated
  the row and column projections are, take the peak. Independent of both;
* the same sweep per tile, which is what says whether "the map's wall rotation"
  is a single number at all.

Run from a checkout root with the default pixi env's python (numpy, Pillow,
cv2 — no ROS):

    .pixi/envs/default/bin/python \
        docs/tuning/2026-09-01-alignment-residual/probe.py [outdir]

Throwaway measurement code, not a supported tool: the primitive this argues for
is deferred work. It is committed so the tables in
``docs/tuning/2026-09-01-alignment-residual.md`` can be re-checked.
"""

from __future__ import annotations

import os
import sys

import cv2
import numpy as np
from PIL import Image

sys.path.insert(0, "mote_bringup")
from mote_bringup.map_cleanup.angular_stats import wall_rotation  # noqa: E402
from mote_bringup.map_cleanup.room_segmentation import dominant_rotation_deg  # noqa: E402

OCCUPIED = 20  # map.png: 0 occupied, 205 unknown, 254 free
UNKNOWN = 205
SRC = "docs/tuning/2026-08-25-slam-build-params"
INJECTED_DEG = -3.0  # what the birth-aligned legs asked for
LEGS = ("car-0.0349", "car-0.0175", "car-0.01745", "car-0.01745-repeat")


def maps():
    for cond in ("unaligned", "birth-aligned"):
        for leg in LEGS:
            path = f"{SRC}/{cond}/{leg}/map.png"
            if os.path.exists(path):
                yield cond, leg, path


def grey(path):
    return np.array(Image.open(path).convert("L")).astype(np.float32)


def wall_of(img):
    return (img <= OCCUPIED).astype(np.float32)


def _rotation(img, degrees):
    """The affine and output size that rotate about the centre without cropping.

    Positive degrees is anticlockwise.
    """
    h, w = img.shape
    m = cv2.getRotationMatrix2D((w / 2.0, h / 2.0), degrees, 1.0)
    cos, sin = abs(m[0, 0]), abs(m[0, 1])
    nw, nh = int(h * sin + w * cos) + 1, int(h * cos + w * sin) + 1
    m[0, 2] += nw / 2.0 - w / 2.0
    m[1, 2] += nh / 2.0 - h / 2.0
    return m, (nw, nh)


def rotate(img, degrees, border=0.0):
    m, size = _rotation(img, degrees)
    return cv2.warpAffine(
        img,
        m,
        size,
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=border,
    )


def sharpness(wall):
    """Concentration of the row and column projections, scale-free.

    Axis-aligned walls pile into few rows and columns, so the sum of squared
    projections peaks there. Normalised by the wall-pixel count squared so
    rotations that resample away a pixel or two cannot move the score.
    """
    n = wall.sum()
    if n <= 0:
        return 0.0
    rows = wall.sum(axis=1)
    cols = wall.sum(axis=0)
    return float((rows**2).sum() + (cols**2).sum()) / float(n * n)


def sweep(wall, lo=-45.0, hi=45.0, step=0.25):
    """Wall rotation by projection sharpness, parabolically refined."""
    angles = np.arange(lo, hi + 1e-9, step)
    scores = np.array([sharpness(rotate(wall, -a)) for a in angles])
    i = int(np.argmax(scores))
    if 0 < i < len(angles) - 1:
        y0, y1, y2 = scores[i - 1], scores[i], scores[i + 1]
        d = y0 - 2 * y1 + y2
        delta = 0.5 * (y0 - y2) / d if d != 0 else 0.0
    else:
        delta = 0.0
    return float(angles[i] + delta * step), angles, scores


def fold(a):
    """Signed offset from the nearest axis, in (-45, 45]."""
    if a is None:
        return float("nan")
    r = a % 90.0
    return r - 90.0 if r > 45.0 else r


def table_a(out):
    print("A. the same maps under three estimators (deg off axis)", file=out)
    print(
        f"{'condition':<14} {'leg':<20} {'wall_rotation':>14} {'efrac':>6}"
        f" {'dominant_rot':>13} {'projection':>11}",
        file=out,
    )
    result = {}
    for cond, leg, path in maps():
        img = grey(path)
        wall = wall_of(img)
        wr = wall_rotation(wall.astype(bool))
        dr = dominant_rotation_deg(img)
        pk, _, _ = sweep(wall)
        result[(cond, leg)] = pk
        print(
            f"{cond:<14} {leg:<20} {fold(wr['angle_deg']):>14.3f}"
            f" {wr['energy_frac']:>6.3f} {dr:>13.3f} {pk:>11.3f}",
            file=out,
        )
    return result


def table_b(out):
    """Each estimator's response to a known rotation of a real map."""
    print(file=out)
    print(
        "B. response to a known rotation applied to birth-aligned/car-0.0349", file=out
    )
    img = grey(f"{SRC}/birth-aligned/car-0.0349/map.png")
    wall = wall_of(img)
    base_wr = fold(wall_rotation(wall.astype(bool))["angle_deg"])
    base_dr = dominant_rotation_deg(img)
    base_pk, _, _ = sweep(wall, -10.0, 10.0)
    print(
        f"   base readings: wall_rotation {base_wr:+.3f}  dominant_rot"
        f" {base_dr:+.3f}  projection {base_pk:+.3f}",
        file=out,
    )
    print(
        f"{'applied':>8} {'wall_rot d':>11} {'err':>7} {'dom_rot d':>10}"
        f" {'err':>7} {'proj d':>8} {'err':>7}",
        file=out,
    )
    for applied in (0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 5.0, 10.0):
        rot_img = rotate(img, applied, border=float(UNKNOWN))
        rot_wall = wall_of(rot_img)
        # wall_rotation and dominant_rotation_deg carry the opposite sign
        # convention to the sweep, so each is compared on the magnitude of its
        # own movement from its own base reading.
        d_wr = abs(fold(wall_rotation(rot_wall.astype(bool))["angle_deg"]) - base_wr)
        d_dr = abs(dominant_rotation_deg(rot_img) - base_dr)
        pk, _, _ = sweep(rot_wall, -10.0, 14.0)
        d_pk = abs(pk - base_pk)
        print(
            f"{applied:>8.2f} {d_wr:>11.3f} {d_wr - applied:>7.3f}"
            f" {d_dr:>10.3f} {d_dr - applied:>7.3f}"
            f" {d_pk:>8.3f} {d_pk - applied:>7.3f}",
            file=out,
        )


def table_c(out, peaks):
    """Did the injected yaw put the map where it was asked to go?"""
    print(file=out)
    print(
        f"C. effect of the {INJECTED_DEG:+.1f} deg injection, by projection sweep",
        file=out,
    )
    print(
        f"{'leg':<20} {'unaligned':>10} {'birth-aligned':>14} {'delta':>8}"
        f" {'vs injected':>12}",
        file=out,
    )
    for leg in LEGS:
        u, b = peaks.get(("unaligned", leg)), peaks.get(("birth-aligned", leg))
        if u is None or b is None:
            continue
        delta = b - u
        print(
            f"{leg:<20} {u:>10.3f} {b:>14.3f} {delta:>8.3f}"
            f" {delta - INJECTED_DEG:>12.3f}",
            file=out,
        )


def table_d(out):
    """Is 'the map's wall rotation' one number? Thirds of two maps."""
    print(file=out)
    print("D. projection sweep per ninth of the map (deg off axis)", file=out)
    for cond, leg in (("unaligned", "car-0.0349"), ("birth-aligned", "car-0.0175")):
        wall = wall_of(grey(f"{SRC}/{cond}/{leg}/map.png"))
        whole, _, _ = sweep(wall)
        print(f"   {cond}/{leg}: whole map {whole:+.2f}", file=out)
        h, w = wall.shape
        tiles = []
        for iy in range(3):
            row = []
            for ix in range(3):
                tile = wall[
                    iy * h // 3 : (iy + 1) * h // 3, ix * w // 3 : (ix + 1) * w // 3
                ]
                if tile.sum() < 1500:
                    row.append(None)
                    continue
                row.append(sweep(tile, -20.0, 20.0)[0])
            tiles.append(row)
            print(
                "     "
                + " ".join("      —" if v is None else f"{v:+7.2f}" for v in row),
                file=out,
            )
        flat = [v for row in tiles for v in row if v is not None]
        near = [v for v in flat if abs(v) < 10.0]
        print(
            f"     spread across all tiles {max(flat) - min(flat):.1f} deg;"
            f" within the dominant family {max(near) - min(near):.1f} deg",
            file=out,
        )


def pictures(outdir):
    """The eye is the arbiter when two estimators disagree by 3 deg.

    Only the squared image is written; the as-solved one is the sibling
    directory's own ``map.png``, referenced rather than copied. Nearest
    resampling keeps the output a three-value occupancy image.
    """
    for cond in ("unaligned", "birth-aligned"):
        img = grey(f"{SRC}/{cond}/car-0.0349/map.png")
        peak, angles, scores = sweep(wall_of(img))
        squared = cv2.warpAffine(
            img,
            *_rotation(img, -peak),
            flags=cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=float(UNKNOWN),
        )
        Image.fromarray(np.clip(squared, 0, 255).astype(np.uint8)).save(
            os.path.join(outdir, f"{cond}-squared.png")
        )
        np.savetxt(
            os.path.join(outdir, f"sharpness-{cond}-car-0.0349.txt"),
            np.column_stack([angles, scores]),
            header=f"angle_deg sharpness (peak {peak:+.3f})",
            fmt="%.4f %.8f",
        )


def main():
    outdir = (
        sys.argv[1]
        if len(sys.argv) > 1
        else "docs/tuning/2026-09-01-alignment-residual"
    )
    os.makedirs(outdir, exist_ok=True)
    with open(os.path.join(outdir, "results.txt"), "w") as out:
        peaks = table_a(out)
        table_b(out)
        table_c(out, peaks)
        table_d(out)
    pictures(outdir)
    print(open(os.path.join(outdir, "results.txt")).read())


if __name__ == "__main__":
    main()
