"""CLI: declutter a ROS occupancy-grid PNG and render a diagnostics panel.

    python -m mote_bringup.map_cleanup.cli INPUT.png [--out DIR]

Writes a cleaned map PNG and a side-by-side diagnostics image (input, Fourier
spectrum with detected orientations, angular-energy plot, cleaned map).
"""

from __future__ import annotations

import argparse
import os

import cv2
import numpy as np

from .structure_extraction import Params, StructureResult, extract_structure


def _to_bgr(gray: np.ndarray) -> np.ndarray:
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)


def _label(img: np.ndarray, text: str) -> np.ndarray:
    out = img.copy()
    cv2.rectangle(out, (0, 0), (out.shape[1], 18), (40, 40, 40), -1)
    cv2.putText(
        out,
        text,
        (4, 13),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.4,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return out


def _spectrum_panel(res: StructureResult, size: tuple[int, int]) -> np.ndarray:
    """Log spectrum as an image with detected orientation lines overlaid."""
    s = res.spectrum
    s = (s - s.min()) / (np.ptp(s) + 1e-9)
    img = _to_bgr((s * 255).astype(np.uint8))
    img = cv2.resize(img, (size[1], size[0]), interpolation=cv2.INTER_NEAREST)
    img = cv2.applyColorMap(img, cv2.COLORMAP_MAGMA)
    cy, cx = size[0] / 2.0, size[1] / 2.0
    r = min(cy, cx) * 0.95
    for d in res.directions_deg:
        rad = np.radians(d)
        dx, dy = np.cos(rad) * r, np.sin(rad) * r
        cv2.line(
            img,
            (int(cx - dx), int(cy - dy)),
            (int(cx + dx), int(cy + dy)),
            (80, 255, 80),
            1,
            cv2.LINE_AA,
        )
    return img


def _energy_panel(res: StructureResult, size: tuple[int, int]) -> np.ndarray:
    """Angular-energy curve g(theta) with detected peaks marked."""
    h, w = size
    img = np.full((h, w, 3), 30, np.uint8)
    e = res.energy / (res.energy.max() + 1e-9)
    n = len(e)
    for i in range(n - 1):
        x0 = int(i / n * (w - 1))
        x1 = int((i + 1) / n * (w - 1))
        y0 = int(h - 20 - e[i] * (h - 30))
        y1 = int(h - 20 - e[i + 1] * (h - 30))
        cv2.line(img, (x0, y0), (x1, y1), (200, 200, 60), 1, cv2.LINE_AA)
    for d in res.directions_deg:
        x = int(d / 180.0 * (w - 1))
        cv2.line(img, (x, 20), (x, h - 20), (80, 255, 80), 1, cv2.LINE_AA)
        cv2.putText(
            img,
            f"{d:.0f}",
            (min(x + 2, w - 28), 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.35,
            (80, 255, 80),
            1,
            cv2.LINE_AA,
        )
    cv2.putText(
        img,
        "angular energy vs orientation (deg)",
        (4, h - 6),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.35,
        (180, 180, 180),
        1,
        cv2.LINE_AA,
    )
    return img


def make_diagnostics(occ: np.ndarray, res: StructureResult) -> np.ndarray:
    h, w = occ.shape
    scale = max(1, int(round(360 / max(h, w))))
    size = (h * scale, w * scale)

    def up(img):
        return cv2.resize(img, (size[1], size[0]), interpolation=cv2.INTER_NEAREST)

    inp = _label(up(_to_bgr(occ)), "input map")
    spec = _label(
        _spectrum_panel(res, size), f"spectrum + {len(res.directions_deg)} dirs"
    )
    energy = _label(_energy_panel(res, size), "orientation energy")
    clean = _label(up(_to_bgr(res.cleaned_map)), "cleaned map")

    top = np.hstack([inp, spec])
    bottom = np.hstack([energy, clean])
    return np.vstack([top, bottom])


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("input", help="ROS occupancy-grid PNG")
    ap.add_argument(
        "--out", default=None, help="output directory (default: alongside input)"
    )
    ap.add_argument("--wedge", type=float, default=Params.wedge_halfwidth_deg)
    ap.add_argument("--peak-rel", type=float, default=Params.peak_rel_threshold)
    ap.add_argument("--gate", type=int, default=Params.dilate_gate_px)
    args = ap.parse_args(argv)

    occ = cv2.imread(args.input, cv2.IMREAD_GRAYSCALE)
    if occ is None:
        ap.error(f"could not read {args.input}")

    params = Params(
        wedge_halfwidth_deg=args.wedge,
        peak_rel_threshold=args.peak_rel,
        dilate_gate_px=args.gate,
    )
    res = extract_structure(occ, params)

    out_dir = args.out or os.path.dirname(os.path.abspath(args.input))
    os.makedirs(out_dir, exist_ok=True)
    base = os.path.splitext(os.path.basename(args.input))[0]
    clean_path = os.path.join(out_dir, f"{base}_cleaned.png")
    diag_path = os.path.join(out_dir, f"{base}_diagnostics.png")
    cv2.imwrite(clean_path, res.cleaned_map)
    cv2.imwrite(diag_path, make_diagnostics(occ, res))

    wall_before = int(res.wall.sum())
    wall_after = int(res.clean_wall.sum())
    print(f"detected orientations (deg): {[round(d, 1) for d in res.directions_deg]}")
    print(
        f"occupied cells: {wall_before} -> {wall_after} "
        f"({100 * wall_after / max(1, wall_before):.0f}% kept)"
    )
    print(f"wrote {clean_path}")
    print(f"wrote {diag_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
