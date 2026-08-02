"""FFT-based structure extraction for decluttering 2D occupancy grid maps.

Implements the structure-identification method described in ROSE / ROSE2
(Robust Structure identification and rOom SEgmentation, arXiv:2203.03519):
straight walls in an occupancy map concentrate their Fourier energy along a few
dominant orientations, while clutter, sensor speckle and ragged edges spread
energy uniformly across all orientations. Detecting those dominant orientations
and keeping only the frequency content aligned with them reconstructs a clean,
structural map and discards the noise.

The pipeline here is purely geometric (no training) and depends only on numpy
and OpenCV:

    1. binarise the occupancy grid into a wall image,
    2. take its 2D FFT and measure spectral energy as a function of angle,
    3. pick the dominant orientations (local maxima of that angular energy),
    4. keep only the frequency wedges around those orientations (a directional
       band-pass) and invert the FFT to get a continuous "structure score",
    5. threshold that score back into a decluttered occupancy grid.

Room segmentation (the ROSE2 layer on top of this) is intentionally out of
scope for this module.

Steps 2 and 3 — the angular spectrum and its peak-picking — live in
:mod:`angular_stats`, which is numpy-only so that scoring a map's angular
structure does not drag OpenCV in. This module keeps the filtering half.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np

from .angular_stats import _angular_energy, _pick_directions

# ROS map_server occupancy-PNG conventions.
FREE = 254
UNKNOWN = 205
OCCUPIED = 0


@dataclass
class StructureResult:
    """Everything the pipeline produces, for both output and diagnostics."""

    wall: np.ndarray  # bool, input walls (occupied cells)
    free: np.ndarray  # bool, input free cells
    spectrum: np.ndarray  # float, log magnitude spectrum (fftshifted)
    angles_deg: np.ndarray  # angular-energy sample angles, [0, 180)
    energy: np.ndarray  # angular energy per sample angle
    directions_deg: list[float] = field(default_factory=list)  # detected peaks
    mask: np.ndarray = None  # float, directional band-pass mask (fftshifted)
    score: np.ndarray = None  # float in [0, 1], reconstructed structure score
    clean_wall: np.ndarray = None  # bool, decluttered walls
    cleaned_map: np.ndarray = None  # uint8, ROS occupancy PNG


@dataclass
class Params:
    angle_step_deg: float = 0.5  # angular resolution of the energy scan
    lowcut_frac: float = 0.02  # ignore this central disc of the spectrum (DC)
    peak_rel_threshold: float = 0.45  # peak must reach this frac of the max
    peak_nms_deg: float = 12.0  # suppress peaks closer than this to a stronger one
    max_directions: int = 5  # keep at most this many orientations
    wedge_halfwidth_deg: float = 5.0  # angular half-width kept around each peak
    keep_dc_frac: float = 0.02  # always keep this central disc (baseline level)
    score_threshold: float | None = None  # None -> Otsu over the wall region
    min_component_area: int = 6  # drop cleaned blobs smaller than this (cells)
    dilate_gate_px: int = 2  # only keep cleaned walls near an original wall
    protect_observed_free: bool = True  # never turn observed-free cells into walls


def _binarise(occ: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Split a ROS occupancy PNG into (wall, free) boolean masks."""
    wall = occ <= (OCCUPIED + 20)
    free = occ >= (FREE - 20)
    return wall, free


def _directional_mask(
    shape: tuple[int, int], directions_deg: list[float], params: Params
) -> np.ndarray:
    """Frequency-domain band-pass keeping wedges around each orientation."""
    h, w = shape
    cy, cx = h / 2.0, w / 2.0
    yy, xx = np.mgrid[0:h, 0:w]
    dy = yy - cy
    dx = xx - cx
    ang = np.degrees(np.arctan2(dy, dx)) % 180.0
    radius = np.hypot(dx, dy)
    rmax = min(cy, cx)

    mask = np.zeros(shape, dtype=np.float32)
    for d in directions_deg:
        diff = np.minimum(np.abs(ang - d), 180.0 - np.abs(ang - d))
        mask[diff <= params.wedge_halfwidth_deg] = 1.0
    # Always preserve the DC neighbourhood so the reconstruction keeps its
    # overall intensity baseline rather than oscillating around zero.
    mask[radius <= params.keep_dc_frac * rmax] = 1.0
    return mask


def extract_structure(occ: np.ndarray, params: Params | None = None) -> StructureResult:
    """Run the full FFT declutter pipeline on a ROS occupancy PNG (uint8)."""
    params = params or Params()
    wall, free = _binarise(occ)

    signal = wall.astype(np.float32)
    fft = np.fft.fftshift(np.fft.fft2(signal))
    mag = np.abs(fft)
    spectrum_log = np.log1p(mag)

    angles, energy = _angular_energy(mag, params)
    directions = _pick_directions(angles, energy, params)

    mask = _directional_mask(signal.shape, directions, params)
    filtered = fft * mask
    recon = np.real(np.fft.ifft2(np.fft.ifftshift(filtered)))
    recon = np.clip(recon, 0, None)
    score = recon / (recon.max() + 1e-9)

    # Threshold the structure score into walls. Otsu over the score restricted
    # to where the reconstruction has any response gives a data-driven cut.
    if params.score_threshold is None:
        vals = (score[score > 0] * 255).astype(np.uint8)
        if vals.size:
            t, _ = cv2.threshold(vals, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            thr = t / 255.0
        else:
            thr = 0.5
    else:
        thr = params.score_threshold
    clean = score >= thr

    # Gate to the neighbourhood of original walls so we declutter/straighten
    # rather than hallucinate structure in never-observed space.
    gate = cv2.dilate(
        wall.astype(np.uint8),
        cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (2 * params.dilate_gate_px + 1,) * 2
        ),
    ).astype(bool)
    clean &= gate

    # A cell the robot actually observed as free must never become a wall: the
    # line-completing reconstruction would otherwise seal real doorways and
    # openings the robot drove through. Gap-filling is only legitimate in
    # never-observed (unknown) space.
    if params.protect_observed_free:
        clean &= ~free

    # Drop tiny speckle components.
    clean = _area_filter(clean, params.min_component_area)

    cleaned_map = _compose_map(occ, wall, free, clean)

    return StructureResult(
        wall=wall,
        free=free,
        spectrum=spectrum_log,
        angles_deg=angles,
        energy=energy,
        directions_deg=directions,
        mask=mask,
        score=score,
        clean_wall=clean,
        cleaned_map=cleaned_map,
    )


def _area_filter(mask: np.ndarray, min_area: int) -> np.ndarray:
    if min_area <= 1:
        return mask
    n, labels, stats, _ = cv2.connectedComponentsWithStats(
        mask.astype(np.uint8), connectivity=8
    )
    out = np.zeros_like(mask)
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] >= min_area:
            out[labels == i] = True
    return out


def _compose_map(
    occ: np.ndarray, wall: np.ndarray, free: np.ndarray, clean: np.ndarray
) -> np.ndarray:
    """Rebuild a ROS occupancy PNG from the cleaned walls.

    Free cells stay free, cleaned walls become occupied, everything the map
    never observed stays unknown. Original-wall cells that did not survive the
    filter become free (they were clutter sitting in observed space).
    """
    out = np.full_like(occ, UNKNOWN)
    out[free] = FREE
    out[wall & ~clean] = FREE  # rejected clutter, was observed as occupied
    out[clean] = OCCUPIED
    return out
