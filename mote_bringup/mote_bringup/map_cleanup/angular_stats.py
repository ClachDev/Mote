"""Angular structure of a map's walls, from the FFT orientation spectrum.

The declutter pass (:mod:`structure_extraction`) already measures how a map's
Fourier energy distributes over wall orientations, in order to keep the wedges
around the dominant ones. This module reuses that same measurement to *score*
the map instead of filtering it: how many distinct directions the walls use,
how tightly the energy sits on them, and how those directions group into
orthogonal frames.

It answers a failure the crispness proxies (wall thickness, speckle, unknown
fraction) are blind to: a section of the map drawn at the wrong *angle*. A
drift-rotated room is crisp and unspeckled and still wrong.

What this can and cannot decide
-------------------------------
**Within a single map, a coherent drift-rotated section is angularly
indistinguishable from real architecture.** Both are a narrow extra family of
wall directions. A building with an angled hallway genuinely has three dominant
directions and always will; charging it for that would raise exactly the signal
this module exists to raise for drift, which is worse than no metric because it
would be believed.

So the split is:

* the scalars (``angular_support_deg``, ``angular_entropy_norm``,
  ``unassigned_energy_frac``) are **frame-model-free** and describe angular
  *smear and fragmentation*. They make no assumption about how many directions
  a building has;
* the **defect verdict** needs a prior, and is only produced when one is given.
  Pass ``reference_directions`` — the site's known wall directions, or those of
  a previously banked revision — and the ``reference_*`` keys report how far
  this map's energy sits from it.

The frame table is the diagnostic between the two: a drift-rotated section of a
rectilinear building duplicates that section's whole orthogonal frame (both
directions, ~90 deg apart), where an angled hallway adds *one* direction.

Two consumers, two entry points
-------------------------------
:func:`angular_stats` is the **tear alarm**. Loop drift only means anything when
the trajectory closes, so a mapping session that exits on its exploration budget
gives a scorer no drift number at all; for those the frame table is the only
automated tear signal there is. It is trustworthy for the tears that matter
(measured: run 3's two frames, 22.5 and 41 deg apart) and blind below roughly
its own merge tolerance — see :data:`FRAME_MERGE_DEG`.

:func:`wall_rotation` is the **alignment primitive**: windowed energy, folded
0/90, sub-bin interpolated. An alignment step measuring how far a map's wall
grid is turned should call it rather than re-deriving the fold, which is how one
hand-rolled attempt landed on a different answer than the next.

Conventions
-----------
Angles are in **image/pixel** coordinates (row-down), on [0, 180), and are not
map-frame bearings. A map frame's absolute rotation is an accident of where the
SLAM session started, so only *relative* angles here carry meaning; that is why
the reference comparison fits a global rotation offset before measuring.

numpy-only and ROS-free by design: :mod:`metrics` imports this lazily so a
benchmark run in an environment without OpenCV, or without ``mote_bringup`` on
the path at all, still scores.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

__all__ = [
    "SpectrumParams",
    "angular_stats",
    "wall_rotation",
    "fold_90",
    "refine_peak",
    "_angular_energy",
    "_smooth_circular",
    "_pick_directions",
    "_angdist",
]


@dataclass
class SpectrumParams:
    """The spectrum-scan half of :class:`structure_extraction.Params`.

    The declutter pass owns the full ``Params`` and passes it straight in — the
    helpers below only read attributes, so it duck-types. This module carries
    its own defaults instead of importing that one because the dependency has to
    run the other way: ``structure_extraction`` imports cv2, and both
    :mod:`metrics` and this module's callers need the angular maths to stay
    numpy-only and cheap to import. Field names and defaults match ``Params``.
    """

    angle_step_deg: float = 0.5
    lowcut_frac: float = 0.02
    peak_rel_threshold: float = 0.45
    peak_nms_deg: float = 12.0
    max_directions: int = 5
    wedge_halfwidth_deg: float = 5.0


# Half-width of the broadband floor estimated and subtracted from the angular
# energy, in degrees. Load-bearing: on an un-subtracted spectrum clutter
# dominates and the angular energy of a torn map is nearly uniform, so a torn
# map can score *better* than a clean one (measured, 2026-07-29).
FLOOR_HALFWIDTH_DEG = 45.0

# Relative peak threshold used when picking the direction families the stats are
# reported against. Deliberately *not* ``Params.peak_rel_threshold`` (0.45),
# which selects wedges to keep in the declutter reconstruction — a different job
# where being conservative is right. At 0.45, picking on the floor-subtracted
# residual drops a whole family on some rotations of an unchanged map and its
# energy lands in ``unassigned_energy_frac`` (measured 0.099 -> 0.181 -> 0.163
# across 0/+17/-31 deg). At 0.15 the same map gives 0.099 / 0.071 / 0.084.
STATS_PEAK_REL_THRESHOLD = 0.15

# At most this many direction families. ``Params.max_directions`` is 5, which
# censors the table: every real map measured hits that cap.
STATS_MAX_DIRECTIONS = 12

# Directions merge into one orthogonal frame when their ``angle mod 90`` values
# are within this. Bounded from below by the shear a genuine frame carries (the
# run-3 conservative leg's own frame is internally sheared 7.5 deg, and splitting
# that into two frames would invent a tear), and from above by the smallest tear
# worth resolving. Measured on the synthetic fixtures: at 10 deg a 13 deg rotated
# section reads as a second frame of 2 directions while an angled corridor stays
# 1, and at 12 deg the rotated section's near wall family merges back into the
# dominant frame and the two become indistinguishable. Run 3's real 23-38 deg
# tear reads identically at either.
FRAME_MERGE_DEG = 10.0

# Zero-padding added on each side before the Hann taper in `wall_rotation`, as a
# fraction of the cropped extent. Measured on synthetic rotated room outlines
# (mean / max rotation error over nine angles): 0.0 -> 0.68/1.24 deg,
# 0.25 -> 0.32/1.05, 0.4 -> 0.17/0.56, 0.6 -> 0.14/0.26. Costs (1+2f)^2 in
# transform area, so 0.5 buys most of the accuracy at 4x the pixels.
ROTATION_PAD_FRAC = 0.5


def _angular_energy(
    spectrum_lin: np.ndarray, params: SpectrumParams
) -> tuple[np.ndarray, np.ndarray]:
    """Sum spectral energy into angle bins over [0, 180).

    Each spectrum pixel at (u, v) relative to the centre contributes to the
    orientation atan2(v, u) (mod 180, since the magnitude spectrum is
    centro-symmetric). A wall oriented at angle a puts its energy on the line
    through the origin at that same angle, so peaks in this histogram are the
    map's dominant orientations.
    """
    h, w = spectrum_lin.shape
    cy, cx = h / 2.0, w / 2.0
    yy, xx = np.mgrid[0:h, 0:w]
    dy = yy - cy
    dx = xx - cx
    radius = np.hypot(dx, dy)
    rmax = min(cy, cx)

    # Drop the DC neighbourhood and the extreme high-frequency corners.
    keep = (radius > params.lowcut_frac * rmax) & (radius <= rmax)

    ang = np.degrees(np.arctan2(dy, dx)) % 180.0
    nbins = int(round(180.0 / params.angle_step_deg))
    bin_idx = np.clip((ang / 180.0 * nbins).astype(int), 0, nbins - 1)

    weights = np.where(keep, spectrum_lin, 0.0).ravel()
    energy = np.bincount(bin_idx.ravel(), weights=weights, minlength=nbins)
    # Normalise out the fact that low-angle-resolution bins near the axes hold
    # different pixel counts is negligible here; a light smoothing stabilises
    # peak-picking on small maps.
    energy = _smooth_circular(energy, k=max(1, int(round(2.0 / params.angle_step_deg))))
    angles = (np.arange(nbins) + 0.5) * params.angle_step_deg
    return angles, energy


def _smooth_circular(x: np.ndarray, k: int) -> np.ndarray:
    """Circular moving-average smoothing (angles wrap at 180 deg)."""
    if k <= 1:
        return x
    kernel = np.ones(2 * k + 1) / (2 * k + 1)
    padded = np.concatenate([x[-k:], x, x[:k]])
    return np.convolve(padded, kernel, mode="valid")


def _pick_directions(
    angles: np.ndarray, energy: np.ndarray, params: SpectrumParams
) -> list[float]:
    """Non-max-suppress the angular energy into a small set of orientations."""
    thresh = params.peak_rel_threshold * float(energy.max())
    order = np.argsort(-energy)
    chosen: list[float] = []
    for idx in order:
        if energy[idx] < thresh:
            break
        a = float(angles[idx])
        if all(_angdist(a, c) >= params.peak_nms_deg for c in chosen):
            chosen.append(a)
        if len(chosen) >= params.max_directions:
            break
    return sorted(chosen)


def _angdist(a: float, b: float) -> float:
    """Smallest distance between two orientations on the 180-deg circle."""
    d = abs(a - b) % 180.0
    return min(d, 180.0 - d)


def _angdist_arr(a: np.ndarray, b: float) -> np.ndarray:
    """Vectorised :func:`_angdist` for an array of orientations against one."""
    d = np.abs(a - b) % 180.0
    return np.minimum(d, 180.0 - d)


def _crop_to_content(wall: np.ndarray) -> np.ndarray:
    """Crop to the bounding box of the wall cells.

    An incidental map extent — how much never-observed padding the grid carries
    — must not change the score, so the transform runs over content only. The
    public entry point takes a wall mask alone (the two callers hold different
    occupancy conventions), so the wall bounding box is the available proxy for
    the decided region; it is contained in it, which makes this the stricter
    crop of the two.
    """
    rows = np.any(wall, axis=1)
    cols = np.any(wall, axis=0)
    if not rows.any():
        return wall
    r0, r1 = np.where(rows)[0][[0, -1]]
    c0, c1 = np.where(cols)[0][[0, -1]]
    return wall[r0 : r1 + 1, c0 : c1 + 1]


def _residual_spectrum(
    wall: np.ndarray, params: SpectrumParams, window: bool = False
) -> tuple:
    """Angular energy with its broadband floor removed, normalised to sum 1."""
    signal = wall.astype(np.float32)
    if window:
        # Pad before tapering. A Hann window over a *tight* crop cuts into the
        # walls themselves -- worst of all on a rotated shape, whose corners
        # reach the crop edge -- and measurably costs accuracy: on synthetic
        # rotated room outlines, crop+taper gives 0.68 deg mean rotation error
        # against 0.39 deg for no taper at all. Padding puts the taper's roll-off
        # on empty space where it belongs, giving 0.14 deg.
        pad = ROTATION_PAD_FRAC
        py, px = (int(round(s * pad)) for s in signal.shape)
        signal = np.pad(signal, ((py, py), (px, px)))
        h, w = signal.shape
        signal = signal * np.hanning(h)[:, None] * np.hanning(w)[None, :]
    mag = np.abs(np.fft.fftshift(np.fft.fft2(signal)))
    angles, energy = _angular_energy(mag, params)
    k = max(1, int(round(FLOOR_HALFWIDTH_DEG / params.angle_step_deg)))
    residual = np.clip(energy - _smooth_circular(energy, k), 0.0, None)
    total = float(residual.sum())
    q = residual / total if total > 0 else residual
    return angles, energy, q


def fold_90(angles: np.ndarray, values: np.ndarray) -> tuple:
    """Fold an orientation distribution from [0, 180) onto [0, 90).

    An orthogonal frame's two wall families sit 90 deg apart, so folding puts
    both on one peak: the quantity that survives is the frame's *rotation*,
    which is what an alignment step wants to measure and correct.
    """
    n = len(values) // 2
    return angles[:n], values[:n] + values[n : 2 * n]


def _parabolic_offset(y0: float, y1: float, y2: float) -> float:
    """Sub-bin peak offset in bins, from a parabola through three samples.

    Returns 0 for a flat or inverted triple rather than dividing by zero, and is
    clamped to the sampled interval so a near-degenerate fit cannot throw the
    peak into a neighbouring bin.
    """
    den = y0 - 2.0 * y1 + y2
    if den == 0 or not np.isfinite(den):
        return 0.0
    return float(np.clip(0.5 * (y0 - y2) / den, -0.5, 0.5))


def refine_peak(angles: np.ndarray, values: np.ndarray, index: int) -> float:
    """Sub-bin interpolated angle of the peak at ``index`` (circular)."""
    n = len(values)
    step = float(angles[1] - angles[0]) if n > 1 else 0.0
    offset = _parabolic_offset(
        float(values[(index - 1) % n]),
        float(values[index]),
        float(values[(index + 1) % n]),
    )
    return float(angles[index] + offset * step)


def wall_rotation(
    wall: np.ndarray,
    params: SpectrumParams | None = None,
    *,
    window: bool = True,
) -> dict:
    """Dominant wall rotation of a map, folded to [0, 90), with sub-bin precision.

    This is the canonical measurement an alignment step should use to answer
    "how far is this map's wall grid turned?" before re-solving. It exists here,
    rather than being hand-rolled at each call site, because the same fold was
    written twice during the 2026-08-02 session and got a different answer each
    time.

    ``window`` zero-pads by :data:`ROTATION_PAD_FRAC` and applies a 2-D Hann
    taper, which removes the rectangular aperture's axis-aligned sinc cross.
    Two things here were measured rather than assumed.

    **The aperture does not pin this fold to 0 deg**, with or without the taper:
    on a room outline rotated 31 deg the peak lands at 59.25 deg un-tapered and
    58.75 deg tapered (truth 59.0), and the energy within 2 deg of 0/90 is
    0.026-0.030 in every combination of taper and low-cut.
    ``_angular_energy`` already drops the DC neighbourhood and works on
    magnitude rather than power, which is most likely why a hand-rolled fold
    that skipped those steps behaved differently. This is pinned by a test.

    **The padding is not decoration.** Tapering a *tight* crop is worse than not
    tapering at all — the roll-off cuts into the walls, and a rotated shape's
    corners reach the crop edge — measured at 0.68 deg mean error against
    0.39 deg untapered. Padded, it is 0.14 deg.

    Returns ``angle_deg`` (sub-bin refined, in [0, 90)), ``bin_angle_deg`` (the
    unrefined bin centre, for debugging), ``energy_frac`` (share of the folded
    residual within ``wedge_halfwidth_deg`` of the peak — low means there is no
    single dominant grid to align to) and ``windowed``.

    Accuracy is limited by the map, not the bins: mean error over nine synthetic
    rotations is 0.14 deg, worst 0.26 deg, the residual being the raster
    staircase of a rotated line. Good enough to measure a wall grid's rotation
    and to drive a re-solve; **not** good enough to certify a 1-2 deg shear as
    absent.
    """
    params = params or SpectrumParams()
    wall = np.asarray(wall, dtype=bool)
    if wall.ndim != 2 or wall.size == 0 or not wall.any():
        return {"angle_deg": None, "note": "no walls"}

    cropped = _crop_to_content(wall)
    if min(cropped.shape) < 4:
        return {"angle_deg": None, "note": "wall extent too small"}

    angles, _energy, q = _residual_spectrum(cropped, params, window=window)
    if q.sum() <= 0:
        return {"angle_deg": None, "note": "no angular structure"}

    fa, fq = fold_90(angles, q)
    idx = int(np.argmax(fq))
    refined = refine_peak(fa, fq, idx) % 90.0

    dist = np.abs(fa - refined) % 90.0
    dist = np.minimum(dist, 90.0 - dist)
    total = float(fq.sum())
    return {
        "angle_deg": refined,
        "bin_angle_deg": float(fa[idx]),
        "energy_frac": float(fq[dist <= params.wedge_halfwidth_deg].sum() / total)
        if total > 0
        else 0.0,
        "windowed": bool(window),
    }


def _directions_table(
    angles: np.ndarray, q: np.ndarray, params: SpectrumParams, dirs: list[float]
) -> list[dict]:
    """Per-direction energy share and angular width about the family centre."""
    table = []
    for d in dirs:
        dist = _angdist_arr(angles, d)
        near = dist <= params.wedge_halfwidth_deg
        share = float(q[near].sum())
        width = (
            float(np.sqrt(np.sum(q[near] * dist[near] ** 2) / share))
            if share > 0
            else 0.0
        )
        table.append(
            {
                "angle_deg": float(d),
                "energy_frac": share,
                "width_deg": width,
            }
        )
    return table


def _frames_table(directions: list[dict], merge_deg: float) -> tuple[list[dict], float]:
    """Group directions into orthogonal frames on ``angle mod 90``.

    A rectilinear building contributes both of its wall directions to one
    frame; an extra frame means a second rectilinear system — which is what a
    drift-rotated *section* of the map looks like, as against an angled hallway,
    which adds a single direction to an existing frame.
    """
    if not directions:
        return [], 0.0

    ordered = sorted(directions, key=lambda d: -d["energy_frac"])
    frames: list[dict] = []
    for d in ordered:
        a90 = d["angle_deg"] % 90.0
        for f in frames:
            gap = abs(a90 - f["_a90"]) % 90.0
            if min(gap, 90.0 - gap) <= merge_deg:
                f["_members"].append(d)
                f["energy_frac"] += d["energy_frac"]
                break
        else:
            frames.append(
                {"_a90": a90, "_members": [d], "energy_frac": d["energy_frac"]}
            )

    frames.sort(key=lambda f: -f["energy_frac"])
    dominant = frames[0]["_a90"]
    out = []
    for f in frames:
        gap = abs(f["_a90"] - dominant) % 90.0
        out.append(
            {
                "angle_deg": float(f["_a90"]),
                "energy_frac": float(f["energy_frac"]),
                "n_directions": len(f["_members"]),
                "offset_from_dominant_deg": float(min(gap, 90.0 - gap)),
            }
        )
    return out, float(frames[0]["energy_frac"])


def _manhattan(angles: np.ndarray, q: np.ndarray, params: SpectrumParams) -> dict:
    """Fit one orthogonal frame to the whole spectrum.

    Descriptive only, and **not rotation-stable on a multi-frame map**: the
    fitted frame jumps between competing wall families, so the same unchanged
    map scores differently after a global rotation (measured: share 0.778 ->
    0.625 -> 0.692 over 0/+17/-31 deg). Kept because it is a compact summary of
    "how Manhattan is this map", not because it ranks anything.
    """
    z = complex(np.sum(q * np.exp(4j * np.radians(angles))))
    frame = float((np.degrees(np.angle(z)) / 4.0) % 90.0) if abs(z) > 0 else 0.0
    gap = np.abs(angles - frame) % 90.0
    dist = np.minimum(gap, 90.0 - gap)
    return {
        "manhattan_frame_deg": frame,
        "manhattan_concentration": float(abs(z)),
        "manhattan_share": float(q[dist <= params.wedge_halfwidth_deg].sum()),
    }


def _reference_fit(
    angles: np.ndarray,
    q: np.ndarray,
    params: SpectrumParams,
    reference_directions: list[float],
) -> dict:
    """Align a reference direction set to this map and measure what misses it.

    A map frame's absolute rotation is arbitrary, so the reference set is free
    to rotate as a whole: one global offset is searched over the bin grid,
    minimising the energy-weighted angular distance from the spectrum to the
    nearest reference direction.
    """
    refs = np.asarray([float(r) % 180.0 for r in reference_directions])
    if refs.size == 0:
        return {}

    # dist[i, j] = distance from bin j to reference i, before any offset.
    per_ref = np.stack([_angdist_arr(angles, r) for r in refs])

    best = None
    for shift in range(len(angles)):
        # Rotating the reference set by +shift bins is a circular shift of the
        # per-bin distances in the opposite direction.
        d = np.min(np.roll(per_ref, shift, axis=1), axis=0)
        cost = float(np.sum(q * d**2))
        if best is None or cost < best[0]:
            best = (cost, shift, d)

    cost, shift, dist = best
    offset = (shift * params.angle_step_deg + 90.0) % 180.0 - 90.0
    return {
        "reference_offset_deg": float(offset),
        "off_reference_energy_frac": float(q[dist > params.wedge_halfwidth_deg].sum()),
        "reference_dispersion_deg": float(np.sqrt(cost)),
    }


def angular_stats(
    wall: np.ndarray,
    params: SpectrumParams | None = None,
    reference_directions: list[float] | None = None,
    *,
    frame_merge_deg: float = FRAME_MERGE_DEG,
    peak_rel_threshold: float = STATS_PEAK_REL_THRESHOLD,
    max_directions: int = STATS_MAX_DIRECTIONS,
) -> dict:
    """Score the angular structure of a boolean wall mask.

    ``wall`` is a 2-D boolean array, True where the map holds a wall. It is a
    mask rather than a map image because the two callers hold different
    occupancy conventions and neither should be converted lossily: the declutter
    pass binarises a ROS occupancy PNG (0 occupied / 205 unknown / 254 free),
    while the bag-replay scorer holds an ``OccupancyGrid`` array (-1 unknown,
    0..100) and thresholds it directly.

    Returns a dict with, in order of what it is for:

    **Ranked scalars** — frame-model-free, rotation-invariant by construction
    (a global rotation is a circular shift of the spectrum, and none of these
    depend on where the shift starts):

    * ``angular_support_deg`` — ``exp(H) * angle_step_deg``, the effective
      number of degrees of wall direction the map uses. **Lower is better.**
    * ``angular_entropy_norm`` — ``H / ln(nbins)``, the same in [0, 1].
      **Lower is better.**
    * ``unassigned_energy_frac`` — energy further than ``wedge_halfwidth_deg``
      from *every* detected direction: genuine angular smear, sitting between
      families rather than on one. **Lower is better.** Note what it does *not*
      catch: a coherent rotated section forms its own family and so does not
      raise this at all. That is what ``reference_directions`` is for.

    **Structure, not score** — ``directions`` (each ``angle_deg``,
    ``energy_frac``, ``width_deg``), ``frames`` (each ``angle_deg``,
    ``energy_frac``, ``n_directions``, ``offset_from_dominant_deg``) and
    ``dominant_frame_share``. ``n_peaks`` is reported but is threshold-bound and
    is not a score.

    **Verdict, when a prior is given** — pass ``reference_directions`` (the
    site's known wall directions, in the same image convention; for a building
    with an angled hallway the set *must include the hallway direction*, or the
    metric re-acquires the single-frame bug this module exists to avoid) and get
    ``reference_offset_deg``, ``off_reference_energy_frac`` and
    ``reference_dispersion_deg``.

    ``manhattan_*`` is an unranked descriptor; see :func:`_manhattan` for why it
    must not be ranked.

    The frame merge tolerance is the same order as the section rotations this
    exists to catch — it must exceed the shear a real frame carries (7.5 deg
    measured) yet stay under the tear being resolved — so the frame table is a
    diagnostic to read, not a threshold to trip. Below roughly its own tolerance
    a rotated section merges back into the dominant frame and becomes invisible
    to it; ``reference_directions`` is what convicts at that scale.

    ``reference_offset_deg`` is unique only up to the reference set's own
    symmetry: a rectilinear set repeats every 90 deg, so an offset of 84.5 and
    one of -5.5 describe the same fit.
    """
    params = params or SpectrumParams()
    wall = np.asarray(wall, dtype=bool)

    if wall.ndim != 2 or wall.size == 0 or not wall.any():
        return {"n_peaks": 0, "note": "no walls"}

    cropped = _crop_to_content(wall)
    if min(cropped.shape) < 4:
        return {"n_peaks": 0, "note": "wall extent too small"}

    angles, _energy, q = _residual_spectrum(cropped, params)
    if q.sum() <= 0:
        return {"n_peaks": 0, "note": "no angular structure"}

    nz = q > 0
    entropy = float(-np.sum(q[nz] * np.log(q[nz])))

    pick = SpectrumParams(
        angle_step_deg=params.angle_step_deg,
        lowcut_frac=params.lowcut_frac,
        peak_rel_threshold=peak_rel_threshold,
        peak_nms_deg=params.peak_nms_deg,
        max_directions=max_directions,
        wedge_halfwidth_deg=params.wedge_halfwidth_deg,
    )
    dirs = _pick_directions(angles, q, pick)
    directions = _directions_table(angles, q, params, dirs)
    frames, dominant_share = _frames_table(directions, frame_merge_deg)

    nearest = np.min(np.stack([_angdist_arr(angles, d) for d in dirs]), axis=0)

    out = {
        "angular_support_deg": float(np.exp(entropy) * params.angle_step_deg),
        "angular_entropy_norm": float(entropy / np.log(len(q))),
        "unassigned_energy_frac": float(q[nearest > params.wedge_halfwidth_deg].sum()),
        "n_peaks": len(dirs),
        "directions": directions,
        "frames": frames,
        "n_frames": len(frames),
        # Frames carrying real energy - the tear signal. A rectilinear building
        # is 1; a drift-rotated section makes 2. The share floor keeps a stray
        # sliver of a family from reading as a second building.
        "n_strong_frames": sum(1 for f in frames if f["energy_frac"] >= 0.15),
        "dominant_frame_share": dominant_share,
    }
    out.update(_manhattan(angles, q, params))
    if reference_directions:
        out.update(_reference_fit(angles, q, params, reference_directions))
    return out
