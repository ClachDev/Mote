# map_cleanup — FFT structure extraction for noisy maps

Post-mapping declutter for 2D occupancy grids. SLAM maps come out speckled:
salt-and-pepper occupied cells in free space, ragged wall edges, furniture and
other movable clutter frozen into the geometry. This module recovers the clean
structural skeleton (the walls) and drops the rest.

It is a clean-room reimplementation of the structure-identification stage of
**ROSE / ROSE²** ("Robust Structure identification and rOom SEgmentation from
occupancy grid maps", [arXiv:2203.03519](https://arxiv.org/abs/2203.03519)).
The original authors' code
([aislabunimi/ROSE2](https://github.com/aislabunimi/ROSE2)) is GPLv3 and ROS 1,
so rather than vendor it this is a from-scratch, ROS-2-native implementation of
the geometry. The technique is purely geometric — **no training, no model, no
network** — which is why it was worth reimplementing rather than reaching for a
learned floor-plan model.

## Idea

Straight walls in a map concentrate their Fourier energy along a few dominant
orientations; clutter and speckle smear energy across *all* orientations. So:

1. binarise the map into a wall image,
2. take its 2D FFT and measure spectral energy as a function of angle,
3. pick the dominant orientations (peaks of that angular energy),
4. keep only the frequency wedges aligned with those orientations — a
   directional band-pass — and invert the FFT to get a continuous structure
   score,
5. threshold that score back into a decluttered occupancy grid, gated to the
   neighbourhood of originally-observed walls so it declutters rather than
   hallucinates.

The building does **not** have to be Manhattan (axis-aligned): the dominant
orientations are whatever the map actually contains, including diagonal
corridors.

## Usage

```bash
pixi run clean-map path/to/map.png [--out DIR] [--wedge 5] [--peak-rel 0.45] [--gate 2]
```

Writes `<map>_cleaned.png` (a ROS occupancy PNG) and `<map>_diagnostics.png`
(input | Fourier spectrum with detected orientations | angular-energy plot |
cleaned map). Depends only on numpy + OpenCV — both already in the robot env.

The core is importable and side-effect free:

```python
from mote_bringup.map_cleanup import extract_structure, Params
res = extract_structure(occupancy_uint8, Params(wedge_halfwidth_deg=5))
res.cleaned_map      # uint8 ROS occupancy PNG
res.directions_deg   # detected wall orientations
```

## Status / next steps

- **Done:** FFT declutter core + CLI + diagnostics, validated on a real noisy
  mote map (see `scratchpad_results/map_cleanup/`).
- **Not yet:** wiring into `save-map` (offer a `--clean` that stores a cleaned
  revision alongside the raw one — never destructive, since map cleaning must
  not corrupt zone coordinates), and the ROSE² **room-segmentation** layer
  (Hough → DBSCAN line clustering → representative lines → face/room graph),
  which needs `scikit-learn` (DBSCAN) added to the env.

## Parameters

See `Params` in `structure_extraction.py`. The two that matter most:
`wedge_halfwidth_deg` (narrower = more aggressive declutter, risks thinning real
off-axis walls) and `peak_rel_threshold` (higher = fewer orientations kept).
Direction detection sharpens considerably on full-resolution maps — the current
validation input is a low-res screenshot, which inflates spectral side-lobes.
