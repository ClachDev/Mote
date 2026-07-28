# map_cleanup — FFT structure extraction for noisy maps

Post-mapping declutter for 2D occupancy grids. SLAM maps come out speckled:
salt-and-pepper occupied cells in free space, ragged wall edges, furniture and
other movable clutter frozen into the geometry. This module recovers the clean
structural skeleton (the walls) and drops the rest.

It implements the structure-identification method described in **ROSE / ROSE²**
("Robust Structure identification and rOom SEgmentation from occupancy grid
maps", [arXiv:2203.03519](https://arxiv.org/abs/2203.03519)). The technique is
purely geometric — **no training, no model, no network** — which makes it a good
fit for a small robot that just needs its saved maps tidied.

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

## Room segmentation

`room_segmentation.py` is the second stage: it takes the same occupancy grid and
carves its free space into **rooms**, each proposed to the task layer as a zone
with a polygon footprint, so a freshly mapped floor arrives with its rooms
already outlined instead of every one taught by driving to it.

```bash
pixi run segment-map [MAP.yaml] [--write] [--out DIR]
```

With no argument it segments the active site floor's current map and writes
`map_rooms.yaml` (the proposal) and `map_rooms.png` (an overlay to look at)
beside that floor's `zones.yaml`; `--write` merges the proposal in, where the
generated `room_NN` names are meant to be renamed to what the rooms are called.
Merging never overwrites — a candidate covering a zone that already has a
footprint is dropped as already-named — so it is additive after hand-teaching
and a no-op run twice.

It follows ROSE²'s idea (extend the walls into lines, let the lines partition the
map into faces, merge the faces back into rooms) with the FFT orientation scan
above doing the work that paper's Hough → DBSCAN line clustering does, so no
`scikit-learn` is needed:

1. rotate the map so the dominant wall direction is axis-aligned,
2. project vertically- and horizontally-extended wall pixels onto the two axes;
   the peaks are the wall lines,
3. cut the map along every wall line — *including where it runs through open
   space*, which is what separates a room from the corridor outside its door,
4. merge neighbouring faces whose shared boundary has a contiguous opening
   wider than a door,
5. keep the merged faces with enough observed free space; each is a room, posed
   at its clearance maximum.

The whole thing rests on one physical assumption — **a doorway is narrow** — so
it does not care how big or how oddly shaped a room is, which a distance-
transform threshold does. Two consequences worth knowing:

- **Corridor networks are not proposed.** A footprint is one outline and cannot
  express a hole, so a region that wraps around a block of rooms would claim
  everything it encircles; those are counted (`encircling`) and dropped. A
  corridor that is simply a stretch of floor (as in `office_world`) does come
  out as a candidate.
- **Manhattan after rotation.** One dominant wall direction and its perpendicular
  are handled, including a map frame rotated arbitrarily (the usual case — a map
  frame's axes are wherever SLAM started). A building with wings at 30° to each
  other will over-cut the off-axis wing.

Scored against ground-truth rooms on the sim ladder by
`mote_simulation/test/room_segmentation_eval.py` (`pixi run segment-eval`);
results and overlays in `docs/tuning/2026-07-27-room-segmentation/`.

## Status / next steps

- **Done:** FFT declutter core + CLI + diagnostics, validated on a real noisy
  mote map (see `scratchpad_results/map_cleanup/`).
- **Done:** wired into `save-map` as an automatic post-processing pass
  (`sites._promote_cleaned`): every saved revision keeps the untouched
  map_saver output as `map_raw.png` and promotes the decluttered image to the
  served `map.png`. The `map.yaml` frame is byte-identical, so zone coordinates
  and localization are unaffected; a cleaning failure falls back to serving the
  raw map rather than losing a freshly-mapped area.
- **Done:** the ROSE² **room-segmentation** layer, above — as a line/face/merge
  partition driven by the FFT orientation scan rather than Hough → DBSCAN, so
  the `scikit-learn` dependency it would have needed never arrived.
- **Not yet:** nothing proposes a *name*. The rooms come out as `room_NN` for a
  human to rename; recognising "this is a kitchen" is a perception problem, not
  a geometry one.

## Parameters

See `Params` in `structure_extraction.py`. The two that matter most:
`wedge_halfwidth_deg` (narrower = more aggressive declutter, risks thinning real
off-axis walls) and `peak_rel_threshold` (higher = fewer orientations kept).
Direction detection sharpens considerably on full-resolution maps — the current
validation input is a low-res screenshot, which inflates spectral side-lobes.

`RoomParams` in `room_segmentation.py` governs the segmentation. `door_max_m`
(1.4) is the width that still counts as a doorway rather than an opening, and
`min_wall_run_m` (1.5) the shortest unbroken wall allowed to define a room —
raise it on a big furnished building where beds and desks are wall-length (the
hospital world recovers one or two more rooms at 2.0 m), lower it in a small
flat with short partition walls. Both are `--door` / `--wall-run` on the CLI.
