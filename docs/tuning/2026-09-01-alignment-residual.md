# The alignment step's residual — what can be asserted, and with what

The mapping pipeline's build stage measures a solved map's wall rotation,
re-solves with that yaw injected, and — as the design was written — asserts the
residual is under 0.5° or fails the build. Three separate things stop that
assertion being made with `angular_stats.wall_rotation`, and only one of them is
the ~2° floor its docstring already records.

1. **The residual is not a single number.** On the map the design cites, thirds
   of the building disagree about where the wall grid is by 8°, and there is a
   second wall family 18° off the first. No rotation squares the map, so
   "the residual" is not a property it has to half a degree.
2. **The primitive does not find the grid.** `wall_rotation` reports every one
   of the seven banked 2026-08-02 solves as within 0.3° of square. Four of
   them are 3.5–5.6° off, which the eye settles in one look.
3. **The injected yaw is not what comes out.** The same bag, the same −3.0°
   injection, three values of `coarse_angle_resolution`: the solved map's
   orientation moved by +0.1°, −4.3° and −5.8°. A re-solve is not a rigid
   rotation, so an alignment step must verify after solving rather than assume.

Raw output, the probe that produced it, and the pictures are in
`2026-09-01-alignment-residual/`.

## 0. What was measured

The seven maps banked by the 2026-08-25 build-params run
(`2026-08-25-slam-build-params/`): the 2026-08-02 flat bag solved under three
`coarse_angle_resolution` values, each with and without `--frame 0 0 -3.0` —
the birth-alignment injection that produced the shipped map. No bag, no
harness and no GPU are needed to re-check any of this; the maps are in the
repository.

Each map is measured four ways:

| | |
| --- | --- |
| `angular_stats.wall_rotation` | the primitive the design's alignment step names — FFT, windowed, folded 0/90, sub-bin refined |
| `room_segmentation.dominant_rotation_deg` | the tree's other FFT estimator, square-padded, 0.5° bins, no refinement |
| projection sweep | rotate the wall mask over ±45° at 0.25°, score `Σrow² + Σcol²` normalised by wall-pixel count², take the parabolically refined peak |
| projection sweep, per tile | the same, on each ninth of the map |

The projection sweep is not FFT-based and is not a proposal by pedigree: §4 is
its validation, and §2 is the check on what it can be asked.

## 1. There is no orientation lattice

The design said, twice, that the live `coarse_angle_resolution` "snaps
solutions to a ~2° orientation lattice", which is why alignment needed a
build-only 1° one. Karto has no such lattice: `ScanMatcher::MatchScan` follows
the coarse sweep with a fine pass of half-range
`0.5 × coarse_angle_resolution` in steps of `fine_search_angle_offset`,
covering the coarse cell exactly. The reachable angles are a contiguous 0.2°
grid at any coarse value.

Measured and rejected in `2026-08-25-slam-build-params.md` §2, which is where
the code excerpt and the drift table live. Nothing here revisits it; the design
text is corrected to match.

## 2. The residual is not a single number

Projection sweep on each ninth of the map, degrees off axis:

```
unaligned/car-0.0349: whole map +3.46
   -17.15   +2.18   -1.56
   -15.89   +0.37   +4.02
   -15.61   -3.76   +4.48

birth-aligned/car-0.0175: whole map +0.74
   -19.82   -0.24   -3.67
   -18.66   +0.27   +0.42
   -12.86   -6.74   +1.23
```

The left column is the flat's angled wing — a real second wall family, which the
2026-08-25 report also names as an orthogonal frame 18.5° off the dominant one
carrying 0.16 of the energy. Discard it and the remaining six tiles still spread
8.2° and 8.0°. Whether that spread is architecture or residual SLAM drift cannot
be told from one map — `angular_stats`' own module docstring makes exactly that
point — and it does not matter here: either way, a 0.5° assertion is being made
about a quantity the artifact does not possess to better than several degrees.

This is the finding that decides the design. It is independent of every
estimator: it is the disagreement *between parts of the map*, not between ways
of measuring one.

## 3. The primitive does not find the grid

All seven maps, degrees off axis:

| condition | leg | `wall_rotation` | `energy_frac` | `dominant_rotation_deg` | projection |
| --- | --- | --- | --- | --- | --- |
| unaligned | car-0.0349 | −0.276 | 0.684 | −0.250 | **3.465** |
| unaligned | car-0.0175 | −0.018 | 0.656 | 0.250 | **4.994** |
| unaligned | car-0.01745 | 0.026 | 0.643 | 0.250 | **5.576** |
| birth-aligned | car-0.0349 | −0.092 | 0.680 | 0.250 | **3.568** |
| birth-aligned | car-0.0175 | −0.107 | 0.703 | −0.250 | 0.736 |
| birth-aligned | car-0.01745 | 0.025 | 0.705 | 0.250 | −0.185 |
| birth-aligned | car-0.01745-repeat | 0.025 | 0.705 | 0.250 | −0.185 |

Both FFT estimators call every map square. The sweep calls four of them 3.5° to
5.6° off. The picture is the arbiter: `unaligned-squared.png` is
`unaligned/car-0.0349/map.png` turned by the sweep's −3.465°, and the corridor
that ran visibly downhill across the as-solved map is level. `energy_frac`
around 0.68 says the primitive believes it found a dominant grid; there is no
low-confidence signal to gate on.

The shipped revision's `meta.yaml` records "walls -0.27° off axis", which is the
same primitive answering about the same build — the `birth-aligned/car-0.0175`
leg reproduces it, matching its recorded `loop_drift_m` to the digit. The sweep
puts that map at +0.74°. The recorded number sits in the band the primitive
cannot see, so it was never evidence that the alignment worked; the alignment
did work on that leg, and the reason we know is the sweep, not the meta.

Two things worth handing to whoever builds the replacement. The module's *other*
exported view of the same map — the wall-direction table in the 2026-08-25
report, un-windowed — puts the dominant family at 2.75°, within a degree of the
sweep and 3° from `wall_rotation`'s own answer for that map, so the windowing is
implicated rather than the spectrum. And both FFT estimators were validated
against synthetic single-family room outlines (`test_angular_stats.py`'s
`_room_outline`); a real occupancy grid with thick walls, furniture, speckle and
two wall families is a different object.

## 4. …and cannot track a known rotation below 2°

Rotate a real map by a known angle and ask each estimator how far it moved.
Base map `birth-aligned/car-0.0349`; errors in degrees:

| applied | `wall_rotation` moved | err | `dominant_rotation_deg` moved | err | projection moved | err |
| --- | --- | --- | --- | --- | --- | --- |
| 0.25 | 0.107 | −0.143 | 0.500 | 0.250 | 0.276 | 0.026 |
| 0.50 | 0.424 | −0.076 | 0.500 | 0.000 | 0.529 | 0.029 |
| 0.75 | 0.411 | −0.339 | 0.500 | −0.250 | 0.785 | 0.035 |
| 1.00 | 0.700 | −0.300 | 0.500 | −0.500 | 1.011 | 0.011 |
| 1.50 | 1.444 | −0.056 | 2.500 | 1.000 | 1.525 | 0.025 |
| 2.00 | 2.746 | 0.746 | 3.000 | 1.000 | 2.008 | 0.008 |
| 3.00 | 3.168 | 0.168 | 3.500 | 0.500 | 3.023 | 0.023 |
| 5.00 | 5.183 | 0.183 | 6.000 | 1.000 | 5.029 | 0.029 |
| 10.00 | 10.215 | 0.215 | 11.000 | 1.000 | 10.021 | 0.021 |

This reproduces the docstring's floor on real data rather than a synthetic
outline: from 0.25° to 1.5° `wall_rotation` reports 43% to 96% of the rotation
it was given, then 137% at 2°. Its worst error over the range is 0.75° — one and
a half times the residual the design wanted to assert. The projection sweep's
worst error over the same range is 0.035°.

So a sub-degree residual *is* measurable differentially, on the map's own
pixels, with no new data source: 0.035° is fourteen times inside the 0.5° the
design asked for. What is not measurable is an absolute one, for §2's
reason.

## 5. The injected yaw is not what comes out

Projection sweep on each pair, against the −3.0° actually injected:

| leg | unaligned | birth-aligned | delta | vs injected |
| --- | --- | --- | --- | --- |
| car-0.0349 | 3.465 | 3.568 | **+0.103** | +3.103 |
| car-0.0175 | 4.994 | 0.736 | −4.259 | −1.259 |
| car-0.01745 | 5.576 | −0.185 | −5.761 | −2.761 |

A rigid pre-multiply on the odometry prior would rotate the whole solve by the
injected amount. It does not, because slam_toolbox's correlation grid is a pixel
grid: starting the map frame 3° elsewhere changes which scan matches are found,
so the injection buys a *different solve*, not the same one turned. Two of the
three legs came out squarer than they went in; the third — at the live coarse
value — came out no squarer at all, and `birth-aligned-squared.png` against its
as-solved map shows that leg is still 3.6° off.

That leg is the one that matters for the design. The alignment step ran, the
map did not move, and the primitive the step was to be gated on reported
−0.092°. The gate would have passed it.

Part of each delta is the sweep's whole-map peak being pulled about by how a
solve redistributes structure between wall families (§2), so these numbers are
not a solid-body claim about the frame. The direction of the finding survives
that: whatever a re-solve does with an injected yaw, it is not "the same map,
turned".

## What this leaves

The alignment step keeps its purpose — a squared map is what `room_segmentation`
assumes ("Manhattan after rotation") and what an operator reads — and gives up
the absolute assertion. What can be built:

- **Measure with a differential estimator.** The projection sweep, or anything
  that clears §4's table. `wall_rotation` cannot carry this; its docstring and
  the bag-replay README both currently point an alignment step at it.
- **Gate on improvement, not absence.** After the re-solve, the dominant
  orientation must be at least as close to axis as before it, by more than the
  estimator's own resolution. That gate fails the `car-0.0349` leg in §5, which
  is the failure the design needs caught, and it makes no claim §2 forbids.
- **Report, do not gate, the rest.** The before/after orientations and the tile
  spread ride on the candidate as review evidence. A building whose thirds
  disagree by 8° has no squared version, and the operator is the right place for
  that to land.

`docs/design/mapping-pipeline.md` is corrected to this, and the estimator itself
is deferred to its own work item — it is a primitive with a validation table,
not a line in an orchestrator.

## Limits of this measurement

- **One bag, one building**, and one with an angled wing, which is what makes
  §2 as stark as it is. A rectilinear warehouse would spread less. It would not
  make §3, §4 or §5 come out differently: those are properties of the
  estimators and of the solver, measured on maps of any shape.
- **The sweep is validated differentially, not absolutely.** §4 shows it tracks
  a rotation it is given; its absolute peak is confirmed only by eye
  (`unaligned-squared.png`) and is soft to the degree the map's own families
  disagree. That is the right shape for a residual check and the wrong shape
  for a claim that a map *is* square.
- **The applied rotations in §4 are themselves resampled**, so every estimator
  in that table is being read on a re-rasterised map. That is the same
  rasterisation a freshly solved map presents, so it does not favour one
  estimator over another.
- **§5's deltas mix the frame's rotation with the solve's redistribution** and
  cannot separate them without instrumenting the pose graph.
