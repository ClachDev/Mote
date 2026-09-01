# Build params for an offline map solve — evidence

`mote_bringup/config/slam_toolbox_build_params.yaml` is the parameter set the
mapping pipeline's build stage solves a recorded bag under, as against the live
file the robot captures with. This is the measurement of which keys earned the
right to differ.

The answer is one key, and not the one the design expected.

Bag: `20260802_142539`, the 2026-08-02 flat mapping session — 542 fed scans,
340 pose-graph nodes, and the only bag on hand whose robot physically returned
to its start, so the only one where loop drift means anything. Harness:
`pixi run bag-replay --lockstep` (21 s per leg). Raw output in
`2026-08-25-slam-build-params/`.

## 0. Lockstep replay is deterministic

Two legs of one parameter set, launched separately under different
`ROS_DOMAIN_ID`s, returned identical metrics to every digit printed — drift
0.108 m, path 139.77 m, speckle 0.0195, the same wall-direction table. The
harness README warns that "SLAM's solver is not bit-exact run-to-run; treat
small deltas as noise", which is the right caution for a paced replay; under
lockstep, on this bag, it did not bite.

That matters for everything below: a 0.01 m difference between two parameter
sets is a real difference, not spread. It also retires a reading of the
2026-08-02 session's six same-named legs (drift 0.088–0.107 m) as a noise band.
They differed by the yaw each had injected, not by chance.

## 1. `loop_match_minimum_chain_size` — the one real divergence

Live is 15, and 15 cannot be satisfied at `loop_search_maximum_distance: 2.0`.
A straight pass through a 2.0 m-radius disc gives a 4.0 m chord;
`minimum_travel_distance: 0.3` puts nodes 0.3 m apart along it; so the longest
chain a pass can contribute is about 13. Closure candidates never form, at any
drift.

| chain | loop drift | map |
| --- | --- | --- |
| 15 (live) | 1.365 m | torn — duplicated hallway, three living-room copies |
| 10 | 0.087 m | coherent |

Measured 2026-08-02 at commit `dabffca`
(`bag_replay_results/20260802T142558Z`) and reproduced here under the committed
build file: 0.087 m.

This is a divergence only until task 335 lands, which moves the live file to 10
for the same reason. `test_slam_build_params.py` fails when it does, naming the
key and saying to delete the note — the live config is best-known-good by
policy, so a build-only correction to it is a temporary state, not a design.

## 2. `coarse_angle_resolution` — rejected

The 2026-08-02 map shipped at 0.0175 (1.0°) against the live 0.0349 (2.0°), on
the reasoning that the coarse sweep snaps a solution to a 2° orientation
lattice and so puts wall alignment out of reach. **There is no such lattice.**

Karto's `ScanMatcher::MatchScan` (slam_toolbox 2.8.5,
`lib/karto_sdk/src/Mapper.cpp`) runs the coarse sweep, then a fine pass:

```cpp
bestResponse = CorrelateScan(pScan, rMean, fineSearchOffset, fineSearchResolution,
    0.5 * m_pMapper->m_pCoarseAngleResolution->GetValue(),   // searchAngleOffset
    m_pMapper->m_pFineSearchAngleOffset->GetValue(),         // searchAngleResolution
    doPenalize, rMean, rCovariance, true);
```

The fine pass searches ±half the coarse step in increments of
`fine_search_angle_offset` — that is, it covers the coarse cell exactly. At the
live values it samples 11 angles across ±1.0° at 0.2° spacing. The reachable
set is a contiguous 0.2° grid whatever the coarse value, and 0.2° —
`fine_search_angle_offset` — is the quantum either way. Halving the coarse step
does not refine the answer; it only moves where the fine pass starts.

Measured, three values × two alignment conditions, everything else held at the
committed build file:

| `coarse_angle_resolution` | drift, birth-aligned | drift, unaligned | wall thickness |
| --- | --- | --- | --- |
| 0.0349 (live, 2.0°) | **0.099 m** | **0.087 m** | **0.064 m** |
| 0.0175 (1.0°) | 0.098 m | 0.097 m | 0.065 m |
| 0.01745 (1.0°, integral) | 0.108 m | 0.109 m | 0.065 m |

Birth-aligned legs ran `--frame 0 0 -3.0`, reproducing the shipped build; the
0.0175 leg returns 0.098 m against the shipped revision's recorded
`loop_drift_m: 0.098`, so the pipeline reproduces.

The live value wins or ties on drift in both conditions and on wall thickness
in both. Speckle prefers 0.0349 aligned (0.0136 vs 0.0159) and 0.01745
unaligned (0.0172 vs 0.0192) — it disagrees with itself and decides nothing.
`wall frames` reads 1 for 0.01745 aligned only because that map's second frame
carries 0.143 of the energy against the report's 0.15 threshold; the frame is
still there in the per-map table. No candidate beat the live value, so
`coarse_angle_resolution` does not diverge.

One aside, since it looks like a reason to prefer 0.01745 and is not:
0.01745 is exactly 5 × `fine_search_angle_offset`, which keeps the fine
sweep's sample count integral and satisfies the assert at the end of
`ComputeAngularCovariance` (`Round(2·offset/resolution)` must land the last
sample on `centre + offset`). 0.0175 does not — the ratio is 5.014 — so that
assert would trip in a debug build. Release builds compile it out, which is why
the shipped map solved without complaint. It is a real latent trap, and moot:
0.01745 scored worst of the three.

## 3. `loop_search_maximum_distance` — rejected

Widening the loop search does not beat radius 2.0 paired with chain 10.
Measured 2026-08-02 at `dabffca`, same bag, unaligned:

| radius | chain | other | loop drift |
| --- | --- | --- | --- |
| 2.0 | 10 | — | **0.087 m** |
| 3.5 | 10 | — | 0.094 m |
| 5.0 | 15 | — | 0.531 m |
| 8.0 | 12 | `loop_search_space_dimension: 12.0` | 1.561 m |

## What this leaves

The build file differs from the live file in one key, and only until the live
file catches up. That is the correct outcome of the pipeline's own rule — the
live config is never deliberately hobbled, so there is little left for a build
to improve by parameter alone. What the build actually buys is *steps* the
robot cannot run: a second solve with a measured yaw injected, prominence-based
declutter, room segmentation, and a human looking at the result before it is
promoted.

The file still earns its place as the seam those steps point at, and as the
record above — so the next person to wonder whether a finer angular sweep helps
can read that it was tried, and why it cannot.

## Limits of this measurement

- **One bag, one building.** A flat with four wall families. The conclusion
  drawn is the conservative one (do not diverge), which is the safe direction to
  be wrong in, but a warehouse or a long corridor could move it.
- **Loop drift needs a closed trajectory**, which most mapping sessions do not
  have. See the harness's own limitations note, reproduced in each report.
- **Angular residual below ~2° is not measurable** with `wall_rotation` — its
  docstring records the floor and the under-reporting below it. Whether a
  finer sweep helps the *alignment* step specifically therefore remains
  unmeasured; it is not evidence for a divergence, it is an absence of it.
