# SfM / Multi-View Depth — Stage 0 Results

*Geometry feasibility measured offline from recorded `perception` bags (issue #21,
plan in [`sfm_multiview_depth.md`](sfm_multiview_depth.md) "Stage 0"). Harness:
[`mote_perception/tools/sfm_stage0_geometry.py`](../../mote_perception/tools/sfm_stage0_geometry.py)
— pure geometry over `/tf` (odom→base), `/image_raw/compressed`, and the static
camera mount; no depth server, no lidar. Run it on any perception bag:*

```bash
pixi run python mote_perception/tools/sfm_stage0_geometry.py <bag> --out DIR
```

## What was measured

The three quantities the whole triage hinges on, over the `~/.mote/bags/perception`
recordings — a mix of autonomous mapping runs and a long parked debug capture:

| bag | duration | path | translating time | frames w/ usable baseline |
|---|---|---|---|---|
| `20260706_172607` (drive) | 359 s | 19.0 m | 29 % | 26 % |
| `20260706_135320` (drive) | 215 s | 15.5 m | 41 % | 32 % |
| `20260630_145232` | 187 s | 3.0 m | 8 % | 11 % |
| `20260630_111443` | 214 s | ~1 m | 4 % | 6 % |
| `20260706_133149` (parked) | 1218 s | 0.7 m | 0 % | 0 % |

"Usable baseline" = a past frame exists ≥ 0.10 m away in camera translation with
inter-view rotation < 8° reached within a 3 s look-back (in-place turns swing the
camera on a short arc but are rejected by the rotation gate, isolating genuine
translation parallax).

### 1. Parallax across the image — **adequate when moving**

On the best driving bag (`135320`, 18 tracked keyframe pairs at B ≈ 0.10 m,
disparity normalised to B = 0.10 m):

- Disparity grows radially from the focus of expansion (FOE) exactly as forward-motion
  geometry predicts: **~4.2 px within 50 px of the FOE**, rising to 13–30 px further
  out. Even the weakest region — the far floor-band edge near the FOE, where upright
  obstacle marks matter most for planning distance — clears the ~0.5–1 px a matcher
  needs, and a 0.30 m keyframe baseline triples it.
- The camera's downward pitch keeps the useful 0.25–1.2 m floor band clear of the FOE,
  confirming the plan's hypothesis that Mote's geometry is more favourable than a
  generic forward-driving robot.
- The near floor-band edge (x = 0.25 m) projects *below* the 480-row frame, so the very
  near floor is not even in view; dense floor matching for the leveling-plane fit would
  be thin at the bottom regardless.

![Optical-flow arrows from a keyframe pair: tiny at the FOE (red cross), growing
radially; floor band marked in orange](sfm_stage0/parallax_example.webp)

![Per-region disparity heatmap: a low-parallax band through the FOE/horizon, growing
outward](sfm_stage0/parallax_heatmap.webp)

### 2. Pose-at-stamp accuracy — **not the bottleneck**

- v4l2 image stamp jitter is small: header→receipt jitter **2.7–3.8 ms (1σ)**, which at
  ~0.2 m/s is only **~1 mm of baseline error**.
- odom→base TF publishes at the ~10 Hz lidar rate (100 ms gaps). Leave-one-out
  interpolation self-consistency (a conservative ~2× -gap bound that also folds in ICP
  pose noise) is **~1 mm translation, ~0.4° rotation (p90)** on translating segments.
- Error budget: the pose/stamp contribution to depth error (`dz|pose-baseline` ≈
  10–25 mm over 0.5–1.2 m) is **comparable to, not dominant over**, the matcher-limited
  error (`dz|match` ≈ 4–21 mm). Stamp error does **not** dominate the triangulation
  budget — the plan's primary geometric kill criterion is *not* tripped.
- One caveat: the residual-rotation reprojection upper bound (~2.6–3.4 px) is
  comparable to the *weakest* (near-FOE) parallax. It only bites near the FOE and is
  managed by gating on low inter-view rotation and preferring larger baselines.

### 3. Usable-baseline duty cycle — **the decisive limit**

The robot spends most of every mission **stopped (42–99 %)** or **turning in place
(1–17 %, zero parallax)**. Even on the two genuine driving bags it translates only
**29–41 % of the time**, and only **26–32 % of image frames** can reach a usable
baseline. A metric multi-view path would therefore produce depth for at most ~1/3 of
frames and fall back to the single-image path for the rest.

## Verdict

The Stage 0 kill criterion **is** tripped — but by **duty cycle**, not by geometry or
stamp error. That distinction changes the recommendation versus a flat "kill":

- **Stage A — video-depth flicker swap: DO.** Motion-independent, works on the
  always-available single-image path, unaffected by the low duty cycle. The robust win.
- **Stage B — sparse triangulated anchors replacing the lidar band: OPTIONAL
  supplement.** Geometry supports it when moving, and it *holds anchors while stopped*
  (better than the lidar fit, which needs a constraining scan) — so the low duty cycle
  hurts it least. Worth a prototype, but not as the primary depth source.
- **Stage C — learned-MVS replacement: DEFER.** High integration cost to produce metric
  depth for < 1/3 of frames, still needing the mono path as fallback, is a poor trade
  until the robot's motion profile changes (or a capable discrete GPU appears).

Net: proceed with Stage A; treat Stage B as an opportunistic decoupling of scale from
the lidar rather than a replacement; Stage C stays parked.
