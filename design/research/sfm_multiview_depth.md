# SfM / Multi-View Depth Research

*Researched July 2026 (issue #21). Context: the L1 pipeline (#18) estimates depth from a single image per frame with relative Depth Anything V2-Small, paying two structural costs — a per-frame affine-in-disparity refit against lidar (`lidar_rescale.py`) because monocular depth has no stable metric scale, and frame-to-frame flicker because each frame is estimated from scratch. The robot's odometry (wheel + kinematic-ICP) gives a metric baseline between any two camera poses, so multi-view geometry could in principle deliver metric, temporally consistent depth directly.*

---

## Problem Statement

Two independent defects to attack, from the issue:

1. **No metric scale.** The monocular model's per-frame affine ambiguity forces the Theil-Sen lidar refit every frame; when the scan can't constrain the fit (flat wall, no depth spread) the pipeline holds the last good correction and hopes.
2. **No temporal consistency.** Depth flickers on a static scene; the plane-fit hold, inlier gates, and Theil-Sen (chosen specifically because count-RANSAC flickered) are all downstream compensation.

These are separable: a method can fix flicker without fixing scale (video-depth models) or scale without flicker (per-pair triangulation). The candidates below are scored on both.

---

## Constraint Zero: Inference Is CPU-Only

This reshapes the whole answer, so it goes first. The off-board `depth_server.py` runs conda-forge `pytorch` in the `depth` pixi environment — CPU inference by default. The workstation GPU is an AMD Phoenix3 iGPU (Ryzen 7040 class, gfx1103). Depth Anything V2-Small (~25 M params) already costs ~0.5 s/frame on the CPU here, and the issue's latency budget is ~2x the current ~0.6 s capture→publish path.

Consequences, before any accuracy argument:

- **Large pointmap transformers are out by an order of magnitude.** MASt3R (~0.7 B params) and VGGT (~1 B) are benchmarked on high-end CUDA GPUs; MASt3R-SLAM reports ~15 FPS *on an RTX 4090*. On this CPU that is tens of seconds per pair, not ~1 s.
- **ViT-based MVS (MVSAnywhere) is likely over budget** at full resolution; would need aggressive downscaling to even measure.
- **What fits:** classical stereo matching (ARCore's depth-from-motion ran on *one phone CPU core*), lightweight cost-volume MVS (DeepVideoMVS is MobileNet-based and was built for exactly this class of budget), sparse feature tracking + triangulation (milliseconds), and small streaming video-depth models (oVDA runs 20 FPS on a Jetson edge device; a CPU should manage the pipeline's ~2 Hz).

If a capable GPU ever joins the fleet, the triage below changes materially — noted per candidate.

> **Update (task 152, 2026-07-23): a ROCm path now runs the server on the Phoenix3 iGPU, but it does not lift this constraint.** `pixi run depth-rocm` serves inference on the iGPU via ROCm (torch masquerading gfx1103 as gfx1100). Measured: for V2-Small the iGPU only *ties* the CPU (~350 ms — the small ViT is memory-bandwidth-bound, and the iGPU shares LPDDR5 + power with the CPU), and for *larger* models it is *slower* than the CPU, because gfx1103's fast flash/mem-efficient attention kernels are broken and fall back to the slow math backend (V2-Base ~1.2 s GPU vs ~0.9 s CPU; fp16 crashes with an invalid-ISA GPU fault). That path was landed for robustness under CPU contention — it stays flat while the CPU-only server balloons to ~1–2 s under load — **not** for raw throughput. So the heavy-model verdicts below stand: the "if a GPU joins" trigger means a *capable discrete GPU* (the RTX-4090-class hardware MASt3R/VGGT are benchmarked on, of which this iGPU is ~1%), not the ROCm iGPU path.

---

## Geometry Feasibility (before any model choice)

Numbers use the calibrated intrinsics (fx ≈ 470 px over 640), the mount's usable floor band (0.25–1.2 m), and the measured drive envelope (~0.2 m/s per-wheel wall; turns are in-place).

**Triangulation error.** δz ≈ z²·δd / (f·B) for depth z, matching error δd (px), baseline B. At the ~2 Hz pipeline rate and 0.2 m/s, consecutive frames give B ≈ 0.1 m:

| z | B | δd | δz |
|---|---|----|-----|
| 1.2 m (band edge) | 0.10 m | 0.5 px | ~15 mm |
| 1.0 m | 0.30 m (keyframe 1.5 s back) | 1.0 px | ~7 mm |
| 0.5 m | 0.10 m | 0.5 px | ~3 mm |

Comfortably inside what the 0.02 m `z_obstacle` gate needs — *on paper*. ARCore-style keyframe selection (pick a past frame by odometry baseline, not the previous frame) buys baseline at the cost of staleness; kinematic-ICP's ~0.4 % RPE over a 0.3 m window contributes ~1 mm, negligible.

**Forward-motion degeneracy.** Parallax vanishes at the epipole (focus of expansion), and a forward-driving robot has the FOE in view — the classic reason motion stereo is hard on ground robots. Mote's geometry is *more favorable than generic*: the camera is pitched down at a 0.25–1.2 m floor band, so the FOE (horizon direction) sits near/above the top of the image while the useful band is at the bottom, far from it. But an upright obstacle at the far band edge is closest to the FOE, i.e. accuracy is worst exactly where marks matter for planning distance. Needs measurement on a real bag, not assertion.

**Zero-baseline cases.** In-place turns (how this robot actually turns — curved drive+turn is infeasible at the velocity wall) and standing still give *no* parallax: pure rotation carries no depth signal. Any multi-view design must gate on translation and fall back — the single-image path stays as the stationary/turning fallback, exactly as the issue anticipated.

**Rolling shutter & timing.** At 0.2 m/s translation, a ~20 ms readout skews geometry by ~4 mm — tolerable. At ~1 rad/s in-place turn, rows smear ~1° top-to-bottom — but turns are zero-baseline and gated out anyway, so RS mostly rides along with the gate. The sharper risk is **pose-at-stamp accuracy**: the baseline comes from interpolating TF at the image stamp, and v4l2 stamps are driver-receipt times with USB jitter; 10 ms of stamp error at 0.2 m/s is 2 mm of baseline error (fine), but during any residual rotation it's the dominant error. Quantifiable offline from an existing bag.

---

## Candidate Landscape

### 0. Better single-image *metric* model (UniDepth V2, Metric3D v2, Depth Pro)

Not in the issue but the obvious control arm: if a zero-shot metric model were accurate enough, no multi-view machinery needed. Already partially tested — `depth_server.py --metric` exists, and the relative model measured *both more accurate and faster* than the metric variant tried (that's why relative+refit is the default). Zero-shot metric models are typically 5–15 % off on out-of-domain cameras; the lidar anchor is better than that today. Depth Pro is ~0.3 s on GPU (CPU: way over budget); UniDepth V2 with the small backbone might fit the budget but doesn't beat the anchor and does nothing for flicker.

**Verdict: rejected** — the existing evaluation already covered this direction; it loses to the lidar anchor on its own terms.

### 1. Odometry-scaled two-view stereo (classical)

The exact precedent is Google's ARCore Depth API — *Depth from Motion for Smartphone AR* (Valentin et al., SIGGRAPH Asia 2018): keyframe selection by VIO baseline, polar rectification (handles in-view epipoles), CPU stereo matching, bilateral-solver densification — all on a single smartphone CPU core. Metric by construction; no learned model in the loop at all (the depth server could become torch-free).

Weaknesses for Mote specifically:

- **Textureless floor kills matching.** ARCore filled holes with planar priors/smoothing. For obstacle *marking*, no floor matches is almost acceptable (no matches → no points → no marks), but `ground_projection.py` needs dense floor points to fit the leveling plane — classical stereo would break the leveling stage, forcing a hybrid (keep DA for the floor fit) and the complexity grows back.
- Near-FOE degradation at the far band edge (see geometry section).
- Sensitive to RS/timing since there's no learned robustness to absorb it.

**Verdict: viable, cheap to prototype** (OpenCV + existing `bag_utils.py`), and the strongest latency story. Prototype offline before committing; expect to pair it with a mono model for leveling.

### 2. Lightweight learned MVS with metric poses (DeepVideoMVS, SimpleRecon, MVSAnywhere)

Pose-conditioned cost-volume models: feed 2+ frames *with their relative poses*, get depth **in the scale of the supplied poses** — odometry is metric, so the output is metric by construction. No lidar refit, no floor fallback. Learned matching degrades far more gracefully on low texture than classical stereo, and the newer models (SimpleRecon's metadata MLP, MVSAnywhere's mono/multi cue combiner) explicitly blend monocular cues where matching fails — which also softens the zero-baseline cliff.

- **DeepVideoMVS** (CVPR 2021): MobileNet encoder + pairwise cost volumes + ConvLSTM temporal fusion. Built for real-time low-memory inference; the *only* learned-MVS candidate plausibly inside the CPU budget. Temporal fusion also directly attacks flicker.
- **SimpleRecon** (ECCV 2022, Niantic): ~70 ms/frame on an A100; designed around noisy phone-VIO poses (encouraging for odometry-grade poses). CPU: likely 1–2 s at reduced resolution — borderline, measure first.
- **MVSAnywhere** (CVPR 2025, Niantic): zero-shot robust, "depth in input pose scale", adaptive cost volume; ViT-based, almost certainly over CPU budget. The quality ceiling if GPU appears.

Integration cost is modest and contained: the wire protocol (`depth_wire.py`, single source of truth) grows a pose field per frame; the server keeps a keyframe buffer and selects by baseline; the node and everything downstream (back-projection, leveling, gating, Nav2 layer) is unchanged. The lidar rescale can be retained initially as a *validator* (alarm on disagreement) rather than a corrector.

**Verdict: the strongest replacement candidate.** DeepVideoMVS first (only one likely inside CPU budget); SimpleRecon if a GPU shows up; MVSAnywhere as the eventual quality ceiling.

### 3. Hybrid: keep Depth Anything, swap the scale anchor to visual triangulation

Track sparse features (LK/ORB — milliseconds on CPU) between odometry-selected keyframes, triangulate with the metric baseline, and feed those sparse metric anchors to the *same* Theil-Sen affine-in-disparity fit — in place of (or alongside) the lidar returns. Smallest possible diff: `lidar_rescale.fit_affine_disparity_theilsen` doesn't care where its (pred, true) pairs come from.

- **Pros:** decouples scale from the lidar entirely (issue's candidate 3); anchors spread over the full image instead of the thin single-height lidar band — better-conditioned fit, and it survives the flat-wall case that defeats the lidar band today (a wall ahead still has corners/texture off-band). VI-SLAM literature does exactly this (sparse VIO landmarks + affine alignment), reporting that ~150 sparse points suffice.
- **Cons:** does not fix flicker by itself (still a per-frame refit, though anchors can be accumulated across frames and held); textureless scenes yield few tracks (hold, as today); stationary/turning yields no *new* anchors (hold — same behavior as today, and old anchors stay valid while stationary, which is actually better than the lidar fit's behavior). The literature also warns the true correction isn't globally affine — but that critique applies equally to today's lidar-band fit, which works.

**Verdict: best effort-to-payoff ratio.** A one-module experiment that removes the lidar dependency without touching the model, server, or protocol.

### 4. Pointmap transformers (DUSt3R / MASt3R / VGGT; streaming: MASt3R-SLAM, StreamVGGT, CUT3R, MASt3R-Fusion)

State of the art in feed-forward multi-view geometry, and the field is moving fast (streaming/constant-cost variants appearing through 2026). But: outputs are still **up-to-scale** (an odometry anchor is needed once per window — fine), and runtime lives on big CUDA GPUs (MASt3R-SLAM ~15 FPS on a 4090; VGGT wants serious VRAM). On a CPU-only server this is 10–100x over budget.

**Verdict: not now.** Re-triage if the depth server ever gets a CUDA GPU; then MASt3R-SLAM/CUT3R-class streaming models become the most interesting option on the list.

### 5. Full incremental SfM / visual SLAM (COLMAP-class)

Minutes-scale offline optimisation, and it solves localisation — which lidar + slam_toolbox/AMCL already solve better indoors. Nothing embedded-friendly in this family fits the budget or the need.

**Verdict: rejected**, as the issue suspected.

### 6. Streaming video-depth (Video Depth Anything Small, oVDA, FlashDepth) — flicker only

Not multi-view and not metric, but attacks defect 2 at near-zero architectural cost: VDA-Small is Depth Anything V2-Small plus a temporal module (same family the server already runs); its online variant (oVDA) does 42 FPS on an A100 and 20 FPS on a Jetson edge device — a desktop Ryzen at the pipeline's ~2 Hz is plausible, measure it. A temporally stable relative depth also stabilises the per-frame lidar fit (the fit input stops jittering), damping both flicker sources. Composable with candidates 2/3, and a model swap inside `depth_server.py` touches nothing else.

**Verdict: cheapest real win available;** orthogonal to the metric question.

---

## Recommendation — staged, cheapest-informative-experiment first

**Stage 0 — geometry feasibility harness (do first, ~a bag and a script).** Everything above hinges on measurable quantities no paper can supply: actual parallax distribution across the image on a representative driving bag, pose-at-stamp accuracy (TF interpolation vs v4l2 stamps), and fraction of mission time with usable baseline (excluding stops + in-place turns). The `perception` record stream already captures everything needed (`/tf`, `/tf_static`, `/image_raw/compressed`, `/camera_info`, `/scan_filtered` as truth) — no new recording infra, and `tools/bag_utils.py` transfers. Kill criteria: if usable-baseline time is low or stamp error dominates the error budget, candidates 1–3 all die here and only stage A survives.

**Stage A — flicker (candidate 6):** swap the server model to VDA-Small/oVDA, keep the lidar rescale, score frame-to-frame flicker on a static-scene bag with `depth_bag_eval.py`. Low risk, independent of stage 0's outcome.

**Stage B — metric decoupling (candidate 3):** sparse triangulated anchors replacing the lidar band in the existing Theil-Sen fit. Smallest diff that answers the issue's core question — does odometry-scaled multi-view geometry beat the lidar anchor on the eval bags?

**Stage C — replacement (candidate 2):** only if stage B's triangulation quality is good but the affine-fit formulation is the remaining bottleneck: DeepVideoMVS-style pose-conditioned server (wire protocol pose extension), CPU latency measured before any integration work; abort fast if > ~1.2 s/frame.

Against the issue's success criteria:

| Criterion | Stage A | Stage B | Stage C |
|---|---|---|---|
| Metric accuracy ≥ lidar-rescaled DA | — (unchanged) | direct test | direct test |
| Less flicker on static bag | direct test | — | via temporal fusion |
| Stationary behaviour | unchanged | holds anchors (≥ today) | mono-cue fallback |
| Latency ≤ ~2x current | measure model swap | +ms (tracking) | measure first, abort-fast |

---

## Sources

- [Issue #21](https://github.com/ClachDev/Mote/issues/21) · [L1 pipeline #18](https://github.com/ClachDev/Mote/pull/18)
- [Depth from Motion for Smartphone AR — Valentin et al., ACM TOG 2018](https://dl.acm.org/doi/10.1145/3272127.3275041) ([PDF](https://3dvar.com/Valentin2019Depth.pdf))
- [DeepVideoMVS — CVPR 2021, arXiv:2012.02177](https://arxiv.org/abs/2012.02177) ([code](https://github.com/ardaduz/deep-video-mvs))
- [SimpleRecon — Niantic, ECCV 2022](https://nianticlabs.github.io/simplerecon/)
- [MVSAnywhere: Zero-Shot Multi-View Stereo — CVPR 2025](https://nianticlabs.github.io/mvsanywhere/) ([code](https://github.com/nianticlabs/mvsanywhere))
- [SimpleMapping: real-time VI dense mapping with deep MVS — arXiv:2306.08648](https://arxiv.org/pdf/2306.08648)
- [VGGT: Visual Geometry Grounded Transformer — CVPR 2025](https://arxiv.org/html/2503.11651v1)
- [MASt3R-SLAM explained (runtime figures)](https://learnopencv.com/mast3r-slam-realtime-dense-slam-explained/)
- [MASt3R-Fusion: feed-forward visual model + IMU/GNSS SLAM — arXiv:2509.20757](https://arxiv.org/html/2509.20757v1)
- [Video Depth Anything — CVPR 2025](https://github.com/DepthAnything/Video-Depth-Anything)
- [Online Video Depth Anything (oVDA) — arXiv:2510.09182](https://arxiv.org/html/2510.09182v1)
- [FlashDepth: real-time streaming video depth — arXiv:2504.07093](https://eyeline-labs.github.io/FlashDepth/)
- [UniDepthV2 — arXiv:2502.20110](https://arxiv.org/abs/2502.20110)
- [Metric3D v2 — TPAMI 2024](https://ieeexplore.ieee.org/document/10638254/)
- [Depth Pro — Apple, arXiv:2410.02073](https://arxiv.org/html/2410.02073v1)
- [VIMD: monocular visual-inertial motion and depth (sparse-anchor alignment critique) — arXiv:2509.19713](https://arxiv.org/html/2509.19713)
- [Metrically scaled monocular depth via sparse priors — arXiv:2310.16750](https://arxiv.org/pdf/2310.16750)
- [Robust multi-view depth benchmark — arXiv:2209.06681](https://arxiv.org/pdf/2209.06681)
- [Avoiding degeneracy in monocular SLAM (epipole/triangulation) — arXiv:2103.01501](https://arxiv.org/pdf/2103.01501)
