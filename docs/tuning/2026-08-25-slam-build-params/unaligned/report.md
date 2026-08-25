# Bag-replay scoring report

- **generated (UTC):** 20260825T101704Z
- **git commit:** `ae37014`
- **bag:** `/home/michael/Projects/data/mote-bags/20260802_142539`
- **mode:** slam  ·  **feed:** lockstep
- **parameter sets:** 3

## Metrics

Truth-free proxies — see Limitations. Arrows mark the better direction.

| metric | car-0.0349 | car-0.0175 | car-0.01745 |
| --- | --- | --- | --- |
| scans replayed | 542 | 542 | 542 |
| pose-graph nodes | 340 | 340 | 340 |
| replay wall (s) | 21 | 26 | 21 |
| traj samples | 340 | 340 | 340 |
| path length (m) | 139.83 | 139.60 | 139.69 |
| loop drift (m) ↓ | **0.087** | 0.097 | 0.109 |
| drift ratio ↓ | **0.0006** | 0.0007 | 0.0008 |
| explored area (m²) ↑ | 62.2 | **62.2** | 62.1 |
| unknown frac | 0.425 | 0.433 | 0.437 |
| occupied frac | 0.1034 | 0.1029 | 0.1016 |
| wall thickness (m) ↓ | **0.064** | 0.065 | 0.065 |
| speckle frac ↓ | 0.0192 | 0.0184 | **0.0172** |
| angular support (°) | 54.86 | 53.43 | 52.29 |
| wall frames (≥15% energy) | 2 | 2 | 2 |

## Maps

### car-0.0349

- params: the committed build params with `coarse_angle_resolution: 0.0349`

![map for car-0.0349](car-0.0349/map.png)

**Wall directions** (wall orientations, image frame, energy-weighted):

| angle (°) | energy frac | width (°) |
| --- | --- | --- |
| 92.75 | 0.270 | 2.69 |
| 163.75 | 0.104 | 2.33 |
| 2.75 | 0.290 | 2.68 |
| 80.75 | 0.100 | 3.24 |

**Orthogonal frames** (dominant share 0.560):

| frame (°) | energy frac | directions | offset from dominant (°) |
| --- | --- | --- | --- |
| 2.75 | 0.560 | 2 | 0.0 |
| 73.75 | 0.204 | 2 | 19.0 |

### car-0.0175

- params: the committed build params with `coarse_angle_resolution: 0.0175`

![map for car-0.0175](car-0.0175/map.png)

**Wall directions** (wall orientations, image frame, energy-weighted):

| angle (°) | energy frac | width (°) |
| --- | --- | --- |
| 95.25 | 0.262 | 2.61 |
| 166.75 | 0.127 | 2.46 |
| 4.25 | 0.301 | 2.66 |
| 75.75 | 0.072 | 2.47 |

**Orthogonal frames** (dominant share 0.562):

| frame (°) | energy frac | directions | offset from dominant (°) |
| --- | --- | --- | --- |
| 4.25 | 0.562 | 2 | 0.0 |
| 76.75 | 0.199 | 2 | 17.5 |

### car-0.01745

- params: the committed build params with `coarse_angle_resolution: 0.01745`

![map for car-0.01745](car-0.01745/map.png)

**Wall directions** (wall orientations, image frame, energy-weighted):

| angle (°) | energy frac | width (°) |
| --- | --- | --- |
| 95.25 | 0.277 | 2.63 |
| 166.75 | 0.130 | 2.44 |
| 5.25 | 0.308 | 2.63 |
| 75.75 | 0.060 | 2.32 |

**Orthogonal frames** (dominant share 0.585):

| frame (°) | energy frac | directions | offset from dominant (°) |
| --- | --- | --- | --- |
| 5.25 | 0.585 | 2 | 0.0 |
| 76.75 | 0.190 | 2 | 18.5 |

## Limitations

These metrics are **truth-free proxies**, not error measures. A real bag carries no surveyed ground truth, so unlike the sim benchmark (which has Gazebo's true pose and reports ATE), this harness can only score *self-consistency* and *map appearance*:

- **Loop drift** is only meaningful when the robot physically returned to its start — it cannot distinguish a legitimate open A→B traverse from a drifting loop. Know the bag's shape before reading it.
- **Map crispness** (wall thickness, speckle, unknown fraction) catches blur, noise, and incompleteness. It does **not** catch a confidently *wrong* map: a mis-closed loop drawn with sharp walls scores well here.
- **Angular structure** is a **tear detector, not a quality ranking**, and is deliberately not bolded. It answers the one question loop drift cannot: loop drift is only meaningful when the trajectory *closes*, so a session that exits on its exploration budget gets no drift number at all, and for those maps the frame table below is the only automated tear signal there is. Read it like this:

  - **`wall frames` > 1 with real energy share means two rectangular systems in one map** — i.e. a section drawn on its own axes. That is what a SLAM tear looks like. Check the per-map frame table for the offset; run 3's two legs were torn by 25° and 41°.
  - **One extra *direction* is architecture, not damage.** A flat with an angled hallway genuinely has three wall directions. The frame table distinguishes them: a rotated section duplicates a whole frame (`directions: 2`), a hallway adds one (`directions: 1`).
  - **It is blind below ~10°**, the frame merge tolerance, which has to exceed the shear a genuine frame carries (7.5° measured on a real leg) or honest shear would read as a tear. A small rotation will show one frame. Catching that needs a declared direction set for the site, which `angular_stats(..., reference_directions=...)` accepts and this report does not yet supply.
  - **`angular support` is confounded by coverage** and must not be used to rank: a map that explored less has fewer long walls and reads as tighter. On the 2026-07-29 run-3 pair the leg that is better by loop drift (0.551 m vs 8.776 m) scores *worse* on it (42.2 vs 39.3), having covered 59 m² against 81 m². It is here to be read beside `explored area`, not to pick a winner.
- No absolute scale/position check is possible without a reference map or survey. For metric-accuracy claims, use the sim benchmark's ATE.
- Replaying the same recorded sensor stream makes the comparison deterministic in its *input*, but SLAM's solver is not bit-exact run-to-run; treat small deltas as noise.
- **Trajectory rows are only comparable within one feed mode.** A paced leg samples `map→base_link` off TF on a fixed period; a lockstep leg takes the pose graph's own node poses, because `map→odom` is broadcast on a wall-clock timer that a lockstep leg outruns. Path length and drift ratio therefore differ by *sampling*, not by quality — the map rows and `pose-graph nodes` are the ones that cross the two.
