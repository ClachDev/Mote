# map-build

One command that turns a recorded mapping bag into a **map revision a human can
review and promote**. Stage 2 of `docs/design/mapping-pipeline.md`: capture
produces a bag, a build produces a map, a human promotes it.

```bash
pixi run map-build -- \
  --bag ~/.mote/bags/mapping/20260802_142539 \
  --site home --floor ground \
  --baseline ~/.mote/sites/home/floors/ground/map
```

It needs no robot, no live SLAM session and no map to already exist. It runs on
any machine with the project's default pixi environment and a copy of the bag.

Output lands in `map_build_results/<UTC>/`:

| | |
|---|---|
| `report.md` | **open this** — inputs, stages, validation, metric diff, renders |
| `revision/<rev>/` | the map revision itself, in the layout `save-map` writes |
| `<rev>.tar.gz` | the same revision packed as the registry accepts it |
| `build.json` | everything in the report, as data |
| `build/` | the replay leg: `stack.log`, `replay.log`, `series.json`, `map.npz` |

## The chain

1. **solve** — lockstep replay of the whole bag through slam_toolbox under
   `mote_bringup/config/slam_toolbox_build_params.yaml`, using the harness in
   `../bag_replay`. A 21-minute bag solves in tens of seconds; that economy is
   what makes rebuilding cheap enough to be the normal path.
2. **assemble** — the finished grid and the serialized posegraph written as
   `map.yaml` + `map.png` + `map.posegraph` + `map.data`. A solve that
   serialized no posegraph **fails the build**: a revision without one cannot
   be extended later, and the frame — with every zone taught in it — is gone.
3. **declutter** — `sites.promote_cleaned`, the robot's own FFT structure pass,
   called rather than copied. The untouched solve is kept as `map_raw.png`.
4. **segment** — one polygon zone per room of the cleaned map, named
   `room_01`… for the reviewer to rename.
5. **validate + score** — `bundle.validate` (**hard**: a revision with an error
   is not emitted), then truth-free metrics diffed against `--baseline`
   (**soft**: a regression is printed as review evidence).
6. **package** — the gzipped bundle, plus the report.

## What it does not do

**It does not align the map frame.** The design's alignment step measures a
solved map's wall rotation, re-solves with that yaw injected, and keeps the
better of the two. Deciding "better" needs an estimator that can see the
difference, and the one in the tree cannot: it called four of the seven banked
2026-08-02 solves square when they were 3.5–5.6° off. A re-solve is not a rigid
rotation either — the same −3.0° injection moved three solves of one bag by
+0.1°, −4.3° and −5.8° — so the step is *undecidable* here, not merely ungated.
That estimator is task 615; the evidence is
`docs/tuning/2026-09-01-alignment-residual.md`.

Until it lands, birth-alignment is an operator's judgment: `--frame X Y YAW`
passes an SE2 through to the solve, and it is recorded in the revision's
`meta.yaml`, so a map built that way is still reproducible. The build measures
and prints the map's wall structure either way, as evidence rather than as a
gate.

**It does not carry a floor's zone names forward.** Re-binding the previous
revision's names onto new geometry is task 345. The build emits the segmenter's
placeholders and *reports* what the baseline floor's places were called, so the
gap is visible rather than silent.

**It does not upload.** The registry's candidate-upload route accepts enrolled
robots only; a builder needs a credential of its own (task 344). The bundle is
emitted locally and the report says what will send it.

## Options

| | |
|---|---|
| `--bag DIR` | a recorded mapping bag directory (required) |
| `--params FILE` | slam parameters; default the committed build params |
| `--site` / `--floor` | stamped into the revision's zone documents |
| `--baseline PATH` | a revision directory (or its `map.yaml`) to diff against |
| `--frame X Y YAW` | birth-align the map frame by this SE2 |
| `--no-clean` | serve the raw solve; for ground-truth-clean maps (sim) |
| `--no-segment` | propose no room zones |
| `--paced` | feed against the wall clock instead of in lockstep |
| `--max-scans N` | stop after N scans — a quick smoke build |
| `--out DIR` | where the UTC-stamped result directory lands |

`--paced` exists for a parameter set whose gates `bag_replay/acceptance.py` has
not been validated against; see that harness's `--validate` mode. Everything
else about the feed is its business, not this tool's.

## Reproducing a build

`meta.yaml` in every emitted revision names the exact inputs:

```yaml
built_by: map-build
bag: 20260802_142539
bag_sha256: 099f9d06…
slam_params: slam_toolbox_build_params.yaml
slam_params_sha256: 072929cf…
frame: [0.0, 0.0, -3.0]
feed: lockstep
harness_commit: 1c75eac
```

The bag digest covers every file's bytes *and* its name, so "the same bag" means
the same bytes rather than the same directory name. Re-running the same
`map-build` on the same inputs at the same harness commit reproduces the map —
to within the solver, which is not bit-identical run to run (small deltas on the
proxies are noise; the map images decide anything marginal).

## Reading the metrics

They are **proxies, not error measures** — the bag carries no ground truth, so
a confidently wrong map (a mis-closed loop drawn with sharp walls) can score
well on all of them. `bag_replay/README.md` "Limitations" is the full list of
what they can and cannot prove. Two that matter here:

- **Loop drift needs a loop.** It cannot tell a legitimate open A→B traverse
  from a drifting one. Know the bag's shape before reading it.
- **`angular_support_deg` is not in the diff table** and must not be used to
  rank candidates: it is confounded by coverage, so a map that explored less
  reads as tighter.

The wall-structure table is the one thing crispness cannot see: a second
orthogonal frame carrying real energy, with **two** directions in it, means a
section of the map is drawn on its own axes — a tear. A second frame with one
direction is an angled hallway, which is architecture.

## Tests

```bash
python mote_simulation/tools/map_build/test_map_build.py
```

Covers the ROS-free half: the pixel convention a revision is written in and the
thresholds `map.yaml` declares for reading it back (get those wrong and unknown
space reads as free — as somewhere the planner may drive straight through), the
origin, the bag digest, and the metric diff's direction. The solve needs
slam_toolbox and a real bag and is exercised by running the tool.
