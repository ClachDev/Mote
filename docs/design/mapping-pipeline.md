# The mapping pipeline

How a space becomes a promoted map: **mapping as a build process** — the bag is
the source, the map revision is a build artifact, the parameters are the
toolchain, and promotion is a release. The robot's live map is scaffolding for
navigation during capture and is never the deliverable.

This is a design document. No implementation lands with it; the work breakdown
at the end is sized so each item becomes one dispatchable task.

> **Scope note.** This builds on seams that already exist: the site bundle and
> its validator (`mote_bringup/mote_bringup/sites.py`,
> `mote_bringup/mote_bringup/bundle.py`), the map registry and its promotion
> flow (`docs/design/fleet.md` M4, `mote_fleet/server/bundle_store.py`), the
> zone vocabulary split (names travel, coordinates do not — fleet-api.md), the
> declutter/segmentation passes (`mote_bringup/mote_bringup/map_cleanup/`), and
> the lockstep replay harness (task 295, `mote_simulation/tools/bag_replay/`).
> None of those move. What this design changes is **what happens between them,
> on which machine, and under whose control**.

---

## Evidence: the 2026-08-02 flat-mapping session

Every step below happened by hand, in a session that produced the best map this
project has had. That is the smell: the good outcome required leaving the paved
road at nearly every stage.

- The live session tore itself (a stuck-escape at a doorway injected drift) and
  the live config could not recover: `loop_match_minimum_chain_size: 15` needs
  a ~4.5 m chain of prior scans inside a 2.0 m search radius whose chord holds
  at most ~13 — closure candidates **cannot form**, at any drift. The saved
  live map (`home/ground/20260802T145731`) was discarded as an artifact and
  kept only as a baseline.
- The map that shipped was built **offline from the bag**: lockstep re-solve
  under corrected params (21 s per leg against ~35 min paced — the economics
  that make this whole design viable), a measured-then-injected frame rotation
  to square the walls (two solves, at a build-only 1° `coarse_angle_resolution`
  on the reasoning that the live 2.0° value snapped solutions to a ~2°
  orientation lattice — **there is no such lattice**, and the finer sweep beat
  the live value on nothing; see the alignment step below), a declutter pass
  with a hand-tuned peak threshold (the default admitted a phantom wall
  direction — task 337), room segmentation, and validation.
- Getting the artifact onto the robot and into the registry required
  hand-assembly of a revision (the layout `save-map` writes, reproduced
  manually), an rsync side-load, a manual symlink flip, and a `publish-map`
  whose design assumes the robot built the map. Distribution order ran
  backwards (robot first, registry second).
- Zones: `segment-map` proposed seven placeholder rooms; renaming is manual;
  the one taught pose (`office`) was invalidated by the new frame and had to
  be captured again (the dashboard's zone editor, which can now place it on the
  candidate, did not yet exist). The candidate was published with placeholder
  names because packing happens at publish time.
- The bags themselves — the actual source — live in the robot's
  `~/.mote/bags/` under a pruner that trims older bags on every recording run.
  Three weeks of mapping bags existed only there until they were pulled by
  hand this session.

Total: one excellent map, roughly a dozen manual judgment calls, and a paved
road that covers none of it.

## Principle

**Capture produces a bag. A build produces a map. A human review promotes it.**

- The **bag** is the product of driving the space. It is synced off the robot
  automatically, retained deliberately, and is sufficient to rebuild every
  downstream artifact from scratch.
- The **map revision** is a deterministic function of (bag, build params,
  harness version). Rebuilds are cheap (~seconds), so the build can afford
  steps a live session cannot: aggressive loop closure, alignment iteration,
  quality scoring against the previous revision.
- The **live map** exists so the robot can navigate while capturing. Its
  params are the best known-good configuration — never deliberately hobbled
  gates (a closure setting that cannot fire is a bug, not a tuning) — but it is
  held to a lower bar than the deliverable: a drifted live map degrades
  frontier exploration, not the product.
- **Live and build params may diverge, deliberately.** A false closure live is
  unrecoverable mid-mission; offline it costs one inspected, discarded
  candidate. So the live config carries conservative, proven values, and the
  build config may optimise further — every build output passes scoring and a
  human eye before release. The two configs live side by side in
  `mote_bringup/config/` with the divergence documented per key.

## The pipeline

Three machines, four stages, one direction of data flow:

```
 robot (Pi)              fleet box                     operator (browser)
 ───────────             ─────────────────────────     ──────────────────
 1 CAPTURE  ──bag──▶     2 BUILD   ──candidate──▶      3 REVIEW ──promote──▶  4 DISTRIBUTE
   drive + live SLAM       solve/align/clean/            see map + zones,       registry flips,
   (nav scaffolding)       segment/carry-forward/        edit zones,            retained announce,
                           validate/score                approve                robots pull
```

### Stage 1 — Capture (robot)

What runs: `pixi run mapping` + `pixi run explore` (or teleop assist), exactly
as today. The live SLAM stack navigates; the `mapping` bag stream records
`/scan_filtered`, `/tf`, odometry context.

What changes:

- **Bags sync off the robot automatically** when a mapping session ends (and
  opportunistically during long sessions): robot → fleet box, verified by
  checksum, into a bag store beside the map registry. The robot-side pruner
  may only trim bags the fleet box has confirmed — sync-then-prune, never
  prune-then-hope. (2026-08-02: 8 mapping bags existed only on the Pi, one
  recording run away from deletion.)
- **`save-map` stops being the deliverable path.** It remains as a
  bench/sim convenience (`map_world.sh` still uses it), but the operator flow
  never calls it: ending the session is enough.

Controls: session start/stop is the only operator input. The robot uploads
bags under its enrolled identity; it can no longer be the only holder of a
mapping session.

### Stage 2 — Build (fleet box)

Trigger: a new bag arriving, or an operator asking for a rebuild of an old bag
under new params. One orchestrator (`pixi run map-build <bag>`) chains what was
done by hand on 2026-08-02:

1. **Solve** — lockstep replay of the whole bag under the committed *build*
   params (`slam_toolbox_build_params.yaml`: the live file plus documented
   divergences, each one measured). Today that is one key,
   `loop_match_minimum_chain_size`, and only until the live file catches up:
   the live config is best-known-good by policy, so there is little for a build
   to win by parameter alone. What the build buys is the *steps* below.
   (`docs/tuning/2026-08-25-slam-build-params.md`.)
2. **Align** — measure the solved map's dominant wall orientation, re-solve
   once with that yaw injected as the frame birth-alignment, then **measure
   again and gate on the change**: the second solve must be at least as close
   to axis as the first, by more than the estimator's own resolution. It is a
   gate on improvement, not on absence, and the reason is that a map has no
   absolute residual to assert — on the 2026-08-02 flat, thirds of the building
   disagree about where the wall grid is by 8°, and there is a second family
   18° off that. Re-measuring is not belt-and-braces either: a re-solve is
   **not a rigid rotation** (slam_toolbox's correlation grid is a pixel grid,
   so a frame born 3° elsewhere finds different matches), and the same −3.0°
   injection moved three solves of one bag by +0.1°, −4.3° and −5.8°. The leg
   that did not move is the live-parameter one, i.e. the case the build will
   actually run.
   **This step is blocked on a measurement that does not exist yet** (work item
   10). `angular_stats.wall_rotation`, which the earlier draft of this design
   named, reports all seven banked 2026-08-02 solves as within 0.3° of square
   when four of them are 3.5–5.6° off, and mis-reads a known rotation below 2°
   by up to 0.75°. A projection-sharpness sweep over the same pixels finds the
   grid and holds a known rotation to 0.035° — but it is a throwaway probe, not
   a primitive. The before/after orientations and the tile spread ride on the
   candidate as review evidence either way. Evidence:
   `docs/tuning/2026-09-01-alignment-residual.md`.
3. **Declutter** — the FFT structure pass with prominence-based peak picking
   (task 337); no hand thresholds.
4. **Segment** — room polygons from the cleaned map, as today.
5. **Carry forward the vocabulary** — the previous revision's zone names,
   aliases, kinds and taught poses re-bind onto the new geometry: same-frame
   rebuilds by containment (a named zone whose pose lands inside a proposed
   room claims it); new frames get proposed matches for the operator to
   confirm in review. Placeholder names are minted only for genuinely new
   rooms. (This is the vocabulary/binding split doing work: names are the
   stable half, coordinates are rebuilt.)
6. **Validate + score** — `bundle.validate`, then the truth-free metrics
   (loop drift when the trajectory closes, explored area, speckle, wall
   thickness) diffed against the current canonical revision. Regressions
   don't block — they are printed on the candidate for the reviewer.
7. **Package + upload** — assemble the revision (map, raw, yamls, posegraph,
   meta with full provenance: bag id, params hash, harness commit) and upload
   it to the registry as a candidate under a **build identity** (today's
   registry accepts uploads only from enrolled robots; the builder needs the
   same standing — this is the M7 credential shape arriving early).

Controls: build params are committed and versioned; every candidate's meta
names its exact inputs, so any candidate is reproducible; a failed gate stops
the candidate, a soft regression rides along as review evidence. Nothing in
this stage touches a robot or the canonical map.

### Stage 3 — Review (operator, dashboard)

The operator decision point, and the place the 2026-08-02 session had nothing.
Requirements, in priority order:

- **See the candidate.** Selecting a candidate in the promote picker renders
  *that revision's* map in the pane (route for a candidate's `map.png`; task
  339), clearly marked as a preview, with a one-click return to canonical.
- **See the zones on it.** The candidate's zones draw over the preview exactly
  as canonical zones draw today (circle, polygon, waypoint cross), plus the
  carry-forward report: which names re-bound automatically, which are proposed
  matches, which rooms are new placeholders.
- **Edit before promoting.** *Built* (task 339's review pane,
  `server/ui/zone_editor.mjs`): rename a zone, write its note, adjust or delete
  a polygon, and place a pose by clicking the map (`⌖`), which is what replaced
  driving to a goto target — `save-zone` remains for poses that need a real
  approach heading. A save derives a new candidate from the one under review
  rather than writing into it (bumping `vocabulary_revision`); the source's
  bytes never change and the result stays inert until promoted. Still to do:
  accepting or rejecting a proposed carry-forward match.
- **See the build report.** The scoring diff from stage 2, on the same page
  as the promote button.
- **Promote.** Unchanged M4 semantics: audited operator action, symlink flip,
  retained announcement.

The browser's write surface grows from one audited POST (dispatch) to three
(dispatch, zone edits on a candidate, promote) — all operator-token
authenticated, all audited, all inert until promotion except promotion itself.

### Stage 4 — Distribute (unchanged)

M4 as built: retained announcement, agents pull and verify, effect at next
bringup, zones travel inside the revision. The 2026-08-02 session bypassed
this stage (side-load first, registry second); with stages 1–3 in place there
is no reason to ever bypass it again, and the robot-side `site use-map` /
rsync path is demoted to a bench tool.

## What stays, what changes, what is new

| | |
|---|---|
| **Stays** | Site bundle layout + `bundle.validate` both ends; M4 registry, promotion, announcement, pull; zone vocabulary split; live-mapping stack for capture; lockstep harness |
| **Changes** | Live SLAM params become best-known-good (chain 15 → 10 now — task 335 — and whatever proves out next); `save-map`/`publish-map` demoted to bench tools; promote picker becomes a review surface |
| **New** | Bag sync + retention (sync-then-prune); build orchestrator + committed build params; an orientation estimator the alignment step can be gated on; build identity for candidate upload; vocabulary carry-forward; candidate preview/edit UI; build report |

## Decisions

- **The live config is never deliberately hobbled.** Best-known-good values,
  always; "strict to the point of unreachable" was a bug wearing a tuning's
  clothes. (Operator decision, 2026-08-02.)
- **Optimisation beyond known-good happens in the build**, where output is
  scored and reviewed before it can affect a robot.
- **The operator sees and edits what they promote.** No promotion on faith in
  a timestamp. (Operator decision, 2026-08-02.)
- **The bag store is the system of record for mapping sessions**; the robot is
  never the sole holder of one.
- **Zones' human half survives rebuilds.** Renaming a flat's rooms is done
  once, not per revision.
- **A build gates on what it can measure, and reports the rest.** Every gate
  names a threshold its own measurement resolves, demonstrated on a real map
  rather than a synthetic one; anything softer than that is review evidence
  printed on the candidate. An assertion the measurement cannot see is worse
  than no assertion, because it is believed. (2026-09-01, from the alignment
  step: a "< 0.5° residual" gate would have passed a map 3.6° out.)

## Open questions

- **Cross-frame rebinding confidence** — matching rooms between unrelated map
  frames (shape/adjacency) is heuristic; how much do we trust auto-matches vs
  always proposing? Start: always propose, never auto-accept across frames.
- **Bag retention** — how long does the fleet box keep bags? (They are the
  source; disk is cheap; start with "forever, compressed", revisit at fleet
  scale.)
- **Build triggers** — auto-build on bag arrival vs operator-initiated. Start
  operator-initiated; auto later.
- **Map pixel edits** — erasing a phantom wall, closing a door the lidar saw
  open. Out of scope here; zones-only editing keeps the review surface small.
  A future task can add a mask layer that survives rebuilds the way names do.
- **Buildings with no single wall grid** — the 2026-08-02 flat has an angled
  wing 18° off its dominant family, and even within that family its thirds
  spread 8°. Aligning such a map squares some of it and turns the rest. Does
  the build say so on the candidate and leave it to the reviewer, or is a
  spread past some width a reason not to align at all? Start: align on the
  dominant family and print the spread. `room_segmentation` has the same
  assumption and the same answer.

## Work breakdown

Sized so each is one dispatchable task; existing tasks noted.

1. **Live params to best-known-good** — task 335 (chain 15 → 10, comment
   rewrite). Unchanged by this design; the divergence note lands with item 3.
2. **Bag sync + sync-then-prune** — robot uploads mapping bags to the fleet
   box on session end; pruner trims only confirmed-synced bags; bag store
   beside the registry with checksums.
3. **Build params file + divergence note** — landed (#106):
   `slam_toolbox_build_params.yaml` beside the live file, one divergent key,
   held to it by `test_slam_build_params.py`. It also records what was measured
   and rejected, so the same sweep is not run twice.
4. **`map-build` orchestrator** — the stage-2 chain as one command on the
   fleet box, emitting a validated candidate + build report. Depends on the
   lockstep harness landing (task 295 / PR 91), prominence picking (337), and
   item 10 for the alignment step's gate.
5. **Build identity** — the registry accepts candidate uploads from a
   credentialed builder, not only enrolled robots; audit rows name it.
6. **Vocabulary carry-forward** — same-frame rebinding by containment +
   new-frame proposals, emitted into the candidate with a carry-forward
   report.
7. **Candidate preview** — task 339 (see the map you promote), extended with
   the zones overlay.
8. **Candidate zone editing** — *done* (task 339): rename, note, polygon and
   click-to-place on the candidate from the dashboard, deriving a new
   candidate, audited, inert until promotion. What remains is accepting or
   rejecting a carry-forward match, which needs step 6.
9. **Build report on the review page** — stage-2 scoring diff rendered beside
   the promote picker.
10. **An orientation estimator the alignment step can be gated on** — a
    primitive that resolves a change of 0.25° on a real map, with the
    validation table `wall_rotation` never had (a known rotation applied to a
    real solved map, not a synthetic outline), plus the tile spread that says
    whether the map has one wall grid at all. Blocks item 4's step 2.
    `angular_stats.wall_rotation` stays as the ≥3° measurement it is, with its
    limits corrected in place. Evidence:
    `docs/tuning/2026-09-01-alignment-residual.md`.
