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
  to square the walls (two solves; the live 2.0° `coarse_angle_resolution`
  snaps solutions to a ~2° orientation lattice, so alignment needed a
  build-only 1° lattice), a declutter pass with a hand-tuned peak threshold
  (the default admitted a phantom wall direction — task 337), room
  segmentation, and validation.
- Getting the artifact onto the robot and into the registry required
  hand-assembly of a revision (the layout `save-map` writes, reproduced
  manually), an rsync side-load, a manual symlink flip, and a `publish-map`
  whose design assumes the robot built the map. Distribution order ran
  backwards (robot first, registry second).
- Zones: `segment-map` proposed seven placeholder rooms; renaming is manual;
  the one taught pose (`office`) was invalidated by the new frame and must be
  re-taught by driving to it. The candidate was published with placeholder
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
   divergences — closure settings tuned for the best map, 1° angular lattice).
2. **Align** — measure the dominant wall orientation of the solved map
   (windowed angular energy, folded 0/90, sub-bin interpolation), re-solve
   once with the measured yaw injected as the frame birth-alignment. Assert
   the residual (< 0.5°) or fail the build loudly.
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
- **Edit before promoting.** Rename a zone, accept/reject a proposed match,
  edit aliases/kind, adjust or delete a polygon, and **click-to-teach a pose**
  (place `office` by clicking the office on the map — replaces drive-to-teach
  for goto targets; taught-by-driving remains for poses that need real
  approach headings). Edits write back to the candidate's `zones.yaml` on the
  server (bumping `vocabulary_revision`); the candidate stays inert
  throughout.
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
| **New** | Bag sync + retention (sync-then-prune); build orchestrator + committed build params; build identity for candidate upload; vocabulary carry-forward; candidate preview/edit UI; build report |

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

## Work breakdown

Sized so each is one dispatchable task; existing tasks noted.

1. **Live params to best-known-good** — task 335 (chain 15 → 10, comment
   rewrite). Unchanged by this design; the divergence note lands with item 3.
2. **Bag sync + sync-then-prune** — robot uploads mapping bags to the fleet
   box on session end; pruner trims only confirmed-synced bags; bag store
   beside the registry with checksums.
3. **Build params file + divergence note** — `slam_toolbox_build_params.yaml`
   committed beside the live file; every divergent key carries a one-line why.
4. **`map-build` orchestrator** — the stage-2 chain as one command on the
   fleet box, emitting a validated candidate + build report. Depends on the
   lockstep harness landing (task 295 / PR 91) and prominence picking (337).
5. **Build identity** — the registry accepts candidate uploads from a
   credentialed builder, not only enrolled robots; audit rows name it.
6. **Vocabulary carry-forward** — same-frame rebinding by containment +
   new-frame proposals, emitted into the candidate with a carry-forward
   report.
7. **Candidate preview** — task 339 (see the map you promote), extended with
   the zones overlay.
8. **Candidate zone editing** — rename/alias/kind/polygon/click-to-teach on
   the candidate from the dashboard; server-side writes to candidate
   `zones.yaml`, audited, inert until promotion.
9. **Build report on the review page** — stage-2 scoring diff rendered beside
   the promote picker.
