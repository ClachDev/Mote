# Auto-segmenting a saved map into rooms — validation

`pixi run segment-map` proposes one zone per room of a saved map so a floor
arrives with its rooms outlined rather than each one taught by driving to it.
This is what it scores on the sim ladder, and what it gets wrong.

Method and parameters: `mote_bringup/mote_bringup/map_cleanup/README.md`.
Harness: `mote_simulation/test/room_segmentation_eval.py` (`pixi run
segment-eval`), scoring against the walkable rectangle of every enclosed room in
`mote_simulation/worlds/<world>.rooms.yaml` — emitted by `gen_hospital.py` for
the generated world, read off the SDF for the two hand-written ones. Raw output
and the three overlays are in `2026-07-27-room-segmentation/`.

## What is being measured

Over the **observed free pixels** of each ground-truth room:

| term | meaning |
| --- | --- |
| `mapped` | truth rooms with ≥50% of their area observed free — the rest were never visited |
| `recovered` | one candidate covers ≥60% of the room and does not also cover ≥35% of another |
| `split` | no candidate covers ≥60% of it: it came out in pieces |
| `merged` | its best candidate spans it *and* another room |
| `other` | candidates matching no truth room — corridor and other non-room free space |

Only mapped rooms are scored. The sim maps come from a timed autonomous
exploration run (`pixi run sim-map-world`), so 20 of the hospital's 53 rooms were
never entered; a room the robot never saw is not a segmentation failure.

`merged` is the failure that matters: a zone claiming two rooms answers "am I in
the kitchen" wrongly and sends `goto` to the wrong place. `split` and `other`
cost the operator a delete.

## Results

```
door<=1.4 m  wall-run>=1.5 m  min-area>=1.5 m^2
world             truth  mapped  cand  recovered  split  merged  other
mote_world            1       1     1          1      0       0      0
office_world         10      10    11         10      0       0      1
hospital_world       53      33    50         30      3       0     20
```

- **mote_world** — the one room, and nothing else.
- **office_world** — all 10 wards, plus the corridor as an 11th candidate (it is
  a plain rectangle of floor, so it is a legitimate zone; the operator renames or
  deletes it).
- **hospital_world** — 30 of the 33 mapped rooms, **no merges**. The 20 extra
  candidates are corridor stretches and fragments of the waiting hall, which is
  only ~40% observed and comes out in pieces where unmapped wedges cut it.

The three splits, from `--verbose`:

| room | observed | best cover | why |
| --- | --- | --- | --- |
| `[11.075, -10.675, 15.8, -6.075]` | 90% | 43% | cut in two along the bed's edge |
| `[11.075, -18.925, 15.8, -13.325]` | 84% | 42% | same |
| `[-28.925, -5.925, -23.325, -1.325]` | 84% | 0% | absorbed into the corridor ring, which is then dropped |

The first two are the bed: 1.9 m of it lies along one line, enough to be taken
for a wall, and the lidar shadow it casts speckles the rest of that line — so
neither side of it is left with a clear span wider than a door and the two
halves never merge back.

The third is the deliberate cost of refusing to propose a region that encircles
other rooms. Its dividers were never mapped, so it joined the corridor; the
corridor region (439.7 m² of floor with a 192.1 m² hole full of wards) is the
one candidate dropped as encircling, and the room goes with it. A room that
disappears is a better failure than a zone claiming the building.

## Rotation

A map frame's axes are wherever SLAM started, so a real map is rarely
axis-aligned; every map on the ladder is. Turning the map *and* the ground truth
before scoring (`--rotate`) exercises the alignment step against real SLAM data:

```
map turned  17 deg:  mote 1/1   office 10/10   hospital 31/33, 2 split, 0 merged
map turned -31 deg:  mote 1/1   office 10/10   hospital 31/33, 2 split, 0 merged
```

Unchanged, and this is the harder case — the harness rotates, then the segmenter
rotates back, so the walls are resampled twice where a real map is resampled
once. Two things had to be right for it:

- **The orientation scan is run on a square-padded wall image.** A frequency-
  domain array index is a real frequency divided by that axis' length, so on an
  oblong map the angular scan is skewed towards the long axis — 20° read as
  14.75° on the test fixture, and an 8° error at the hospital's 58×38 m aspect.
  (The declutter pass is immune: it places its wedges in the same index space it
  found them in.)
- **Wall runs are measured after closing pinholes along the run direction.** A
  wall is ~3 px thick at 5 cm; nearest-neighbour rotation resamples cells out of
  it, and without bridging those gaps a real wall breaks into runs too short to
  vote for a cut line. Before this, a rotated hospital scored 15/33 with 18
  merges; after, 31/33 with none.

## Real map

`~/.mote-fleet/sites/home/floors/ground` (the fleet box's copy of the robot's
own map — a real, cleaned, partially-mapped SLAM map with no ground truth)
yields 6 candidates: the two actual rooms come out whole, and the rest are
stretches of the corridor-like open area, cut where its ragged walls happen to
run. No ground truth exists for it, so it is a sanity check rather than a score.
