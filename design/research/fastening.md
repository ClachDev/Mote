# Fastening & Mounting Grid Research

*Researched July 2026. Context: issue #5 — the ORP-based mounting works but is
sub-optimal: some mounts need edge-hugging shapes or a 10 mm pitch to fit, the
captive-nut hex pockets force an assembly order/direction (and make the Pi
vulnerable to an over-long screw), and nuts loosen under driving vibration.*

---

## Problem Statement

Three separate pains, often conflated:

1. **Placement granularity.** The ORP grid (Ø3.5 mm holes on a 20 mm pitch) is
   too coarse to position every mount where it needs to be on a 235 mm plate.
2. **Assembly order & direction.** Hex nut pockets keep the nut still while
   tightening (good), but they fix which side the bolt enters from and force
   sub-assemblies to be built in a set order. An over-long screw can bottom out
   past the pocket — near the Pi this risks board damage.
3. **Vibration loosening.** Plain M3 nuts back off while driving
   (already flagged in [ASSEMBLY.md](../ASSEMBLY.md)).

---

## Measured Current State

Extracted from the committed STLs with [`hole_survey.py`](hole_survey.py)
(binary-STL loop detection, numpy only — re-run it after any CAD change):

| Part | Ø3.5 grid holes | Grid pitch | M3 hex pockets |
|------|----------------|------------|----------------|
| Chassis Base | 101 | **20 mm, pure ORP** | none |
| Chassis Top | 97 | **20 mm, pure ORP** | none |
| Motor Support | 4 | 20 mm | none |
| Battery Mount | 13 | 10 mm | 4 |
| C1 Lidar Mount | 16 | 10 mm | ~6 (side-loaded) |
| Waveshare Mount | 8 | 9–10 mm | ~4 (side-loaded) |
| Pi Bottom | 27 | 10 mm | 3 |
| Camera Mount | 8 | mixed 4–10 mm | 1 |

Two findings worth stating explicitly:

- **The plates are already clean ORP.** Every one of the ~200 plate holes is
  Ø3.5 mm at exactly 20 mm nearest-neighbour spacing. The deviations all live
  in the *mounts*.
- **The mounts have already converged on a 10 mm half-pitch.** A mount drilled
  at 10 mm pitch mating with a 20 mm plate grid gets 10 mm placement
  granularity by choosing which subset of its holes to use — the "more play"
  the issue asks for, without touching the plates. This is a de-facto rule
  today; it just isn't written down anywhere.

---

## Options

### 1. Finer / offset / hex plate grid

A full 10 mm plate grid is a strict superset of ORP 20 mm, so it *keeps* ORP
compatibility (an offset second grid is the same thing by another name). But it
roughly quadruples hole count (~400 holes/plate), costs print time, and weakens
a 6 mm PLA plate for a benefit the half-pitch mount rule already delivers with
zero plate changes. A hex grid is strictly worse: it breaks ORP
interoperability (a stated requirement) and is awkward to pattern in Fusion.

**Verdict:** Don't change the plates. Instead *formalise the existing
convention*: plates stay pure ORP 20 mm; mounts carry holes at 10 mm
half-pitch. Revisit a 10 mm plate variant only if a real placement need appears
that half-pitch mounts can't solve.

### 2. Enforced top-down assembly (hex cutouts under the chassis)

Makes the build order linear, but the cost lands on the weakest part: an M3 nut
pocket is ~2.4 mm deep, which is 40% of a 6 mm plate, times ~100 positions —
and pockets on the underside face the print bed, so they'd print as bridged
cavities or force printing the plates upside-down. It also bakes today's layout
into the plates: a pocket grid only helps where a mount happens to sit.

**Verdict:** Reject as a plate change. The *goal* (one-sided, any-order
assembly) is better reached with inserts (option 4).

### 3. Printed clip system

PLA clips creep and fatigue; snap-fit tolerances are printer- and
filament-specific, which fights the "anyone can print this" goal; and the
structural joints (motor supports, standoffs) see exactly the vibration loads
clips are worst at.

**Verdict:** Reject for structural joints. Clips (TPU/PETG) remain fine for
non-structural retention — battery strap, cable management — where
tool-free removal is actually wanted.

### 4. Heat-set brass inserts ⭐ Recommended

Replace each hex nut pocket with an M3 heat-set insert (the ubiquitous
M3×D5×L4, ≈Ø4.2 mm bore — check the datasheet of the insert you buy).
Installed once with a soldering iron (+£3 insert tip), ~£6 per 100.

This dissolves problem 2 rather than managing it:

- **One tool, one side.** No nut to hold — every joint tightens from the bolt
  side, in any order. Sub-assemblies stop being order-constrained.
- **Strength is fine.** CNC Kitchen's pull-out tests put M3 heat-set inserts at
  ~120 kg in PLA — *above* side-loaded hex pockets (~85 kg), below only
  in-line ("bottom") pockets (160 kg+). Most Mote pockets are side-loaded.
- **Vibration.** Brass insert + steel bolt takes medium-strength threadlocker
  (use a plastic-safe one near PLA); a nut pocket can't be threadlocked
  without gluing the nut into the part.
- **Screw-length hazard shrinks but doesn't vanish.** A too-long bolt still
  protrudes past the insert. Where the far side is a PCB (the Pi holder), give
  the insert a blind bore beneath it so an over-long screw hits plastic, never
  board — and document the correct length per joint in ASSEMBLY.md.

CAD cost is low and incremental: each hex pocket becomes a plain cylindrical
bore, one part at a time, no plate changes, old and new parts stay
inter-compatible (both are M3).

### 5. Vibration loosening (orthogonal to all of the above)

Cheap fixes that need no CAD at all, effective immediately:

- **Nyloc nuts** for through-bolted joints where the nut is accessible.
- **Medium-strength (blue) threadlocker** on metal-to-metal threads —
  plastic-safe formulations only near printed parts.
- Spring/serrated washers are a weaker third option.

---

## Recommendation

Adopt as a package, smallest change first:

1. **Now, no CAD:** add nyloc nuts / plastic-safe threadlocker to the BOM and
   ASSEMBLY.md for the loosening problem; document the assembly-order and
   Pi screw-length constraints so builders stop being surprised by them.
2. **Write the grid rule down:** plates are pure ORP (Ø3.5 mm / 20 mm);
   mounts use a 10 mm half-pitch. This is already true — formalising it is a
   documentation change (see design/README.md).
3. **Per-part, incrementally:** migrate mounts from hex nut pockets to M3
   heat-set inserts as each part next gets touched in Fusion — Pi holder first
   (it has the board-damage hazard), then the side-loaded pockets (lidar,
   Waveshare, battery), which are also the mechanically weakest.
4. **Don't** re-grid the plates, don't add underside pocket grids, don't move
   structural joints to printed clips.

---

## Sources

- [ORP design rules](https://openroboticplatform.com/designrules) — Ø3.5 mm
  holes, 20 mm X/Y pitch; no finer-grid provision exists in the standard.
- [CNC Kitchen: Helicoils, threaded inserts and embedded nuts — strength
  assessment](https://www.cnckitchen.com/blog/helicoils-threaded-insets-and-embedded-nuts-in-3d-prints-strength-amp-strength-assessment)
  — M3 pull-out: printed thread / insert / Helicoil all ≈120 kg, side nut
  pocket ≈85 kg, in-line nut pocket 160 kg+.
- [Tom's 3D on threaded connections in prints](https://toms3d.org/2025/01/14/a-better-way-to-add-threads-to-your-3d-prints/)
- [`hole_survey.py`](hole_survey.py) — the measurements in this document.
