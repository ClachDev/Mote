# Mounting Grid Survey: Measured State of the v1 Parts

*Measured July 2026, in the course of issue #5. This note records what the
committed STLs actually contain — hole counts, pitches, pocket census — and
the one design rule the measurements support. Fastening direction is covered
separately: the option survey and candidate scheme live in
[fastening.md](fastening.md), and both feed the
[v2 redesign process](../REDESIGN.md), which owns per-joint decisions.*

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
  the issue asks for, without touching the plates. This was a de-facto rule;
  it is now written down (see design/README.md, "Mounting grid").

One caution for anyone reasoning from this table: the pocket census counts
geometry, not usage — which pockets are side-loaded versus open-topped
matters for the fastening discussion (open-top slots rely on the mounted
component to retain the nut), and the per-part details above should be
re-verified in CAD before a part is revised.

## The half-pitch grid rule

A full 10 mm plate grid would be a strict superset of ORP 20 mm, so it would
keep ORP compatibility — but it roughly quadruples hole count (~400
holes/plate), costs print time, and weakens a 6 mm PLA plate for a benefit
the half-pitch mounts already deliver with zero plate changes. A hex grid is
strictly worse: it breaks ORP interoperability and is awkward to pattern in
Fusion.

**Rule:** plates stay pure ORP (Ø3.5 mm / 20 mm); mounts carry their holes at
10 mm half-pitch. Revisit a finer plate grid only if a placement need appears
that half-pitch mounts can't solve — and route that decision through the
redesign process, which also owns the question of how much plate area remains
general-purpose grid at all.

---

## Sources

- [ORP design rules](https://openroboticplatform.com/designrules) — Ø3.5 mm
  holes, 20 mm X/Y pitch; no finer-grid provision exists in the standard.
- [`hole_survey.py`](hole_survey.py) — the measurements in this document.
