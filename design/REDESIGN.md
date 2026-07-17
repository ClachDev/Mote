# The v2 Redesign Process

*Written July 2026, out of the review of the fastening research
([research/fastening.md](research/fastening.md)). Mote's printed hardware is a
working prototype with known flaws. This document is the path from that
prototype to a second iteration deliberately engineered for FDM manufacture by
other people — and the process is the point: individual design decisions
(fastening included) are made inside it, per joint, not globally up front.*

---

## Why a process

The fastening research went through three framings — better fasteners, better
joint geometry, a clever plate — and each produced a workable scheme that
still felt off. The diagnosis, once stated, is obvious: they were all uniform
systems applied to every joint at once, preserving a generality (any part,
anywhere, any time) that the design no longer needs. That is prototype
thinking carried past the prototype.

A professional pass over the same hardware reads differently. The loads are
trivial — an M3 joint carries hundreds of newtons of preload and this robot's
joints see a few — so strength is free and every scheme "works", which is
exactly why choosing between them felt arbitrary. The real requirements are
serviceability, position accuracy, and assembly ergonomics. Loose nuts are a
metal-fabrication idiom that plastics products simply don't use: both sides
of every Mote joint are printable geometry, and consumer plastic products
ship with thread-forming screws in blind bosses, snap hooks, and not a nut
anywhere. Location and clamping are different jobs — parts should locate off
datum features, with fasteners only clamping — and the wheel mounts having to
be hand-fitted to place the wheels is the symptom of that separation missing.
Finally, the top-plate access problem is architectural: a two-plate sandwich
creates joints that face inward. No fastener choice fixes an architecture.

The SO-101 arm shows what the destination looks like for printed parts made
by strangers: teardrop screw holes because print orientation was fixed first
and features designed for it; friction-fit rings oriented so loads run along
layer lines; no purchased fasteners beyond what ships in the servo bags. None
of that is a single clever decision — it is the residue of rules applied part
by part over two iterations. Mote v1 is our SO-100. This process is how we
get to our SO-101.

---

## Requirements (draft — ratified by reviewing this document)

What v2 must satisfy, collected from operating v1. Amendments belong in this
section, not in people's heads.

- **Built by others.** Unknown printers and filaments, so tolerance variance
  is a first-class constraint: any interference or sliding fit must be
  covered by a calibration coupon the builder prints first, and tolerance-
  critical features must state their class (clearance / slip / press).
- **Swappable components behind fixed interfaces.** The camera and the power
  bank are the availability-sensitive parts; each mounts via a defined
  interface (a printed interface plate or documented hole pattern) so a
  variant swaps at one boundary without touching the chassis. Everything
  else (Pi 5, STS3215, Waveshare board, C1 lidar) is treated as standard.
- **Still iterating.** v2 is not a freeze; parts will be revised. Reprints
  are cheap; design decisions should prefer geometry over purchased
  hardware, and purchased hardware should prefer what already ships with the
  components (the Feetech screw bags, factory threads) over new BOM lines.
- **Serviceability by declared class, not uniformly.** Each joint's service
  frequency is declared in the joint schedule and its fastening chosen
  accordingly — set-and-forget joints may use low-cycle solutions; only
  frequently-opened joints (today: the SO-101 arm mount, battery access) pay
  for quick access.
- **ORP as an expansion zone, not a universe.** Plate area reserved for
  future payloads keeps the pure Ø3.5 mm / 20 mm ORP grid; converged
  subsystems may trade grid generality for bespoke features. How much area
  stays ORP is decided in Phase 2.
- **Print constraints:** all parts fit a 256 mm bed; each part has one
  declared print orientation and must print without heroic supports.

## The process

**Phase 1 — audit.** Two artifacts, no redesign yet. First, complete the
joint schedule (seeded below): every joint, its function, load class, service
frequency, current fastening, observed pain. Second, a teardown study of
exemplars — the SO-101 CAD and physical arm (extract the *rules* it
embodies, not admiration), TurtleBot 3's tileable waffle plates (measure the
actual pattern from the published CAD), and one consumer product for
contrast. `research/hole_survey.py`-style measurement over recollection.

**Phase 2 — design rules.** Distill the audit into `design/DESIGN-RULES.md`,
Mote's equivalent of the ORP design rules: print orientation chosen per part
before features; teardrop or bridged holes where geometry fights orientation;
loads cross layer lines only with justification; the tolerance classes and
the builder calibration coupon; the fastener policy (per the joint schedule;
prefer shipped hardware; no loose nuts; datum features locate, fasteners
clamp); the camera/battery interface definitions; the ORP expansion-zone
decision. The rules doc is living — every later phase amends it when reality
disagrees.

**Phase 3 — iterative redesign, one subsystem at a time.** No big-bang v2.
Order by risk and pain: the drivetrain module first (servo mount + wheel —
highest loads, and the wheel-position datum problem), then the chassis
architecture decision (two-plate sandwich vs single structural tray —
decided deliberately with a cheap cut-down prototype, not inherited), then
the remaining mounts, then the camera/battery interface plates. Each
iteration: design to the rules → print → assemble → defect log → amend the
rules. Fastening for each joint is chosen here, against the schedule and the
rules — which is why no global fastening decision is taken today.

**Phase 4 — validation.** "Built by others" has an integration test: a
from-scratch build following only the published docs, ideally on a different
printer or by a different person, with a timed defect log. Every stumble
becomes a task.

---

## Joint schedule (seed — completed in Phase 1)

Load class: S = structural (drive/ground forces), P = positional (must not
shift), R = retention (holds a component in place). Service: how often the
joint is actually opened in practice.

| Joint | Function | Load | Service | Current fastening | Observed pain |
| --- | --- | --- | --- | --- | --- |
| Servo → Motor Support | S | S | rare | M3 + bare nuts on lugs | Nuts loosen (no slot restraining them) |
| Motor Support → Chassis Base | S | S | rare | M3×12 + captive slots | Fine in service |
| Wheel Inner → servo horn | S | S | rare | Factory horn screws | Fine |
| Wheel position vs chassis | P | P | — | Hand-fitted at mount design time | No datum: mount/plate pair is bespoke |
| Caster → Chassis Base | S | S | rare (part provisional) | M3 + captive slots | Part itself unresolved |
| Battery Mount → Chassis Base | R | P | rare | M3×12 + captive slots | Fine |
| Power bank → Battery Mount | R | R | frequent (charging/swap) | Seated in printed mount | Access between plates |
| Waveshare Mount → Chassis Base | R | P | rare | M3×12 + captive slots | Fine |
| C1 Lidar Mount → Chassis Base | P | P | rare | M3×12 *exact* + slots trapped by lidar | Nut retention depends on lidar; screw length critical |
| Lidar → C1 Lidar Mount | P | P | rare | Factory threads | Only reachable with mount off plate |
| Pi Bottom → Chassis Top | R | P | rare | M3×10 from below | Inside-sandwich access; M3×12 damages board |
| Pi board → holders | R | R | occasional | Printed retention | — |
| Camera Mount → Chassis Top | P | P | occasional | Stud workaround, free nuts below | Loose nuts, awkward tightening |
| Chassis Top → standoffs | S | S | occasional (any top change) | Screws into 50 mm standoffs | Standoffs missing from BOM |
| SO Base ORP → Chassis Top | S | S | **frequent** (arm on/off) | 4× top-down, free nuts under plate | Blind under-plate access |

---

## Relationship to existing work

[research/fastening.md](research/fastening.md) is the fastening survey and
carries the strongest candidate scheme found before this process existed
(trapped-nut plate + roofed slots); it stands as the default answer for any
joint the schedule doesn't overrule, and its "considered and not adopted"
section is the option playbook Phase 3 draws from.
`research/mounting_survey.md` (PR #38) holds the measured state of the v1
parts and the half-pitch grid rule, and its `hole_survey.py` is the
measurement tool Phases 1 and 3 reuse. The ORP field notes in the fastening
doc are the seed of the Phase 2 expansion-zone and datum discussions.
