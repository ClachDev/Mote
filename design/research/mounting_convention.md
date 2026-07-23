# Mounting Standards Beyond ORP: Datums, Tiling, and the Three-Layer Convention

*Researched July 2026 (task #146). Context: living with the first build surfaced
three flaws in leaning on [ORP](https://openroboticplatform.com/) alone
(recorded in [fastening.md](fastening.md), "Field notes: ORP after one build",
and measured in [mounting_survey.md](mounting_survey.md)): the 20 mm pitch is
coarse for dense fitting on a 235 mm plate; the standard fixes a hole pattern
but no **functional datums**, so the servo mounts had to be hand-fitted to place
the wheels and neither plate nor mount is reusable on another robot; and
fastening (nut retention, thread side, direction) is unspecified, so every
design re-solves it ad hoc. [#144](fastening.md) fixed the third with the
trapped-nut plate. This doc studies the prior art the field notes flagged —
TurtleBot 3 waffle plates, goBILDA, and the print-native tiling systems — and
works out what, if anything, a Mote mounting convention should specify beyond
ORP's hole pattern.*

---

## Problem Statement

A hole grid answers exactly one question: *where may a clamp screw go?* It says
nothing about where a part's working feature ends up in space, or how the joint
is held once the part is located. Those are separate questions, and ORP —
deliberately — answers only the first. The field notes are three symptoms of
that one gap:

- **Granularity** (a grid question): 20 mm is coarse; the mounts already recover
  10 mm via half-pitch drilling ([mounting_survey.md](mounting_survey.md)), and
  a finer *plate* grid was analysed and rejected in issue #5 / task #141
  (quadruples hole count, weakens a 6 mm plate, buys nothing the half-pitch
  mounts don't already deliver).
- **No functional datums** (the deep one): grid compatibility gave *fastener*
  interchange, not *part* interchange. The servo mounts were positioned by hand
  so the wheel sat at the right height and track — a constraint that lives
  entirely outside the standard.
- **Unspecified fastening**: solved separately by the trapped-nut plate
  ([#144](fastening.md)).

It helps to name the layers a complete mounting convention actually has, because
the confusion in the field notes is really that ORP is one layer being asked to
do the work of three:

1. **Grid** — the lattice of positions a clamp screw *can* occupy. (ORP: Ø3.5 mm
   holes on a 20 mm pitch.)
2. **Datum** — where a part's *functional* feature (a wheel-contact patch, an
   axle line, a sensor origin) sits relative to a known reference on the plate.
   ORP says nothing here.
3. **Fastening** — how the joint is made once the part is located: which side
   threads, how the nut is retained, assembly direction. ORP says nothing here
   either; [#144](fastening.md) is Mote's answer.

The rest of this doc surveys how other systems populate these three layers, then
asks what Mote should adopt in the two ORP leaves empty.

---

## Prior Art

### TurtleBot 3 waffle plates — the tiling case study

**Measured pattern.** ROBOTIS publishes no dimensioned drawing, and its official
`waffle_base.stl` is a simplified URDF *visual* mesh (sparse decorative holes, a
big centre octagon) — unusable for the real lattice, a discrepancy TB3 users have
logged against the repo ([issue #192](https://github.com/ROBOTIS-GIT/turtlebot3/issues/192),
[#437](https://github.com/ROBOTIS-GIT/turtlebot3/issues/437)). So the numbers
below were extracted (binary-STL rasterisation of the top face, the same loop-
finding approach as [hole_survey.py](hole_survey.py)) from a **faithful community
replica** — Printables model 243076, a 459k-triangle mesh that is a genuine
lattice (~52% solid / 48% open). Local hole geometry is scale-invariant and
cross-checks internally, so pitch and diameters are solid; the overall extent
carries a caveat (the model is titled "*Double* Waffle Plate").

- **Grid pitch: ~6 mm** (nearest-neighbour hole spacing 5.99 mm) — over **three
  times denser** than ORP's 20 mm. This is the direct answer to the granularity
  field note: a 6 mm pitch on a moulded plate is dense enough to approximate
  continuous placement, which is *why* TB3 never needed a half-pitch-mount
  workaround.
- **Two hole sizes on the grid:** ~5.0 mm (M3 clearance / head seat, ~108 of
  them) interleaved with ~2.7 mm holes every 12 mm (~112) — plus rounded
  **~17 × 10 mm slots**. The plate is a thin ribbed sheet (~9 mm tall envelope),
  ~137 mm square as modelled.
- **Fastener: M3** (documented — the retail set ships PHS M3×8 bolts + M3 nuts;
  [ROBOTIS store](https://www.robotis.us/tb3-waffle-plate-ipl-01-8ea/),
  [generationrobots](https://www.generationrobots.com/en/402842-tb3-waffle-plate-ipl-01-x8.html)).
  Same fastener family as ORP and Mote.

**The tiling mechanism.** The plates are injection-moulded engineering plastic,
and the "waffle" is meant to be built up, not used monolithically. Frames join
**edge-to-edge with plastic rivets** ("can connect 6 mm frames by using rivets",
[e-manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/features/)),
and stack **vertically on M3 plate-support standoffs** (the "Plate Support
M3×35 mm / M3×45 mm" parts). Because every hole is on the 6 mm grid, rivet and
standoff positions are free — any multiple of the pitch — so ROBOTIS specifies no
fixed rivet spacing. The tile *is* the unit; the chassis is however many tiles
you rivet together.

**What TB3 standardizes — and what it doesn't.** It nails two of the three layers
the field notes care about: a very dense **grid** (6 mm, solving granularity by
brute density) and a **tiling/assembly method** (rivets + standoffs, solving
size-decoupling and cheap damage replacement). But it pointedly does *not*
standardize **fastening** — bolts + loose nuts for components, rivets for plates,
with nut retention, thread side, and assembly order all left to the builder
exactly as ORP leaves them — and it carries **no functional datums**: a component
bolts anywhere on the lattice, so where a wheel or sensor ends up is still the
individual design's problem. TB3 solves granularity and extensibility; it does
not solve fastening or datums. That is the same split ORP has, one layer richer.

### goBILDA (and its Actobotics predecessor) — a grid that encodes function

goBILDA is the interesting counter-example to ORP, because its grid is not pure
geometry — it *encodes functional relationships*. The base pattern is **4 mm
holes on an 8 mm grid**, M4 hardware ([gm0](https://gm0.org/en/latest/docs/hardware-components/kit-and-hardware-guide/gobilda.html),
[goBILDA pattern](https://www.gobilda.com/pattern)). But layered onto that grid
are functional features at chosen multiples of 8 mm:

- A **14 mm bearing seat** hole, so a ball bearing drops directly into the
  pattern — "ball bearings are a pillar of the system."
- **24 mm** bearing-to-bearing spacing, chosen because MOD 0.8 gears mesh
  perfectly at that centre distance *and* it lands on the 8 mm grid — so a gear
  pair, its bearings, and their bolts all fall on pattern positions with no
  custom layout.
- The four holes nearest each bearing are **clocked** (rotated off-grid) to
  allow 45° mounting.
- Channel widths (48 mm full, 12×48 low-side, 12×32 mini) and 8 mm-pitch chain /
  HTD belts all resolve to exact centre-to-centre distances on the same grid.

That is the crucial contrast with ORP: goBILDA's grid *is* a partial datum
system. Where a bearing goes, where a gear meshes, where a belt closes — these
are pinned by the pattern, not re-solved per design. It buys this with an
ecosystem's worth of precision-moulded parts (bearings, hubs, MOD 0.8 gears)
whose bores and shoulders carry the *out-of-plane* datums the flat grid can't —
axle height still comes from a precision hub, not from the hole pattern. Its
predecessor **Actobotics** did the same in imperial units (a 0.770″ / 0.750″
hole pattern, 6-32 hardware, ½″ bores; [ServoCity](https://www.servocity.com/actobotics),
[gm0](https://gm0.org/en/latest/docs/hardware-components/kit-and-hardware-guide/actobotics.html));
goBILDA is its metric successor. Both answer all three layers — grid, a
part-family datum system, and fastening (tapped holes / clearance + nuts) — but
only by being a purchased ecosystem, which is exactly what ORP (and Mote)
declines to be.

### OpenBuilds — the framing paradigm, for contrast

OpenBuilds is a different animal worth one paragraph so the comparison is
honest: it is not a plate grid but a **structural-framing** system built on
20 mm V-slot aluminium extrusion, with fastening holes on a 20 mm pitch offset
10 mm from the edge, and mount points at ~20.25 mm centres
([OpenBuilds](https://us.openbuilds.com/v-slot-20x20-linear-rail/)). Datums come
from the extrusion's slots and faces (a part slides along a slot and clamps
anywhere), not from a hole pattern — continuous position instead of discrete.
It solves the granularity problem completely (infinite placement along a slot)
but at the cost of being metal stock, not a printable plate, and it carries no
robot-specific functional datums at all (axle height is whatever bracket you
bolt on). Relevant to Mote only as the reminder that "grid" is one of several
ways to pin position, and the continuous-slot alternative trades printability
for granularity.

### Print-native tiling — Gridfinity and Multiboard

The tiling idea the field notes flag (decouple chassis size from print-bed size;
make a damaged section a cheap replacement) has a mature print-native form worth
studying, separate from TB3's injection-moulded version:

- **Gridfinity** — a 42 mm baseplate grid; you print baseplates that tile
  arbitrarily and bins that drop in ([Gridfinity](https://gridfinitylayouttool.com/what-is-gridfinity)).
- **Multiboard** — a 25 mm grid with *screw-together edge connectors*, built for
  wall panels that tile to any size ([comparison](https://gridpilot.us/blog/gridfinity-vs-multiboard-vs-minutegrid)).

Both are proof that "tile printed modules to an arbitrary size" works and is
popular — but note *what* they tile: low-load organizers and wall panels, where
the seam between tiles never sees a structural load. That distinction is the
crux of whether tiling belongs on Mote, addressed below.

---

## What ORP Leaves Unspecified

Mapping the surveyed systems onto the three layers makes the gap precise:

| System | Grid | Datum layer | Fastening | Tiling / size-decoupling |
| --- | --- | --- | --- | --- |
| **ORP** | Ø3.5 / 20 mm, M3 | **none** — hole pattern only | **none** — unspecified | none (monolithic plate) |
| **TB3 waffle** | ~6 mm lattice, M3 (measured) | none (bolt-anywhere) | unspecified (bolts + nuts + rivets) | **yes** — riveted tiles |
| **goBILDA** | 4 mm / 8 mm, M4 | **partial** — bearing seats, gear mesh, belt C2C on-grid; hubs carry axle datums | tapped / clearance + nuts | none |
| **OpenBuilds** | 20 mm on extrusion | slot faces (continuous) | T-nuts in slot | n/a (cut stock to length) |
| **Gridfinity / Multiboard** | 42 / 25 mm | none | drop-in / edge screws | **yes** — printed tiles |

Two things fall out of the table. First, **no low-cost system gives all three
layers** — goBILDA comes closest but only by being a purchased ecosystem; ORP,
TB3, and the print-native systems each standardize a grid (and sometimes tiling)
and explicitly punt on datums and fastening. ORP's own design rules confirm this
directly: they specify hole diameter and pitch and *nothing* about edge margins,
plate thickness, datums, nut retention, thread side, or assembly direction
([ORP design rules](https://openroboticplatform.com/designrules)). Second,
**the datum layer is the one nobody hands you** — even goBILDA only pins the
in-plane, gear-train datums and offloads axle height to precision hubs. That is
not an oversight; it is because functional datums are *robot-specific*, which is
the whole difficulty and the subject of the next section.

---

## The Functional-Datum Question

This is the flaw with no off-the-shelf fix, so it gets the most space. The field
note's example — servo mounts hand-fitted to place the wheels — is a datum
problem: a reusable drivetrain part must guarantee where its working features
land, and a hole grid guarantees only where its *bolts* land.

**What a drivetrain interface standard would have to pin.** For a servo mount (or
a plate carrying one) to be reusable across robots without hand-fitting, these
functional features need defined positions relative to a single plate origin:

- **Axle height above the ground plane (Z).** The dominant datum: it sets ride
  height and ground clearance, and it must agree with the caster/support height
  or the robot sits tilted. It is coupled to wheel radius (contact patch = axle
  centre − radius), so it cannot be declared independently of the wheel.
- **Track / wheel-contact position in plane (Y separation, X fore-aft).**
  Mote already treats `wheel_separation` as a first-class value in
  `robot.yaml` — the diff-drive controller reads it for odometry. The mount must
  place the contact patch at that track and on the drive axis; today that
  placement is baked into hand-tuned mount geometry rather than derived from the
  declared value.
- **Axle-axis orientation.** The wheel axis must be horizontal and square to the
  drive direction — no camber, no toe. A reusable mount has to pin the axle as a
  line at a fixed height and angle, not just a bolt cluster.
- **Sensor origins.** The lidar scan frame and camera optical frame are TF nodes
  the whole stack depends on (and the scan frame is already known to sit yawed
  90° from `base` — a measured constant, not a chosen one). Reusability means a
  sensor mount pins its sensor's origin at a *known offset from a plate datum*,
  so the URDF numbers become a property of the mount, not something re-measured
  per robot.

**Why a hole grid can't carry this, and what can.** A grid pins a part *to the
plate*; it does not pin a feature *to its function*. Bridging the two needs two
halves working together — which is exactly the "locate off datums, let fasteners
only clamp" separation that [REDESIGN.md](../REDESIGN.md) already argues for:

1. A **part-local datum**: the mount defines its functional feature (the axle
   centreline) at a specified offset from a *registration feature* — a locating
   boss or pin into a designated grid hole treated as the mount's origin — not
   from the averaged position of a bolt cluster. The screws then only clamp; the
   boss locates. Get this and the wheel sits right by construction.
2. A **plate-frame convention**: the plate declares an origin and axes (ORP's
   grid already implies a centre-origin lattice), so a mount registered at grid
   coordinate (i, j) lands its axle line at a computable robot-frame position.

**The honest tension: datums are per-geometry, not universal.** Axle height is
bound to wheel radius; track is bound to chassis width. "A servo mount reusable
across all robots" is therefore not a real object — it is only reusable across
robots that share a wheel radius and want the same ride height. So the reusable
unit is not a frozen physical mount but a **mount plus a declared datum spec**
`(axle-height H, track T, axis orientation)`, such that any robot adopting that
spec gets a working drivetrain by bolting the mount on. That is a *parameterized*
interface — and Mote already has the parameter file. `robot.yaml` is the single
source of truth for `wheel_radius` and `wheel_separation` (the URDF and the
controller both read it); it is one short step from being the **datum
declaration** as well.

This points at a Mote-shaped answer that differs from the ecosystem systems.
goBILDA and TB3 chase reusability through a *frozen physical interface* shared by
thousands of purchased parts — the right move when you amortize tooling over a
catalogue. Mote's parts are printed per robot, where a different size is just a
different STL at near-zero cost. So Mote's reusability should come from
**parametric regeneration, not a frozen interface**: make the functional datums
explicit in `robot.yaml`, and have the mount CAD *reference* those values (axle
height and axle line driven by `wheel_radius` / `wheel_separation` and a declared
ride height) so the mount is regenerated correctly for any geometry, instead of
hand-fitted once and thrown away. Reusability-by-regeneration suits printed parts
exactly as reusability-by-frozen-interface suits moulded catalogues.

---

## Does Tiling Carry Over? (TB3 → printed 6 mm plate)

The tiling idea is genuinely clever, and it is worth being precise about *why*
it works for TB3 before deciding it doesn't for Mote — the reasons are specific,
not a dismissal.

**Why tiling pays off for TB3.** The plates are **injection-moulded**. Tooling is
the dominant cost, and it is fixed per part shape: a mould for one small waffle
tile, amortised across every robot, lets ROBOTIS build a chassis of *any* size
and shape by riveting tiles — with no new tooling for each size. The tiles are
also isotropic moulded ABS, so a butt seam between two tiles, pinned by a dense
rivet grid, is nearly as stiff as the parent material. Tiling converts a tooling
constraint (moulds are expensive and size-specific) into a non-constraint. That
is real value — *for injection moulding*.

**Why it does not transfer to a printed 6 mm plate.** Every premise above
inverts:

- **There is no tooling cost to amortise.** A Mote plate is an STL. A
  differently-sized robot is a differently-sized STL — near-zero marginal cost,
  produced in minutes of CAD. The *entire economic driver* of TB3 tiling
  (reuse one mould across sizes) simply does not exist for a printed part. You
  already get "arbitrary chassis size without new tooling" for free, monolithically.
- **Seams are the weak axis of an FDM part, not a neutral one.** A moulded tile
  seam is fine; a printed one is not. A butt joint between two 6 mm PLA tiles
  concentrates load exactly where FDM is weakest, and a monolithic printed plate
  is stiffer than a tiled one of equal mass. TB3's rivet-grid trick buys back the
  stiffness a moulded seam barely lost; on PLA it would be fighting layer
  adhesion to recover stiffness a monolith never gave up. This is the same
  distinction the print-native systems make by construction — Gridfinity and
  Multiboard tile *non-structural* organizers and panels, never a load path.
- **The print-bed limit is marginal for Mote today.** The stated motivation for
  tiling is decoupling plate size from bed size. Mote's plate is ~235 mm; common
  hobby beds are 220–256 mm (Bambu/Prusa-class) and larger machines exceed it
  outright. The current plate already fits. Bed-decoupling only becomes a real
  need if a future robot grows past ~250 mm — at which point tiling is one
  option and simply printing on a bigger machine is another.
- **Cheap damage replacement is already there.** "A damaged section is a cheap
  replacement" is TB3's headline tiling benefit. For a printed monolith the
  whole plate *is* the cheap replacement — reprint the one STL. Tiling would
  only win here if reprinting a *whole* plate were expensive, which for FDM it
  is not.

**Verdict: do not tile.** The transferable insight from TB3 is not the tiling
*mechanism* but the recognition that the plate is a commodity substrate whose
size should not be a design constraint — and printed per-robot STLs already
deliver that, more cheaply and without the seam penalty. Tiling stays on the
shelf as a genuine escape hatch for exactly one future scenario: a robot whose
plate outgrows the available print bed *and* which cannot be moved to a larger
machine. If that day comes, the print-native pattern to copy is Multiboard's
screw-together edge connector (a clamped, serviceable seam), not TB3's permanent
rivets — but nothing on Mote earns it now.

---

## Recommendation — ORP plus two layers, not a new standard

The concrete outcome is neither "adopt a new grid" nor "stay pure ORP and accept
the flaws." It is: **keep ORP as the grid layer, and add the two layers it was
never meant to provide** — a lightweight datum layer and the fastening layer from
[#144](fastening.md). Each is independent, and none touches ORP's Ø3.5 mm / 20 mm
through-holes, so ecosystem compatibility is preserved.

**1. Grid layer — keep ORP, unchanged.** Plates stay pure ORP (Ø3.5 / 20 mm);
mounts keep the de-facto 10 mm half-pitch ([mounting_survey.md](mounting_survey.md),
task #141). *Rejected:* a finer plate grid (task #141 — cost without benefit);
adopting goBILDA/foreign grids (buys purchased-ecosystem interchange Mote isn't
part of, at the price of the £6 M3 inventory and a full reprint).

**2. Datum layer — declare functional datums in `robot.yaml`, reference them in
CAD.** Promote `robot.yaml` from "wheel geometry + servo params" to the robot's
**datum declaration**: ride height / axle height, track (`wheel_separation`),
axle-axis orientation, and each sensor origin offset. Then drive the mount CAD
from those values and give each mount a *registration feature* (a locating boss
into its origin grid hole) so parts **locate off datums and the grid only
clamps** — the separation [REDESIGN.md](../REDESIGN.md) calls for. This is the
minimum that makes a servo mount reusable-by-regeneration instead of
hand-fitted, and it is a documentation-and-CAD change, not new hardware.

**3. Fastening layer — the trapped-nut plate.** Adopt [#144](fastening.md)
wholesale: hex-trapped nuts in the top plate (screw from above), roofed nut slots
in the mounts, nyloc on the bare servo-lug joints. This is the "which side
threads, how the nut is retained, assembly direction" that ORP omits, using the
existing £6 M3 set.

**Not adopted: plate tiling.** For a printed part its benefits (arbitrary size,
cheap damage replacement) are already free from monolithic per-robot STLs, while
its costs (seam stiffness loss on FDM, joint proliferation, tolerance stack-up
across tiles) are real — see the verdict above. It stays on the shelf as the
escape hatch if Mote ever outgrows the print bed.

Net: Mote does not author a rival to ORP. It uses ORP for what ORP is good at (a
shared hole pattern) and writes down the two layers ORP deliberately leaves to
the builder — locating parts by datum, and fastening them the same way every
time.

---

## Sources

- [ORP — design rules](https://openroboticplatform.com/designrules) · [ORP home](https://openroboticplatform.com/) · [Hackaday: Building Robots With A 20×20 Grid](https://hackaday.com/2024/02/05/building-robots-with-a-20x20-grid/)
- [fastening.md — trapped-nut plate & ORP field notes (#144 / PR #41)](fastening.md) · [mounting_survey.md — measured v1 grid (#141)](mounting_survey.md) · [REDESIGN.md — locate-vs-clamp, per-joint process](../REDESIGN.md) · [hole_survey.py — STL hole/pitch measurement tool](hole_survey.py)
- [ROBOTIS TurtleBot3 e-manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/) · [TB3 waffle plate (ROBOTIS store)](https://www.robotis.us/tb3-waffle-plate-ipl-01-8ea/) · [ROBOTIS-GIT/turtlebot3 — Waffle Plate CAD (issue #192)](https://github.com/ROBOTIS-GIT/turtlebot3/issues/192)
- [goBILDA — the pattern](https://www.gobilda.com/pattern) · [gm0 — goBILDA guide](https://gm0.org/en/latest/docs/hardware-components/kit-and-hardware-guide/gobilda.html) · [gm0 — Actobotics guide](https://gm0.org/en/latest/docs/hardware-components/kit-and-hardware-guide/actobotics.html) · [ServoCity — Actobotics](https://www.servocity.com/actobotics)
- [OpenBuilds — V-Slot 20×20 rail](https://us.openbuilds.com/v-slot-20x20-linear-rail/)
- [Gridfinity — 42 mm modular system](https://gridfinitylayouttool.com/what-is-gridfinity) · [Gridfinity vs Multiboard comparison](https://gridpilot.us/blog/gridfinity-vs-multiboard-vs-minutegrid)
