# Fastening Research: Nested Stacks, Blind-Side Joints, and the Trapped-Nut Plate

*Researched July 2026. Context: every printed part on Mote is joined the same
way — M3 button-head screws driven from the plate side into captive-nut slots
in the mounts. This doc works out where that convention actually fails, and
fixes those places rather than replacing the fastening system wholesale.*

---

## Problem Statement

First, what is **not** broken. Nuts seated in the mounts' captive slots hold
under driving vibration; the loosening recorded in
[ASSEMBLY.md](../ASSEMBLY.md) happens only where a nut is used *bare*, without
a slot restraining it. And the base plate joints are fine as a class: the
underside of the robot is its outside, so attaching mounts from below is easy
access, not a constraint. Any scheme that spends money re-solving these (e.g.
threaded inserts at ~£25) buys insurance against a failure that isn't
occurring.

The real problems, seen only after living with the robot:

1. **Nested sensor stacks share one screw axis.** A sensor joint is two
   joints — sensor→mount and mount→plate — built as one stack. The mount's nut
   slots open on its *top* face, so the nuts are held in place by the sensor
   sitting on them: the lidar is a structural nut-retainer. Consequences: the
   nuts must be seated and the sensor installed before the mount can go on the
   plate; the mount can't be handled freely once loaded; and the mount→plate
   screw is length-critical — through 6 mm of plate and 6 mm of mount,
   **exactly M3×12** reaches the nut, while anything longer presses into the
   underside of the sensor (the C1's base, or at the Pi holder, a £99 board).
2. **The top plate fastens from inside the sandwich.** "Screw from the plate
   side" points the screwdriver *into* the chassis on the top deck. Once the
   top plate sits on the standoffs, its underside is barely reachable, so
   adding or removing any top-deck part — including the SO-101 arm, the one
   component that is regularly taken on and off — means working blind under
   the plate or removing the plate entirely.
3. **Bare nuts loosen.** The servo lugs, and the interim camera-mount
   workaround (screw heads seated in the captive slots as studs, free nuts
   spun on from below the plate — which relocates the problem but leaves a
   free nut below deck: awkward to tighten and first to loosen).

Usage fact that shapes the answer: the build has stabilised. Most components
are static; only the SO-101 arm comes on and off often. So the design goal is
*every joint serviceable from outside the robot*, not universal quick-release.

---

## Constraint Audit

**The ORP 3.5 mm grid** (20 mm spacing on the plates, 10 mm on some mounts for
placement flexibility) buys interchange with
[ORP](https://openroboticplatform.com/) ecosystem parts and matches LeKiwi
convention. It is a *hole pattern*, not a fastening scheme — nothing about it
dictates where the nut lives or which side the screw enters from. Every change
below keeps the 3.5 mm through-holes intact.

**M3 screws + nuts** were chosen to match LeKiwi/ORP and buy a single £6
fastener inventory. Worth keeping — the fix below reuses exactly this
hardware.

**The plates are printed parts.** So are the mounts. Joint geometry is a
slicer decision, not a purchasing decision, and the plates can be revised
whenever the change earns a reprint.

### Field notes: ORP after one build

Mote is a first build on ORP, and three flaws are now visible from use.
Recorded here because they bound how much the standard should be deferred to,
not because any of them changes the recommendation below.

First, granularity: a 20 mm pitch suits larger robots, but optimising
component fit on a 235 mm plate wants finer placement. The mounts' de-facto
10 mm half-pitch recovers some of it, but the standard itself has no
finer-grid provision.

Second, and more fundamental: **ORP standardises a hole pattern, not an
interface.** The servo mounts had to be explicitly positioned so the wheel
sat correctly relative to the chassis — a functional constraint (axle height,
wheel-contact position) that lives entirely outside the standard. The result
is that neither the plate nor the servo mount is actually reusable on a
different robot without some additional convention; grid compatibility gave
fastener interchange, not part interchange. A useful standard for drivetrain
parts would have to specify functional datums, not just hole locations.

Third, the mounting system is underspecified: the standard says nothing about
which side threads, how nuts are retained, or assembly direction, so every
ORP design re-solves fastening ad hoc — this document included. Other ORP
components don't visibly converge on an answer (no inserts sighted). The
trapped-nut plate below is effectively a local extension of the standard's
unclaimed territory.

Prior art worth studying if the mounting convention is ever revisited:
TurtleBot 3's waffle plates — small injection-moulded lattice modules,
riveted edge-to-edge into arbitrary plate sizes and shapes, with a hole
pattern dense enough to approximate continuous placement. The tiling idea is
the interesting part: the "plate" stops being a monolith, so chassis size
decouples from bed size and a damaged section is a cheap replacement — though
notably TB3 also leaves threading unspecified (bolts + nuts for components,
rivets for plate-to-plate), so it solves granularity and extensibility, not
fastening.

---

## Recommendation

Three changes, all geometry, using hardware already in the £6 set:

### 1. Hex-pocket the underside of every top-plate hole

Add a hex counterbore (~5.6–5.8 mm across flats, calibrated by test coupon;
~2.8 mm deep) to the underside of every 3.5 mm hole in `Chassis Top`, leaving
~3.2 mm of plate above the pocket. An M3 nut pressed in (light interference
fit, dab of CA for insurance — the standard printed nut-trap, proven at scale
in printer frames) turns that hole into a fixed thread driven **from above**.

- Every top-deck part — Pi holder, camera mount, SO Base ORP — installs and
  removes entirely from outside, with an ordinary driver, one-handed. The nut
  cannot rotate (hex walls), cannot fall (press fit), and cannot loosen any
  more than the proven slotted joints do.
- The SO-101 arm needs nothing special: its four top-down screws land in four
  trapped nuts. On/off becomes four screws from above, no hands below deck.
  The camera-mount stud workaround retires.
- Screw length stops being critical on the top deck: excess thread protrudes
  harmlessly into the chassis cavity. At the Pi holder the screw path now ends
  *below* the plate, nowhere near the board — the M3×12/M3×10 hazard is
  deleted, not mitigated.
- ORP compatibility is preserved: the through-hole is unchanged, and a
  conventional ORP mount bolted with a nut below simply finds its nut seated
  in a pocket that stops it spinning — strictly better.
- Hex **every** hole, not just occupied positions: an empty pocket costs
  nothing, keeps the plate uniform, and makes every future position free.
  Tightening pulls the nut up against the pocket ceiling — compression across
  flat-printed layers, the plate's strong direction.

### 2. Roof the nut slots in the mounts

Move each mount's nut slot from an open-top pocket to a slot buried ~1.5 mm
below the mount's top face (the slot itself is unchanged; it just gains a
printed ceiling). This decouples the nested stack:

- The nut is captive in the mount *alone* — the sensor stops being a
  structural nut-retainer, and mount+sensor becomes a bench-assembled unit
  that can be flipped, handled, and stored loaded.
- An over-long mount→plate screw bottoms on plastic instead of the sensor
  base. M3×12 remains the right length; M3×14 becomes a non-event.
- The joint mechanics — which demonstrably hold under vibration — are
  untouched, and the base plate needs no changes at all.

The sensor→mount screws (factory threads in the C1 base; camera clamp) are
unchanged and were never the problem: with the mount handleable as a unit,
"sensor screws only reachable with the mount off the plate" is just bench
assembly, not an ordering constraint.

### 3. Nyloc nuts on the bare-nut joints

The servo lugs are the one place a nut must sit in free air, and the one place
loosening is actually observed. M3 nyloc (DIN 985), ~£4/100, fit-and-forget.
Highest-vibration joint on the robot; easy access at assembly step 1.

**BOM delta: ~£4.** Everything else is reprints of `Chassis Top` and the
mounts.

### Per-joint effects

Joints in ASSEMBLY.md assembly order:

| # | Joint | Current | After | Notes |
| --- | --- | --- | --- | --- |
| 1a | STS3215 servo → Motor Support | M3 + bare nuts on lugs | **Nyloc nuts** | The joint that actually loosens. |
| 1b | Wheel Inner → servo horn | Factory horn screws | Unchanged | |
| 2a | Motor Support → Chassis Base | M3×12, open-top slots | **Roofed slots**, otherwise unchanged | Drive-train joint stays fully bolted. |
| 2b | Battery Mount → Chassis Base | M3×12, open-top slots | **Roofed slots** | |
| 2c | Waveshare Mount → Chassis Base | M3×12, open-top slots | **Roofed slots** | |
| 2d | C1 Lidar Mount → Chassis Base | M3×12 *exact*, nuts trapped by lidar | **Roofed slots** | Mount+lidar becomes a bench unit; screw length tolerant. |
| — | Lidar → C1 Lidar Mount | Factory threads in C1 base | Unchanged | Never the problem. |
| 3 | Caster → Chassis Base | M3, open-top slots | **Roofed slots** | Part is provisional — carry the roofed slot into its replacement. |
| 4 | Camera → Camera Mount | Friction fit / clamp | Unchanged | |
| 5a | Pi Bottom → Chassis Top | M3×10 from below (M3×12 damages Pi) | **From above into trapped plate nuts** | Screw path ends below the plate; hazard deleted. |
| 5b | Pi board → Pi Bottom / Pi Top | Printed retention | Unchanged | |
| 5c | Camera Mount → Chassis Top | Stud workaround, free nuts below | **From above into trapped plate nuts** | Workaround retires. |
| 6 | Chassis Top → standoffs | Screws into 50 mm standoffs | Unchanged | Standoffs are absent from BOM.md — identify (metal vs printed) and add them. |
| 9 | SO Base ORP → Chassis Top | 4× top-down, free nuts under plate | **From above into trapped plate nuts** | Arm on/off = four screws from the top deck. |

Net effect on ASSEMBLY.md: the base-plate sequence is unchanged (it worked);
the top deck becomes order-free and serviceable with the chassis closed; the
one recurring operation (arm swap) needs no access below deck.

---

## Considered and Not Adopted

Surveyed while the problem was still framed as "nuts loosen" or "needs
quick-release everywhere". Kept as the playbook for when circumstances change:

- **Brass heat-set inserts** (ruthex-class M3): excellent measured performance
  (CNC Kitchen: ~181 kg pull-out; bolt shears before torque-out) and blind
  bores would also fix the screw-length hazard — but at ~£25 in parts and
  tooling they re-solve joints that aren't failing. The trapped-nut pocket
  gives the same "fixed thread in the part" behaviour for £0 using the proven
  nut. Revisit only if a pocket ever strips or nut installation proves
  irritating at scale.
- **Thread-forming screws into PLA:** ~10–20 re-assembly cycles before formed
  threads give up (Formlabs). Disqualified on a robot that gets taken apart.
- **Printed threads:** unresolvable at M3 pitch on 0.2 mm layers; viable at
  M6+ (printed thumbscrews), relevant only if a tool-free joint is ever wanted.
- **Registration pins + single clamp screw** (separating location from
  clamping): mechanically sound and still free, but it solves fastener-count
  fiddliness, which stopped being the binding problem once the build went
  static. Adopt opportunistically when a mount is being reworked anyway.
- **Dovetails, bayonets, quick-release carriers, straps, zip ties, magnets:**
  the tool-free tier. Nothing currently comes on and off often enough to earn
  the plate-side features (the arm is four screws from above, which is
  enough). If a sensor ever needs genuine hot-swapping, the tripod
  quick-release-plate pattern — component screwed to a carrier once on the
  bench, carrier clamped to the robot — is the shape to reach for.
- **Trapped studs + thumb-nuts for the arm:** the formalised version of the
  camera workaround (hex heads captive in plate pockets, shafts up as locating
  studs). Superseded by uniform nut pockets — one pocket design everywhere
  beats a special case, and studs would snag when the arm is off.
- **Threadlocker on the current nuts:** works as a zero-CAD stopgap but is
  dominated by nyloc where nuts are bare and by trapped nuts everywhere else.

---

## Implied CAD Changes

For the follow-up implementation task (Fusion 360; do not hand-edit STLs):

1. **Test coupon first.** A small plate with hex pockets at 5.5 / 5.6 / 5.7 /
   5.8 mm across flats (target: firm press fit for a DIN 934 M3 nut, no
   rotation under driving torque, no fall-out inverted) and one roofed nut
   slot. One evening of printing calibrates the two dimensions everything else
   depends on.
2. **Chassis Top:** hex counterbore, calibrated AF × ~2.8 mm deep, on the
   underside of every grid hole; 3.5 mm through-holes unchanged. Mind print
   orientation: the pocket-to-hole transition bridges over the hex — use a
   stepped/sacrificial-bridge transition so the 3.5 mm bore stays round.
3. **All mounts** (Motor Support, Battery, Waveshare, C1 Lidar, Camera
   Mounts, Pi Bottom, Caster): bury the existing nut slots ~1.5 mm below the
   top face. Slot geometry otherwise unchanged. Mounts' own hole grids
   (20 mm / 10 mm) unchanged.
4. **Chassis Base: no changes.**
5. **ASSEMBLY.md:** replace the hardware note — nut installation into the top
   plate (press in, optional CA dab) as a one-time prep step; top-deck
   convention "screw from above"; nyloc on servo lugs; note that screw length
   is no longer critical anywhere. Retire the camera-mount workaround text and
   the order-constraint language.
6. **BOM.md:** add nyloc nuts; identify and add the 50 mm standoffs (currently
   missing from the BOM).

---

## Sources

- [Bolt Science — Vibration loosening of bolted joints (Junker mechanism)](https://www.boltscience.com/pages/vibloose.htm)
- [CNC Kitchen — Threaded inserts, cheap vs expensive (pull-out/torque tests)](https://www.cnckitchen.com/blog/threaded-inserts-for-3d-prints-cheap-vs-expensive)
- [Formlabs — Threads and inserts in 3D-printed parts (re-assembly cycle estimates)](https://formlabs.com/blog/adding-screw-threads-3d-printed-parts/)
- [Open Robotic Platform (ORP) standard](https://openroboticplatform.com/)
- Precedents: press-fit nut traps in printed printer frames (Prusa-style),
  tripod quick-release plates (the carrier pattern, if ever needed).
