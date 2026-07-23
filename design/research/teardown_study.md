# Teardown Study: Design Rules from Three Exemplars

*Researched July 2026 — the second Phase 1 artifact of the
[v2 redesign process](../REDESIGN.md). The goal is not admiration but
extraction: for each exemplar, the reproducible **rules** it embodies, so
Phase 2 can distil them into `design/DESIGN-RULES.md`. Where a claim can be
measured it is measured (`hole_survey.py` re-pointed at the exemplar CAD),
not recalled.*

Three exemplars, chosen to bracket Mote's problem:

- **SO-101 arm** — an open-source FDM part made by strangers on unknown
  printers. This is the destination: Mote v1 is our SO-100, and the SO-101 is
  what a second iteration under design rules looks like.
- **TurtleBot 3 waffle plate** — an *injection-moulded* modular chassis, for
  the tiling idea and as a foil: it solves granularity and extensibility but
  gets its repeatability from tooling Mote doesn't have.
- **LEGO brick** — the injection-moulded precision standard, for contrast.
  Chosen deliberately: it is the part SO-101 prints a gauge *against*.

The through-line: **injection moulding buys dimensional repeatability from the
mould; FDM-by-strangers cannot, and must design around that.** Every rule
below is really an answer to "what do you do when you can't trust the
tolerance of the machine that makes your part?"

---

## Exemplar 1 — SO-101 arm (FDM, made by others)

Source: [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)
(`STL/SO101/`, `README.md`, `STL/Gauges/`). Measured with `hole_survey.py`
against the individual-part STLs.

The SO-101 is the same class of object as Mote: printed structural plastic,
STS3215 servos, M3 hardware, built by people the designer will never meet. The
rules it embodies are legible in the CAD and stated in its print guide.

**R1 — One fixed print orientation per part, chosen first.** The follower's
parts ship "in a single file, correctly orientated for z upwards to minimise
supports" (README, Step 4). Orientation is not left to the builder's slicer;
it is a design decision baked into the delivered geometry. Features are then
drawn *for* that orientation — the reverse of Mote v1, where parts were drawn
and orientation left implicit.

**R2 — Holes that fight the orientation are bridged, not supported.** The
print guide is explicit: "There should be no supports in the screw holes with
horizontal axes" (README, Step 2.7). A horizontal-axis hole in a z-up print
overhangs; SO-101 draws those as bridged/teardrop bores that self-support. The
measurement confirms horizontal-axis holes are pervasive — every structural
part carries Ø-class holes under the x and y scans (e.g. `Base_SO101`,
`Upper_arm_SO101`, `Under_arm_SO101`), i.e. the arm routinely runs holes
across the print axis and has designed them to print dry.

**R3 — The support policy is a spec, not a vibe.** "Supports everywhere but
ignore slopes greater than 45° to the horizontal" plus R2. The 45° rule is the
standard FDM overhang limit; stating it turns "print without heroic supports"
into a checkable constraint the geometry must satisfy.

**R4 — Print a calibration coupon before the real parts.** SO-101 ships two
gauges (`STL/Gauges/`) and requires printing one *first* (README, Step 3): a
**Lego-block gauge** (measured 27 × 26 × 10 mm) to check general dimensional
accuracy against a known-standard part, and an **STS3215 servo gauge**
(measured 51 × 35 × 10 mm, with an M3 nut pocket at ~5.8 mm across-flats) to
check the fit that actually matters. "If the fit is appropriate, go on;
otherwise change your printer settings and try again." This *is* the
tolerance-variance answer: the builder tunes the unknown printer against a
coupon before committing filament. It is exactly the "calibration coupon the
builder prints first" that Mote v2's requirements demand — already proven in
the field on this arm.

**R5 — Fasteners are only what ships with the components.** The build needs
nothing but a Phillips driver (README BOM): the screws come in the STS3215
bags, and joints capture ordinary M3 nuts in printed pockets. `hole_survey.py`
finds M3-hex nut pockets throughout (Base, motor holders, arms, wrist), and no
heat-set inserts or specialty hardware anywhere. No new BOM line for fastening.

**R6 — Holes are placed *functionally*, not on a grid.** Unlike Mote's ORP
plates, SO-101 has no global hole lattice. Nearest-neighbour pitches differ
part to part and axis to axis (e.g. `Base_SO101` ~7–11 mm on x, ~18/42 mm on
y; `Upper_arm` a run at 10 mm; `Rotation_Pitch` ~10/19 mm), and nut pockets
appear on all three axes — each joint's holes sit where that joint needs them
and face the direction that joint loads. Location is a per-joint decision, and
the servo-gauge coupon (R4) is how the *interface* to the STS3215 is held
constant even though the hole pattern is bespoke. (Contrast Mote v1's finding
that an ORP grid standardises holes but *not* the interface — see
[fastening.md](fastening.md) field notes.)

**Physical-arm pass (operator-in-the-loop).** The rules above are CAD- and
doc-derived; the physical arm is the check on them. Confirmations to fold in
from the built follower — see the open question at the end of this document:
whether the gauge-first workflow was actually used and caught anything;
whether any horizontal-axis hole needed manual support despite R2; and which
of the arm's own joints are opened often in use (feeds the serviceability
class, and the SO-Base ↔ Chassis Top arm-swap joint in the schedule).

---

## Exemplar 2 — TurtleBot 3 waffle plate (injection moulded, modular)

Sources: ROBOTIS product data
([TB3 Waffle Plate IPL-01](https://www.robotis.us/tb3-waffle-plate-ipl-01-8ea/))
and the published mesh
(`turtlebot3_description/meshes/bases/waffle_pi_base.stl`), surveyed with
`hole_survey.py` and cross-sectioned for the plate grid.

The waffle plate is the tiling idea Mote's fastening survey flagged as prior
art. Torn down for rules:

**T1 — The plate is tileable, so chassis size decouples from bed/mould size.**
The system ships as 8 identical injection-moulded plates that assemble "in
various directions" into arbitrary chassis shapes; a damaged section is a
cheap single-tile replacement. Mote's monolithic 235 mm plates are bed-limited
and a crack condemns the whole plate — the opposite trade.

**T2 — A dense, multi-directional hole field, not one uniform grid.** ROBOTIS
describes "diverse holes for bolts and nuts": the plate takes M3 device bolts
on its faces *and* 6 mm frames/rivets on its edges. The published mesh shows a
hole field far denser than ORP's 20 mm (round holes on the order of Ø4 mm with
sub-15 mm spacing), approximating continuous placement — but it is a *visual*
mesh, decimated, so treat the pitch as indicative; the exact grid lives in
ROBOTIS's Onshape source. The rule that transfers is the intent: **dense
enough that placement is effectively continuous**, achieved with a fine grid
rather than Mote's coarse-grid-plus-half-pitch-mounts workaround.

**T3 — Tolerance comes from the tool, so there is no builder calibration.**
Every plate is identical to within moulding tolerance; the builder never tunes
anything. This is the capability Mote-built-by-others explicitly lacks, and is
why T2's fine grid is cheap for TB3 (mould once) but expensive for FDM (every
hole is print-time and every plate is a different printer's tolerance).

**T4 — Threading is left unspecified — the same gap as ORP.** TB3 standardises
the hole field and the plate-to-plate rivet, but component fastening is just
"bolts + nuts", retention unaddressed. Tiling and granularity are solved;
fastening is not. Mote inherits this gap from ORP and must close it itself
(the whole point of the joint schedule).

Uses M3 throughout (kit includes PHS M3×8 bolts + M3 nuts), so the fastener
inventory is compatible with Mote/ORP — the divergence is manufacturing
process and modularity, not hardware.

---

## Exemplar 3 — LEGO brick (injection moulded, the precision standard)

For contrast, the extreme of the injection-moulded end — and, not by accident,
the part SO-101 calibrates against (Exemplar 1, R4).

- **Tolerance is a property of the steel tool, ~±0.01 mm (≈10 µm), repeatable
  across billions of units.** The 8.0 mm stud pitch and the stud-and-tube
  clutch fit are held to a precision no FDM printer approaches. Nothing about
  the part is tuned per unit — the mould *is* the calibration.
- **Design rules that follow from the process:** near-uniform ~1.2–1.5 mm wall
  thickness (even cooling), draft on every vertical face (release from the
  mould), and location + retention combined in one press-fit feature (the stud
  clutch does both).
- **The pivot for Mote:** a consumer plastic product **never ships a
  calibration coupon**, because the tool guarantees the fit. FDM-by-strangers
  is the mirror image — tolerance varies per printer and per filament — so the
  repeatability LEGO gets for free must be *bought back* on Mote's side with a
  builder coupon (R4) and with fits that declare a tolerance class instead of
  assuming one. SO-101 gauging its printer against a LEGO block is this exact
  logic in miniature: borrow the injection-moulded standard to calibrate the
  additive machine.

(Uses the emblematic consumer-plastic joints Mote's fastening survey already
noted plastics prefer — press fits and snap features, no loose nuts. Those
belong in the Phase 2 fastener policy as the "considered" tier, gated by FDM's
layer-adhesion and tolerance limits.)

---

## Distilled rules → what transfers to Mote v2

| Rule (source exemplar) | Transfers to Mote v2 (FDM, built by others)? |
| --- | --- |
| Fix one print orientation per part, first; design features for it (SO-101 R1) | **Yes — core rule.** Mote v1 left orientation implicit; v2 declares it per part. |
| Bridge/teardrop holes that fight the orientation; no support in them (SO-101 R2) | **Yes.** Directly fixes v1's horizontal-axis and blind-hole pains. |
| State the support policy as a checkable spec (≤45°, none in cross-axis holes) (SO-101 R3) | **Yes.** Makes "no heroic supports" testable. |
| Ship a calibration coupon; builder tunes the printer before the real parts (SO-101 R4 / LEGO contrast) | **Yes — the answer to tolerance variance.** v2 requires it; SO-101 proves it works. |
| Fasteners only what ships with the components; no inserts (SO-101 R5) | **Yes.** Matches v2's "prefer shipped hardware" requirement. |
| Locate off functional datums per joint; hold interfaces with a gauge, not a grid (SO-101 R6) | **Yes.** Answers v1's "grid gives holes, not an interface" finding. |
| Dense grid for near-continuous placement (TB3 T2) | **Partly.** The *intent* transfers; the fine grid itself is cheap only under moulding — Mote keeps coarse ORP + half-pitch mounts (see mounting_survey.md). |
| Tileable plates decouple chassis size from bed size (TB3 T1) | **Open — Phase 3.** Attractive for the 235 mm bed limit; weigh against a single structural tray. |
| Tolerance/repeatability from the tool; no per-unit calibration (TB3 T3 / LEGO) | **No — inverted.** FDM-by-strangers has no tool; this is *why* the coupon rule exists. |
| Combine location + retention in one moulded press feature (LEGO) | **Cautiously.** Press fits work on FDM only with a declared tolerance class and layer-aware load paths. |
| Leave fastening/threading unspecified (TB3 T4 / ORP) | **No — the anti-rule.** The gap Mote's joint schedule exists to close. |

The spine of `DESIGN-RULES.md` (Phase 2) is the top block: orientation-first,
bridged holes, a stated support policy, a builder calibration coupon,
shipped-hardware fastening, and datum-located joints. Everything the
injection-moulded exemplars get from their tooling, Mote has to earn in the
design — and the SO-101 shows it is earnable.

---

## Open question for the operator (physical SO-101 pass)

The SO-101 rules above are extracted from CAD and the print guide. The
physical-arm confirmations (R4 gauge workflow, R2 support behaviour, and the
arm's own service-frequency joints) need the built follower in hand — recorded
here so the teardown is honest about what is measured vs. stated. See the
task's operator question.
