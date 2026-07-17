# Fastening Research: Rethinking the Screwed-Everywhere Chassis

*Researched July 2026. Context: every printed part on Mote is joined the same
way — M3 button-head screws through the chassis plates into captive-nut slots
in the mounts. This doc asks whether that uniformity is earning its keep, not
just which fastener to swap in.*

---

## Problem Statement

First, a correction to the folklore: the captive-slot joints **hold**. Nuts
seated in the mounts' slots have not been observed to loosen; the loosening
recorded in [ASSEMBLY.md](../ASSEMBLY.md) happens on joints where a nut is
used *bare*, without a slot to restrain it — the servo lugs being the main
case. So "nuts loosen" is not a reason to redesign the slotted joints, and any
scheme that spends money re-solving them (threaded inserts at ~£25) is buying
insurance against a failure that isn't occurring.

What is actually wrong, in order of day-to-day pain:

1. **Assembly-order coupling.** A slotted joint needs the nut seated and the
   screw driven from the plate side while both are accessible, so the
   ASSEMBLY.md sequence is partly dictated by fastener access rather than by
   anything the robot cares about. On a robot that is taken apart often, this
   is the tax paid at every session.
2. **Fastener count and fiddliness.** Four screws and four loose nuts per
   mount, a tool for every operation, nothing removable by hand. The battery —
   the part swapped most often — lives behind screwed joints.
3. **Bare-nut joints loosen.** Servo lugs and any joint made without a slot
   back off under drive vibration.
4. **Screw-length hazard at the Pi holder.** The slot is a through-feature, so
   an M3×12 used where M3×10 was meant passes the nut and presses into the
   underside of the Pi board.

The constraints inherited from the original design are all negotiable, and it
is worth being explicit about what each one actually buys before treating it
as load-bearing.

---

## Constraint Audit

**The ORP 3.5 mm / 20 mm grid.** Buys interchange with
[ORP](https://openroboticplatform.com/) ecosystem mounts and matches LeKiwi
convention. Cheap to keep — but note it is a *hole pattern*, not a fastening
scheme. A 3.5 mm grid hole can host a screw, a printed pin, a zip tie, a snap
rivet, or the shaft of a twist-lock. Keeping the plates ORP-compatible does
not oblige every joint to be a screwed joint.

**M3 machine screws + nuts.** Chosen to match LeKiwi/ORP, buys a single cheap
fastener inventory (the £6 set). Worth keeping *where a screw is the right
tool* — high-preload structural joints — not as a default for everything.

**Every joint is a screwed joint.** This is the real unexamined assumption.
Both sides of every joint on this robot are printed parts (the plates too —
see [BOM.md](../BOM.md): "Chassis plates (3D printed)"). Printed geometry is
free per unit; purchased fasteners cost money, assembly time, and ordering
constraints. The design space is therefore much wider than the fastener
catalogue: pins, rails, bayonets, clips, straps — anything a printer can make.

**PLA everywhere.** Also loose. Spring features (snaps, detents) fatigue in
PLA; a mount that wants a living clip can simply be printed in PETG — a
per-part slicer choice, not a project migration.

---

## Design Space

### 1. Separate location from clamping ⭐ core recommendation

The classic machine-design rule the current scheme ignores: **fasteners should
clamp, geometry should locate**. Today each mount's four screws do both jobs —
they position the part *and* hold it down — which is why there are four of
them and why every one needs a nut.

Instead: give each mount two printed registration pins (Ø ~3.3 mm, length just
under the 6 mm plate) that drop into existing ORP grid holes, and keep **one**
M3 screw + captive slot for clamp-up. The pins take all shear and set position
exactly; the screw only provides preload.

This directly attacks the vibration problem at its mechanism. Threaded joints
back off when transverse micro-slip cycles the joint (the Junker effect — see
Bolt Science below); with dowel pins carrying the shear, the remaining screw
sees almost no transverse load and has little reason to loosen. It also cuts
the fastener count per mount from four screws + four nuts to one + one,
removes three quarters of the nut-seating that drives assembly order, and
costs **nothing** — no new parts, plates completely untouched (the pins live
on the mounts and use holes the plates already have), ORP compatibility
preserved by construction.

- **Assembly-order freedom:** near-total — one screw per mount instead of
  four, and the pins mean the part self-locates while you drive it.
- **Vibration:** better than today (shear off the threads).
- **Re-assembly cycles:** pins are the wear item; at Ø3.3 in a 3.5 hole they
  are slip-fit, not press-fit, so wear is minimal. A worn mount is a reprint.
- **Cost:** filament.

**Verdict:** the default treatment for every plate↔mount joint. Everything
below is layered on top of this for specific joints.

### 2. Printed slide and twist joints

For parts that come off *often*, even one screw is friction. Printed
quick-release joints are mature tech in the printing community:

- **Dovetail / French-cleat rails:** the mount slides onto a printed rail and
  a detent or single thumbscrew stops the slide. Honeycomb storage walls,
  Multiboard, and countless tool-wall systems run on exactly this. Strong in
  every axis except the slide axis.
- **Bayonet / quarter-turn:** insert, twist ~30–90°, locked — the smoke
  detector baseplate joint, proven for years of ceiling vibration with zero
  tools. Suits round-footprint parts (the lidar mount is the obvious
  candidate).
- **Keyhole slots:** shoulder over a wide hole, slide into the narrow slot.
  Simplest possible quick-release for light parts; weak against the return
  direction, so pair with a detent.

The catch: the *plate* side needs a mating feature, so unlike option 1 these
change the plates. Two ways to keep ORP intact: (a) model the rail/bayonet
into the plate but keep the grid holes clear around it — ORP mounts still bolt
anywhere else; or (b) make a **printed adapter plate** that screws onto the
grid once (with the existing hardware) and carries the rail — zero plate
changes, and the quick-release scheme becomes itself an ORP accessory.

- **Assembly-order freedom / cycles:** excellent; tool-free.
- **Vibration:** needs a positive detent or latch, not just friction. Design
  the detent as a stiff, low-strain bump, or print the sprung part in PETG.

**Verdict:** not the default, but the right treatment for the two or three
parts that are actually removed frequently. Prototype before committing (see
CAD section).

### 3. Snap fits and clips

Cantilever clips, printed corner tabs, annular snaps. The strongest use here
is not mount-to-plate but **component retention**: the Pi board can be held by
printed corner clips exactly the way every commercial Pi case holds it — zero
screws anywhere near the board, which *deletes* the screw-length hazard rather
than mitigating it. Same pattern suits the Waveshare board.

PLA's brittleness is the standard objection; the standard answers are
low-strain geometry (long thin arms, generous root fillets — BASF's snap-fit
manual has the formulas) and printing the clipping part in PETG. Clips on an
often-opened robot should be designed to flex a fraction of their allowable
strain, or be a separate small printed part that is cheap to replace.

**Verdict:** adopt for board retention (Pi, Waveshare). Use sparingly for
structural joints.

### 4. Strap, tie, and stick

The unglamorous options, all vibration-immune because nothing threads:

- **Velcro strap for the power bank.** Every drone in existence retains its
  battery — the heaviest, most-swapped component — with a hook-and-loop strap
  through two slots. The UGREEN bank is a 160×81×27 brick; two printed slots
  in the Battery Mount and a ~£3 strap make battery swaps a five-second,
  zero-tool operation. This is precedent-backed, not exotic.
- **Zip ties through the grid.** The 3.5 mm ORP holes pass a standard 2.5 mm
  zip tie. For low-load, rarely-moved items and all cable management, a tie is
  free, rattle-proof, and any-order. Consumable by design.
- **Nylon snap rivets.** Push-in trim rivets sized for ~3.5 mm panel holes
  cost pennies and install/remove by hand. A middle ground between zip tie and
  screw for light mounts.
- **Magnets** for lids and covers (Pi Top): fine for anything that carries no
  structural load, wrong for anything that does.

**Verdict:** velcro strap for the battery is a clear win. Zip ties as the
sanctioned answer for cable-adjacent light stuff. Rivets and magnets:
situational.

### 5. Better fasteners (the conventional survey, condensed)

For completeness — the options a fastener-first framing would reach for:

- **Brass heat-set inserts** (ruthex-class M3): excellent measured performance
  (CNC Kitchen: ~181 kg pull-out, bolt shears before torque-out) and blind
  bores would fix the Pi hazard. But at ~£25 (inserts + iron tip) they
  re-solve the slotted joints — which are not failing — and still need a
  threadlocker for vibration. Wrong problem, real money. **Not recommended
  here.**
- **Thread-forming screws into PLA bosses:** ~10–20 re-assembly cycles before
  the formed threads give up (Formlabs' estimate matches community
  experience). Disqualified on a robot that is disassembled often.
- **Printed threads:** not resolvable at M3/0.5 mm pitch on 0.2 mm layers —
  but perfectly printable at **M6 and up**, which is exactly the thumbscrew
  size. A printed M6 thumbscrew + printed boss is a free tool-free clamp for
  quick-release joints (and the GoPro ecosystem proves the pattern at scale).
- **Nyloc nuts (DIN 985):** the correct fix for the joints that *actually*
  loosen — the bare-nut servo lugs. ~£4/100. Slightly taller than a plain nut,
  irrelevant on a lug joint.
- **Threadlocker (Loctite 243):** works, but on this robot it is dominated by
  nyloc (re-apply at every disassembly vs. fit-and-forget). Keep as an option
  for the drive train if it ever proves needed.

### 6. Delete the joint

The cheapest fastener is no joint. Worth a pass in the follow-up CAD task:

- Mounts that always travel together can merge into one print (Waveshare +
  Battery mounts are neighbours on the base).
- The chassis sandwich already exists — a part the full 50 mm tall can be
  located by pins and *clamped by the standoffs* when the top plate closes,
  with no fasteners of its own.
- Anything permanently attached could in principle print into the plate;
  in practice servo access argues against it for the current parts. Noted for
  future ones.

---

## Criteria Matrix

| Option | Assembly-order freedom | Vibration | Re-assembly cycles | Added cost | Plates unchanged |
| --- | --- | --- | --- | --- | --- |
| **Pins + one screw** | Near-total | Better than today (pins take shear) | High (slip-fit pins) | £0 | **Yes** |
| Slide/twist joints | Total, tool-free | Good with detent | High | £0 | Rail needed (or adapter plate) |
| Snap clips (boards) | Total | Good | Moderate in PLA, good in PETG | £0 | Yes |
| Velcro battery strap | Total, tool-free | Excellent | Unlimited | ~£3 | Yes |
| Nyloc on bare nuts | n/a (unchanged) | Good | Good | ~£4 | Yes |
| Heat-set inserts | Total | Needs threadlocker anyway | Excellent | ~£25 | Yes |
| Thread-forming screws | Total | Moderate | **~10–20 in PLA** | new screws | Yes |
| Zip ties / rivets | Total | Excellent | Consumable | pennies | Yes |

---

## Recommendation

**Stop treating this as fastener selection. Locate every mount with printed
registration pins in the existing ORP grid holes, clamp with the minimum
fastener count (usually one M3 + the captive slot that already works), retain
boards with printed clips instead of screws, strap the battery, and put nyloc
nuts on the only joints that actually loosen — the bare-nut ones.**

Alongside this, **standardise on a single screw length**. With pins carrying
location and boss heights unified in CAD, every remaining screwed joint can
take M3×10; the M3×12s leave the build entirely. The Pi screw-length hazard is
then eliminated twice over — by inventory (no long screw exists to grab) and
by geometry (the board is clipped, not screwed, so no screw path ends near
it).

BOM delta: **~£7** (nyloc + strap), versus ~£34 for the insert scheme this doc
previously recommended — and the £7 version is also mechanically better on the
axes that matter here (tool-free battery, fewer fasteners, shear off the
threads).

Interim, zero-CAD fixes usable today: nyloc nuts straight onto the servo lugs;
velcro strap around the battery mount.

### Per-joint recommendations

Joints in ASSEMBLY.md assembly order:

| # | Joint | Current | Recommended | Notes |
| --- | --- | --- | --- | --- |
| 1a | STS3215 servo → Motor Support | M3 + bare nuts on lugs | **Nyloc nuts** | The joint that actually loosens. Metal lug through-holes — a nut is unavoidable; make it a locking one. |
| 1b | Wheel Inner → servo horn | Factory horn screws | Unchanged | Threads into the metal horn. |
| 2a | Motor Support → Chassis Base | 4× M3×12 + slots | **2 pins + 2× M3×10 + slots** | Drive-train joint: keep two screws, not one. Pins take the shear that loosens things. |
| 2b | Battery Mount → Chassis Base | M3×12 + slots | **2 pins + 1× M3×10**; bank held by **velcro strap** | Battery swap becomes tool-free. Consider merging with 2c into one print. |
| 2c | Waveshare Mount → Chassis Base | M3×12 + slots | **2 pins + 1× M3×10**; board in **printed clips** | |
| 2d | C1 Lidar Mount → Chassis Base | M3×12 + slots | **2 pins + 1× M3×10**, or **bayonet** if it proves frequently-removed | Round footprint suits a quarter-turn; prototype first. |
| 3 | Caster → Chassis Base | M3 + slots | **2 pins + 2× M3×10 + slots** | Ground-contact vibration; part is provisional — carry pins into its replacement. |
| 4 | Camera → Camera Mount | Friction fit / clamp | Unchanged | No threaded joint. |
| 5a | Pi Bottom → Chassis Top | M3×10 + slot (M3×12 damages Pi) | **2 pins + 1× M3×10**, screw boss placed **outside the board footprint** | With single-length inventory and clipped board (5b), the hazard is gone by design. |
| 5b | Pi board → Pi Bottom / Pi Top | Printed retention + screws nearby | **Printed corner clips** (commercial-case style) | Zero screws near the board. Clip arms in PETG if PLA fatigues. |
| 5c | Camera Mount → Chassis Top | M3×12 + slot | **2 pins + 1× M3×10** | |
| 6 | Chassis Top → standoffs | Screws into 50 mm standoffs | Unchanged | The standoffs are absent from BOM.md — identify (metal vs printed) and add them either way. |
| — | Lidar → C1 Lidar Mount | Factory base holes | Unchanged | C1 base has its own threads. |
| 9 | SO Base ORP → Chassis Top | M3 + slots | **Keep pure ORP screw pattern** | This part exists to be the standard interface — don't get clever with it. |

With pins locating everything and one clamp screw per mount, ASSEMBLY.md's
"suggested order" becomes genuinely suggested: any mount attaches or detaches
independently with one screw, and the battery with none.

---

## BOM Delta

Additions to [BOM.md](../BOM.md) (the existing M3 set stays; M3×12 drops out
of use; its plain nuts remain spares):

| Part | Qty | Unit price | Notes |
| --- | --- | --- | --- |
| M3 nyloc nuts, DIN 985 (100 pcs) | 1 | ~£4 | Servo lugs and any future bare-nut joint |
| 25 mm hook-and-loop strap (2-pack) | 1 | ~£3 | Battery retention, drone-style |

**Core delta: ~£7.** Everything else in the recommendation is filament.

---

## Implied CAD Changes

For the follow-up implementation task (Fusion 360; do not hand-edit STLs):

1. **Print a test coupon first.** One small plate carrying: a pin-fit array
   (Ø 3.2 / 3.3 / 3.4 against a 3.5 mm hole) to calibrate the slip fit, one
   dovetail pair, one bayonet pair, one Pi-corner clip. An evening of printing
   de-risks every decision below before any real part changes.
2. **Every mount:** add two Ø ~3.3 × 5.5 mm registration pins on the mating
   face, positioned on the ORP grid; delete all but one (drive train: two)
   captive-nut slot + screw hole. Chamfer pin tips for blind engagement.
3. **Chassis Base / Chassis Top: no changes.** Pins use existing grid holes;
   ORP compatibility is preserved by construction.
4. **Battery Mount:** two strap slots (~27 × 4 mm) for a 25 mm velcro strap;
   consider merging with the Waveshare Mount into one print.
5. **Pi Bottom / Pi Top:** corner clips sized to the Pi 5 board (clip over the
   board edge, clear of connectors); relocate the single clamp-screw boss
   outside the board footprint. Print clip-bearing part in PETG if coupon
   testing shows PLA arms fatiguing.
6. **Unify stack heights** so every screwed joint takes M3×10; update
   ASSEMBLY.md's hardware note accordingly (single screw length, nyloc on
   servo lugs, strap for the battery, pins-then-screw order per mount).
7. **Standoffs:** identify the current 50 mm standoffs (absent from BOM.md)
   and add them to the BOM either way.

Open questions for review: which mounts are removed often enough to deserve a
tool-free (bayonet/dovetail) joint rather than one screw; whether the lidar
mount's cable makes quick-release moot; PETG vs PLA for clip-bearing parts.

---

## Sources

- [Bolt Science — Vibration loosening of bolted joints (Junker mechanism)](https://www.boltscience.com/pages/vibloose.htm)
- [BASF — Snap-Fit Design Manual](https://web.mit.edu/2.75/resources/random/Snap-Fit%20Design%20Manual.pdf)
- [CNC Kitchen — Threaded inserts, cheap vs expensive (pull-out/torque tests)](https://www.cnckitchen.com/blog/threaded-inserts-for-3d-prints-cheap-vs-expensive)
- [Formlabs — Threads and inserts in 3D-printed parts (re-assembly cycle estimates)](https://formlabs.com/blog/adding-screw-threads-3d-printed-parts/)
- [Open Robotic Platform (ORP) standard](https://openroboticplatform.com/)
- Precedents: smoke-detector bayonet baseplates (quarter-turn under ceiling
  vibration), drone battery hook-and-loop straps, GoPro printed-thumbscrew
  mount ecosystem, honeycomb-storage-wall dovetail systems.
