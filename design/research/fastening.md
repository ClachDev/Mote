# Fastening Research: Alternatives to Hex Screws + Captive Nuts

*Researched July 2026. Context: all printed parts are joined with M3 button-head
screws through the chassis plates into captive-nut pockets in the mounts.
[ASSEMBLY.md](../ASSEMBLY.md) already flags that nuts loosen from driving
vibration; the pockets also force an assembly order (the nut side must be
accessible when the joint is made), and the Pi holder joint can damage the Pi
board when M3×12 is used where M3×10 was meant.*

---

## Problem Statement

The current scheme: plates (`Chassis Base`, `Chassis Top`, ~6 mm) carry plain
3.5 mm through-holes on the ORP 20 mm grid; each mount (Motor Support, Pi
Bottom/Top, Waveshare Mount, C1 Lidar Mount, Battery Mount, Camera Mount) has a
hex pocket holding an M3 nut captive while the screw is driven from the plate
side. Three problems:

1. **Vibration loosening.** Plain nuts back off under drive vibration
   (documented in ASSEMBLY.md's hardware note).
2. **Assembly-order coupling.** A captive nut must be seated (and its pocket
   face accessible) when the joint is made, so parts can't be attached or
   removed independently — the ASSEMBLY.md sequence exists partly to satisfy
   this.
3. **Screw-length hazard at the Pi holder.** The pocket is a through-feature,
   so an over-long screw (M3×12 instead of M3×10) passes the nut and presses
   into the underside of the Pi board.

Constraints: parts are PLA; the robot is disassembled often, so re-assembly
cycles matter; and the plates' 3.5 mm ORP grid should stay unchanged so the
chassis remains ORP-compatible (strong preference — any change should live in
the mounts, not the plates).

---

## Option Survey

### Brass heat-set inserts ⭐ Recommended

A knurled brass bushing with an internal M3 thread, melted into a printed bore
with a soldering iron (print temperature + 10–20 °C). The plastic reflows into
the knurling, giving metal threads permanently embedded in the part.

**Measured performance** (CNC Kitchen pull-out/torque-out tests, M3 in printed
plastic): quality inserts (ruthex) held **~181 kg pull-out**; torque-out
exceeded 3–4 Nm — the M3 bolt head sheared before the insert moved. Both are
far beyond anything an M3 joint on this robot sees. Cheap unbranded inserts
tested ~4× weaker in pull-out and lack the lead-in chamfer that makes
installation self-aligning, so buy branded (ruthex or CNC Kitchen's own).

- **Assembly-order freedom:** total. The insert is installed once at part-prep
  time; thereafter every joint is screw-from-the-plate-side into a fixed
  thread. Any part attaches/detaches independently, in any order.
- **Vibration:** the *thread* is metal-metal, same as a nut — so a screw can
  still back off. Pair with medium threadlocker (Loctite 243, serviceable) on
  drive-train joints, or external-tooth washers on joints opened frequently.
- **Re-assembly cycles:** excellent — the brass thread doesn't wear on the
  timescale of this robot's life. This is the deciding criterion.
- **Tooling/cost:** a soldering iron (already owned for the IMU) plus a ~£10
  insert-tip set; inserts ~£13/100. ~30 s per insert to install.
- **Plates unchanged:** yes. The 3.5 mm ORP holes are untouched; the only CAD
  change is in the mounts, where the hex pocket becomes a round bore.
- **Bonus:** installed in a **blind** bore, the insert physically caps the
  screw path — an over-long screw bottoms in brass instead of reaching the Pi
  board. The Pi-holder hazard disappears as a side effect.

**Verdict:** the right scheme for every plate↔mount joint. Solves problems 2
and 3 outright and problem 1 in combination with threadlocker.

### Thread-forming (self-tapping) screws into printed bosses

Screws for plastics (Delta-PT style, or plain self-tappers) driven into a
~2.7 mm printed pilot hole. Zero added parts, and CNC Kitchen measured direct
screwing into plastic surprisingly strong on first assembly (~142 kg pull-out).

- **Assembly-order freedom:** total (same as inserts).
- **Vibration:** moderate — the formed plastic thread has some inherent
  friction, but PLA creeps under sustained load and the joint relaxes.
- **Re-assembly cycles:** the killer. Expect roughly **10–20 cycles in PLA**
  before the formed threads wear or cross-thread; torque-out is limited by
  plastic failure (~1 Nm — easy to strip with a normal hex driver).
- **Tooling/cost:** none, but a second screw inventory (plastic-thread screws
  are not the machine screws in the current set).

**Verdict:** fine for build-once products; wrong for a robot that is
disassembled often. Not recommended.

### Printed threads

Modelling the M3 thread into the part. At M3 (0.5 mm pitch) an FDM printer at
0.2 mm layers cannot resolve the thread form — printed threads only become
usable around M6 and above.

**Verdict:** not viable at this screw size. No.

### Nyloc nuts / threadlocker on the existing scheme

Two incremental patches to the current design:

- **Nyloc (DIN 985) in the pockets:** the nylon collar resists back-off well.
  But a nyloc is ~1.6 mm taller than a plain M3 nut, so the pockets need
  re-cutting anyway (a CAD change of similar scope to insert bores, for a
  worse result), the assembly-order problem is untouched, and the collar wears
  with repeated cycles.
- **Loctite 243 (medium) on the existing nuts:** zero CAD change, ~£7, cures
  the loosening today. But it must be re-applied at every disassembly and does
  nothing for assembly order or the Pi hazard.

**Verdict:** threadlocker is the correct *interim* measure while the insert
revision is printed (and remains part of the final scheme on drive-train
joints). Nyloc is dominated by inserts — same CAD effort, fewer benefits — and
is only worth keeping for lug joints where a nut is unavoidable (servo tabs).

### Standoffs

Metal spacers with male/female threads. Already precedented on this robot: the
chassis plates are separated by the 50 mm standoffs. For board mounting the
relevant kit is **M2.5 brass Pi standoffs** (the Pi 5's mounting holes are
2.7 mm — M2.5, not M3).

- For the **Pi specifically**: seating the Pi on four M2.5 male–female
  standoffs threaded into the holder (or retained with M2.5 nuts) makes the
  screw geometry self-limiting — there is no screw that *can* reach the board,
  because board screws thread into the standoff body. Also improves airflow
  under the board.
- As a general plate↔mount scheme, standoffs don't fit — the mounts sit flush
  on the plates by design.

**Verdict:** adopt for the Pi board↔holder interface if/when `Pi Bottom` is
revised; not a general answer. Note the blind-bore insert already removes the
damage risk at the holder↔plate joint, so this is an optional refinement, not
a prerequisite.

### Combinations

The schemes compose cleanly, and the recommendation below is a combination:
inserts for all plate↔mount joints, threadlocker on the vibration-critical
subset, nyloc on the one lug joint that genuinely needs a nut, M2.5 standoffs
at the Pi board.

---

## Criteria Matrix

| Option | Assembly-order freedom | Vibration resistance | Re-assembly cycles | Tooling / added cost | Plates unchanged |
| --- | --- | --- | --- | --- | --- |
| **Heat-set inserts** | Full — any part independently | Good (with threadlocker on drive joints) | Excellent (brass threads) | Iron tip ~£10 + inserts ~£13 | **Yes** |
| Thread-forming screws | Full | Moderate (PLA creep) | Poor: ~10–20 in PLA | New screw inventory | Yes |
| Printed threads | Full | Poor | Very poor | None | Yes (but not viable at M3) |
| Nyloc nuts | None (unchanged) | Good | Good (collar wears slowly) | ~£4 | Yes (pockets re-cut in mounts) |
| Threadlocker on current | None (unchanged) | Good | Re-apply every cycle | ~£7 | Yes (no CAD change at all) |
| Standoffs (Pi) | Good | Good | Excellent | ~£9 kit | Yes |

---

## Recommendation

**Replace every captive-nut pocket with a blind heat-set insert bore
(ruthex-class M3×5.7 brass inserts), keep the plates and their 3.5 mm ORP grid
exactly as they are, and screw from the plate side as today.** Add Loctite 243
to the drive-train joints (Motor Support, Caster), use nyloc nuts on the servo
mounting lugs, and (optionally, when `Pi Bottom` is next revised) seat the Pi
on M2.5 standoffs.

Rationale: inserts are the only option that fixes all three problems at once —
independent attach/detach of every part (pockets no longer dictate the
ASSEMBLY.md sequence), durable threads for a frequently-disassembled robot, and
blind bores that make an over-long screw at the Pi holder mechanically
harmless. The measured strength margin is enormous (bolt shears before the
insert lets go), the cost is ~£25 in consumables plus a soldering-iron tip, and
the change is invisible from the plate side, preserving ORP interchange.

Until the revised mounts are printed: a drop of Loctite 243 on the existing
nuts stops the loosening with zero other changes.

### Per-joint recommendations

Joints in ASSEMBLY.md assembly order:

| # | Joint | Current fastening | Recommended | Screw | Notes |
| --- | --- | --- | --- | --- | --- |
| 1a | STS3215 servo → Motor Support | M3 screws + nuts on servo lugs | Keep screws; swap plain nuts → **nyloc** | as today | Lugs are through-holes on the servo body — a nut is unavoidable here, and it's the highest-vibration joint. Access is easy at step 1, so nyloc costs nothing in order-freedom. |
| 1b | Wheel Inner → servo horn | Factory horn screws | Unchanged | factory | Threads into the metal horn. |
| 2a | Motor Support → Chassis Base | M3×12 + captive nut | **Insert** in Motor Support + **Loctite 243** | M3×10 | Drive-train joint; threadlocker mandatory. |
| 2b | Battery Mount → Chassis Base | M3×12 + captive nut | **Insert** in Battery Mount | M3×10 | |
| 2c | Waveshare Mount → Chassis Base | M3×12 + captive nut | **Insert** in Waveshare Mount | M3×10 | |
| 2d | C1 Lidar Mount → Chassis Base | M3×12 + captive nut | **Insert** in C1 Lidar Mount | M3×10 | |
| 3 | Caster → Chassis Base | M3 + captive nut | **Insert** in Caster + **Loctite 243** | M3×10 | Ground-contact vibration. Part is provisional — carry the insert bore into whatever replaces it. |
| 4 | Camera → Camera Mount | Friction fit / clamp | Unchanged | — | No threaded joint. |
| 5a | Pi Bottom → Chassis Top | M3×10 + captive nut (M3×12 damages Pi) | **Blind insert** in Pi Bottom | M3×10 | The blind bore caps the screw path — an M3×12 bottoms in brass, never reaching the board. This is the Pi-holder fix. |
| 5b | Pi board → Pi Bottom / Pi Top | Printed retention | Unchanged now; **M2.5 standoffs** when Pi Bottom is revised | M2.5 | Pi 5 holes are M2.5. Standoff-mounted board also improves under-board airflow. |
| 5c | Camera Mount → Chassis Top | M3×12 + captive nut | **Insert** in Camera Mount | M3×10 | |
| 6 | Chassis Top → standoffs | Screws into 50 mm standoffs | Unchanged if standoffs are metal female-thread; if printed with nut pockets, convert both ends to **inserts** | as today | The BOM doesn't list the standoffs — confirm which they are and add them to the BOM either way. |
| — | Lidar → C1 Lidar Mount | Factory base holes | Unchanged | factory | C1 base has its own mounting threads. |
| 9 | SO Base ORP → Chassis Top | M3 + captive nut | **Insert** in SO Base ORP | M3×10 | Optional part; same treatment. |

With inserts in place, ASSEMBLY.md's "suggested order" becomes genuinely
suggested — any mount can be added or removed later without opening the
chassis sandwich.

---

## BOM Delta

Additions to [BOM.md](../BOM.md) (the existing M3 screw set stays; its nuts
become spares/servo-lug stock):

| Part | Qty | Unit price | Link | Notes |
| --- | --- | --- | --- | --- |
| ruthex M3×5.7 threaded inserts (100 pcs, RX-M3x5.7) | 1 | ~£13 | [Amazon UK](https://www.amazon.co.uk/dp/B08BCRZZS3) | ~20 used per build; buy branded — cheap inserts test ~4× weaker and lack the alignment chamfer |
| Heat-set insert soldering-iron tip set (M2–M8) | 1 | ~£10 | [Amazon UK](https://www.amazon.co.uk/dp/B0BNTCK2PQ) | One-time tooling; fits 900M/T18-style irons |
| Loctite 243 threadlocker (10 ml) | 1 | ~£7 | widely available | Drive-train joints; also the zero-CAD interim fix |
| M3 nyloc nuts, DIN 985 (100 pcs) | 1 | ~£4 | widely available | Servo lugs only |
| Geekworm M2.5 brass standoff kit (optional) | 1 | ~£9 | [Amazon UK](https://www.amazon.co.uk/dp/B07MN2GY6Y) | Only if/when Pi Bottom is revised for standoff mounting |

**Core delta: ~£34** (~£25 without the optional standoff kit). Per-build
consumable cost after tooling: well under £5.

---

## Implied CAD Changes

For the follow-up implementation task (Fusion 360; do not hand-edit STLs):

1. **Every hex-nut pocket → blind insert bore**, cut from the mating (plate)
   face: **Ø 4.0 mm × 6.0 mm deep** (ruthex spec hole for the 5.7 mm insert,
   +0.3 mm seating margin). Bore stays blind — do not punch through. Keep
   ≥ 1.6 mm wall (≥ 2 perimeters) around the bore: boss OD ≥ 8 mm. A small
   entry chamfer (0.5 mm × 45°) helps the insert start square. Affected parts:
   Motor Support, Battery Mount, Waveshare Mount, C1 Lidar Mount, Camera
   Mount, Pi Bottom, Caster, SO Base ORP.
2. **Pi Bottom:** bores must be blind *toward the Pi cavity* so no screw
   length can reach the board; with the 6 mm plate + insert stack, spec M3×10
   everywhere and note it in ASSEMBLY.md (M3×12 becomes harmless but M3×10 is
   correct).
3. **Chassis Base / Chassis Top: no changes.** The 3.5 mm ORP grid holes are
   untouched; ORP compatibility is preserved by construction.
4. **Standoffs:** identify the current 50 mm standoffs (they are absent from
   the BOM); if printed with nut pockets, apply the same insert bore at both
   ends, and add whichever part they are to the BOM.
5. **ASSEMBLY.md follow-ups:** replace the captive-nut hardware note with
   insert installation instructions (iron at print temp + 10–20 °C, press
   flush and square), drop the order constraint language, add the M3×10
   default + Loctite-on-drive-joints rule.

Model the bores in CAD rather than drilling prints — printed bores give the
knurling clean material to reflow into.

---

## Sources

- [CNC Kitchen — Threaded inserts, cheap vs expensive (pull-out/torque tests)](https://www.cnckitchen.com/blog/threaded-inserts-for-3d-prints-cheap-vs-expensive)
- [CNC Kitchen — Tips & tricks for heat-set inserts](https://www.cnckitchen.com/blog/tipps-amp-tricks-fr-gewindeeinstze-im-3d-druck-3awey)
- [ruthex RX-M3x5.7 insert spec (hole Ø 4.0 mm)](https://www.ruthex.de/en/collections/gewindeeinsatze/m3)
- [ruthex M3 inserts on Amazon UK](https://www.amazon.co.uk/dp/B08BCRZZS3)
- [Heat-set insert iron tips on Amazon UK](https://www.amazon.co.uk/dp/B0BNTCK2PQ)
- [Geekworm M2.5 Pi standoff kit on Amazon UK](https://www.amazon.co.uk/dp/B07MN2GY6Y)
- [Formlabs — Threads and inserts in 3D-printed parts (re-assembly cycle estimates)](https://formlabs.com/blog/adding-screw-threads-3d-printed-parts/)
- [Hackaday — Threading 3D printed parts with heat-set inserts](https://hackaday.com/2019/02/28/threading-3d-printed-parts-how-to-use-heat-set-inserts/)
- [Open Robotic Platform (ORP) standard](https://openroboticplatform.com/)
