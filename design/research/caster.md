# Caster Design Research

*Researched July 2026. Context: the printed hemisphere caster ([`Caster.step`](../step/Caster.step)) has zero tolerance for uneven floors — slight slopes strand the robot and rugs are a no-go ([issue #3](https://github.com/ClachDev/Mote/issues/3)). The central constraint: any added compliance must not introduce wobble, because lidar scan-plane tilt degrades SLAM/localisation.*

---

## Problem Statement

Mote stands on two centred drive wheels plus caster support both fore and aft of
the axle. That makes **four ground contacts for a rigid body — statically
over-constrained**. On any floor that isn't perfectly coplanar with the build
(and given FDM print tolerance, even a flat floor isn't safe), the chassis rocks
fore/aft between the casters and the **drive wheels unload → no traction →
stuck**. This is the observed failure; slopes and rug edges just make it worse.

Three things follow, and they frame every option below:

1. **Caster *shape* cannot fix the stance.** A better hemisphere, a bought
   nylon ball, a wheel — any *rigid* third/fourth contact leaves the statics
   unchanged. This matches testing: both printed and purchased ball casters
   were tried with no real improvement.
2. **A 3-point stance is off the table.** Turn-on-the-spot requires the centred
   axle, and the CoM straddles it: slightly behind for the base robot, moving
   in front when the SO-101 arm is fitted. Support is genuinely needed both
   fore and aft, and the CoM can't be shifted (the power bank is already
   crammed against the lidar between the servos).
3. **The remaining degree of freedom is vertical compliance at the casters** —
   a few mm of sprung travel so the wheels stay loaded and contact-height error
   stops mattering. The question this doc answers is whether that can be done
   *without* trading traction problems for lidar wobble.

### Geometry and height budget

| Quantity | Value | Source |
|---|---|---|
| Drive wheel | Ø65 mm (r = 32.5 mm) | `robot.yaml` |
| Caster positions | ≈ ±100 mm from the axle | URDF (`caster_x`) |
| Ride height (base-plate underside → floor) | ~16 mm | measured on robot |
| Current printed caster | Ø30 hemisphere, ~14 mm tall | `Caster.step` |
| Lidar scan plane | ~54 mm above floor | URDF (`lidar_ground_height`) |
| Extra height without cutting the top plate | ~5 mm | larger wheels, issue #3 |
| Further gain with through-plate caster mounting | ~6–7 mm | issue #3 |
| Robot mass | ~2 kg (estimate, more with arm) | BOM |

So the caster envelope is **~16 mm as built**, ~21 mm with bigger wheels, and
~27–28 mm with bigger wheels *plus* cutting mounting holes so a caster hangs
from the top face of the base plate. Every bought "small" caster below is
measured against those numbers.

### What wobble costs

The lidar scans at only ~54 mm. A downward scan-plane tilt of **1° grounds the
beam at ~3.1 m; 2° at ~1.5 m** — floor returns appear as phantom walls right in
the middle of useful range, corrupting both slam_toolbox and AMCL. Kinematic-ICP
additionally *assumes* planar motion, so pitch transients leak directly into
odometry error. Two distinct mechanisms matter:

- **Static/slow tilt** — a sagging or unevenly compressed mount tilts the scan
  plane persistently. This is the dangerous one for mapping.
- **Transient rocking** — the current *rigid* over-constrained stance already
  produces this: every accel/decel slams the chassis from one caster to the
  other, a hard-contact impact with the full contact-height error as amplitude.
  Rigid is not the wobble-free baseline; it is the current wobble source.

---

## Option Survey

### 1. Optimised rigid printed caster (shallower angle / slider foot)

Reprofile the hemisphere: a flat furniture-slider style foot with a rounded,
shallow (<30°) lead-in ramp instead of a curved face that presents >45° at rug
height, optionally with a PTFE glide insert for low friction (PTFE furniture
glides are a commodity — screw-on and adhesive discs in exactly this size
range).

- **Uneven floors/rugs:** fixes only the rug-edge *attack angle*; does nothing
  for the over-constraint, so slopes still strand the robot.
- **Wobble:** none added.
- **Height:** fits trivially.
- **Cost:** filament (+ ~£4 for PTFE glides).

**Verdict:** necessary but not sufficient. The shallow-ramp, low-friction foot
is the right *tip geometry* and carries into the recommendation — but as a
rigid part it cannot fix the failure mode.

### 2. Bought ball casters / mini ball transfer units

[Pololu-style ball casters](https://www.pololu.com/category/45/pololu-ball-casters)
(3/8″–1″ ball, ~10–18 mm heights) and industrial
[mini ball transfer units](https://www.kippusa.com/en-us/products/METRIC/Transport-technology/Ball-transfer-units/Ball-transfer-units-mini/p/agid.17882)
fit the height budget and roll omnidirectionally.

- **Uneven floors/rugs:** already tested on Mote — no improvement, as the
  statics predict. Small balls additionally dig into carpet pile, and open
  bearing races ingest carpet fibre and hair until they seize (the classic
  small-robot complaint).
- **Wobble:** none added (rigid), but inherits the rocking of any rigid stance.
- **Height:** fits.
- **Cost:** ~£3–8 each.

**Verdict:** rejected on evidence. Rigid contact in a nicer package.

### 3. Wheel casters (swivel)

The [smallest commodity swivel casters](https://www.mcmaster.com/products/low-profile-casters/)
start around 30–40 mm overall height — above even the stretched ~28 mm budget,
so this path requires cutting the top plate or a bigger wheel redesign than the
5 mm allowance. And two new problems arrive with the swivel:

- **Caster flutter/shimmy** at speed, and **trail kickback**: every direction
  reversal makes the swivel flip 180°, producing a lateral jerk right when the
  controller reverses — a yaw disturbance wheel odometry can't see.
- Still **rigid** vertically, so the over-constraint remains.

- **Uneven floors/rugs:** good over bumps a rolling wheel can climb; stance
  problem unsolved.
- **Wobble:** *adds* yaw/lateral disturbances (flip, flutter).
- **Height:** does not fit, even stretched.
- **Cost:** ~£3–6 each, plus wheel/chassis rework to gain the height.

**Verdict:** rejected. Doesn't fit the budget, and the swivel dynamics are
actively hostile to odometry on a robot this light.

### 4. Sprung drive wheels (the robot-vacuum approach)

Robot vacuums invert the problem: drive-wheel modules are spring-biased
downward (iRobot's patents describe
[5–25 N of bias per wheel](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10766324)),
and the chassis rides on rigid front (and sometimes rear) casters. Wheels
follow the floor; the body stays put.

- **Uneven floors/rugs:** the gold standard — traction is guaranteed by spring
  preload over large travel.
- **Wobble:** mixed. The chassis attitude is set by the *rigid casters*, so
  every bump under a caster still pitches the lidar directly; and traction is
  capped at spring preload rather than robot weight.
- **Height/complexity:** a major redesign — the STS3215s are hard-mounted to
  `Motor Support` blocks between the plates; sprung modules need vertical
  travel, guides, and cable service loops in space that doesn't exist. It also
  decouples wheel odometry from body motion during suspension travel.
- **Cost:** highest of any option (custom sprung modules).

**Verdict:** rejected for Mote. Right answer for a 3 kg vacuum designed around
it from day one; wrong retrofit for this chassis, and it doesn't even remove
caster-induced pitch.

### 5. Articulated rocker (linked fore/aft casters)

Join the front and rear casters with a beam pivoting on the chassis — four
contacts become three effective ones (wheels + rocker), statically determinate,
like a mini rocker-bogie.

- **Uneven floors/rugs:** kinematically exact; handles anything within pivot
  range.
- **Wobble:** the pivot is a genuinely free DoF, so the *chassis* pitch is
  determined — good — but the mechanism needs a stiff, low-slop pivot or the
  determinacy is lost to rattle.
- **Height/complexity:** fatal — the beam must run fore–aft under the base
  plate through the space occupied by the power bank and lidar mount, inside a
  16 mm envelope. There is no route.
- **Cost:** printed, but the most moving parts of any option.

**Verdict:** rejected on packaging. The theoretically cleanest fix, worth
remembering if the chassis is ever redesigned from scratch.

### 6. Compliant (preloaded plunger) casters — ⭐ recommended

Replace each rigid caster with a two-part printed unit: a **casing** bolted to
the ORP grid on the base plate, and a **plunger** sliding vertically inside it
with a rounded shallow-ramp tip (option 1's geometry) and ~4–5 mm of travel. A
force element in the plunger's hollow Ø12 core reacts against the chassis
plate itself (the plate is the cap — the only way the mechanism fits a 16 mm
ride height, since the small tip retracts *up through* the casing lip rather
than needing foot clearance below it). The casing lip retains the plunger; it
inserts from the top before the casing is bolted on.

Force element candidates:

| Element | Force curve | Notes |
|---|---|---|
| **Compression spring** (from a [~£6 assortment kit](https://www.amazon.co.uk/compression-spring/s?k=compression+spring)) | Linear, preload settable by fitted length | Best for wobble: firm, constant preload at ride height. Metal — no creep. |
| **Opposing Ø12×3 N42 magnet pair** (~[£5 for spares-drawer quantities](https://www.first4magnets.com/circular-disc-rod-c34/12mm-dia-x-3mm-thick-n42-neodymium-magnet-2-5kg-pull-p3619)) | Steeply rising (~soft at rest gap, firm near contact) | "Standard printer parts", no sourcing precision. But the curve is backwards for this job: weakest exactly at ride height, where preload matters. Usable only if installed pre-compressed. |
| Printed TPU/PLA flexure | Printable, zero BOM | Creep/compression-set under constant load → ride height and preload drift. Rejected. |
| Off-the-shelf spring plunger (M8/M10) | Linear, ready-made | Tiny ball tip (Ø5–6 mm) digs into carpet; travel typically only ~2–3 mm. Rejected as the contact, could serve as an internal force element but the spring alone is simpler. |

**Recommendation within the option: compression spring**, sized per the
analysis below; keep the magnet pair as the documented no-spring alternative
with the pre-compression caveat.

- **Uneven floors/rugs:** the travel absorbs contact-height error, slope
  transitions, and rug pile up to ~4–5 mm while the preload keeps pressing the
  tip down and — critically — keeps the drive wheels loaded. Rug *edges* are
  handled by the shallow-ramp tip plus the traction that now exists.
- **Wobble:** analysed in full below — net improvement over rigid.
- **Height:** fits the existing 16 mm ride height. **No bigger wheels, no
  top-plate cuts, no through-plate mounting, lidar height unchanged.**
- **Cost:** filament + ~£6 springs (or ~£5 magnets); optional PTFE tip insert
  ~£4.

---

## Wobble vs Compliance Analysis

The trade-off the issue asks about, with numbers. Model: casters at
L = ±100 mm from the axle, spring rate k per caster, robot mass ~2 kg.

**Preload keeps the stance defined.** With the CoM ~15 mm behind the axle, the
casters must supply a net restoring moment of only
2 kg × 9.81 × 0.015 m ≈ **0.3 N·m**, i.e. ~3 N more force at the rear caster
than the front. Setting preload at ~3–4 N per caster at ride height covers
this with margin while leaving ≈ 12–13 N (~65 % of weight) on the drive
wheels — traction *improves* over today, where rocking can take wheel load to
zero. Braking/acceleration pitch moments are tiny by comparison: at 0.5 m/s²
and ~40 mm CoM height, the load shift at a caster is ~0.4 N ≪ preload, so
**contact never breaks and the chassis never crosses a free-play gap** — the
impact transient that defines the current rigid rocking simply has no
mechanism.

**Stiffness sets the pitch mode.** With k ≈ 0.8 N/mm per caster, pitch
stiffness is 2kL² ≈ 16 N·m/rad. Against a pitch inertia of ~0.016 kg·m²
(2 kg at ~90 mm radius of gyration) the pitch natural frequency is ~5 Hz —
well above drive-command content (< 2 Hz), so accel/decel excites the
suspension quasi-statically rather than ringing it. The printed sliding fit
adds friction damping for free. Softer springs push the mode down toward the
excitation band and increase sag; much stiffer ones stop absorbing floor error.
k ≈ 0.5–1 N/mm with 3–5 N preload is the design window.

**Bounded, smooth tilt instead of impacts.** Compliance does not eliminate
pitch — a robot that tolerates a 4 mm floor step *must* pitch while crossing
it (a 4 mm differential across the 200 mm caster span is ~1.1°). What changes
is the character: the rigid stance takes that error as a hard impact with wheel
unloading; the sprung stance takes most of it into the spring (the wheels, not
the caster, are the stiff reference), leaving a smooth, friction-damped
fraction-of-a-degree chassis motion. For the lidar the comparison is:
occasional bounded ramps vs today's per-accel impact steps *plus* getting
stuck. Static tilt is bounded by preload matching (front/rear springs fitted to
the same length; a 1 mm asymmetry across 200 mm is < 0.3°) and by using metal
springs so nothing creeps.

**Why not magnets first:** the repulsion curve means the preload at ride height
is the *weakest* point of the stroke, exactly where the stance must be firm;
the effective rate then rises steeply through the travel. The same parts work
acceptably if the pair is installed with the gap pre-closed (preload built in),
but a spring achieves the target curve without the workaround.

---

## Recommendation

**Fit preloaded plunger casters (option 6) at both the front and rear
positions**: printed casing + printed plunger, rounded shallow-ramp tip,
compression spring in the hollow core reacting against the base plate,
~4–5 mm travel, ~3–4 N preload, k ≈ 0.5–1 N/mm. Combine with option 1's tip
geometry (shallow lead-in, optional PTFE insert).

Rationale: it is the only option that (a) addresses the actual failure — the
over-constrained rigid stance — rather than the caster's packaging, (b) fits
inside the existing 16 mm ride height with no wheel, plate, or lidar-height
changes, (c) *reduces* lidar disturbance relative to today by replacing hard
rocking impacts with preloaded, friction-damped, bounded compliance, and
(d) costs a few pounds of commodity hardware.

### BOM delta (vs [`BOM.md`](../BOM.md))

| Change | Part | Qty | Unit price | Notes |
|---|---|---|---|---|
| − | Caster (3D printed hemisphere) | 1 | (filament) | removed |
| + | Caster casing + plunger (3D printed, PLA) | 2 sets | (filament) | front and rear |
| + | Compression spring assortment kit | 1 | ~£6 | [Amazon UK](https://www.amazon.co.uk/compression-spring/s?k=compression+spring); pick ~Ø8–10 mm, k ≈ 0.5–1 N/mm |
| + *(alt.)* | Ø12×3 mm N42 disc magnets (opposing pair per caster) | 4 | ~£5/pack | [first4magnets](https://www.first4magnets.com/circular-disc-rod-c34/12mm-dia-x-3mm-thick-n42-neodymium-magnet-2-5kg-pull-p3619); only with built-in pre-compression |
| + *(opt.)* | PTFE glide insert for plunger tip | 2 | ~£4/pack | [commodity furniture glides](https://www.amazon.com/teflon-glides/s?k=teflon+glides) |

**Net delta: ~£6–10.**

### Implied CAD / mounting changes (follow-up task, not this one)

1. Retire `Caster.step`; author a two-part **casing** (bolts to existing ORP
   grid holes on the base-plate underside — no new holes in either plate) and
   **plunger** (rounded shallow-ramp tip, hollow Ø12 core, lip-caught body,
   inserted from the top). The base plate itself is the spring's upper seat.
2. Two units: the front position and a rear position on the same grid.
3. Update `ASSEMBLY.md` (print table, step 3, drop the "provisional" caveat)
   and `BOM.md` per the delta.
4. Update the URDF caster properties (`caster_radius`, `caster_x`, `caster_z`
   are currently marked *assumed*) to the as-built values, and add the rear
   caster link.
5. Fit check on hardware: confirm ride-height preload front/rear, then a
   mapping run over a rug edge as the acceptance test — wheels must stay
   loaded and the map must stay clean.

---

## Sources

- [Mote issue #3 — caster design](https://github.com/ClachDev/Mote/issues/3)
- [Pololu ball casters (category)](https://www.pololu.com/category/45/pololu-ball-casters)
- [Pololu 3/8″ metal ball caster](https://www.pololu.com/product/951) / [1/2″](https://www.pololu.com/product/953)
- [KIPP mini ball transfer units](https://www.kippusa.com/en-us/products/METRIC/Transport-technology/Ball-transfer-units/Ball-transfer-units-mini/p/agid.17882)
- [SKF miniature ball transfer units](https://www.skf.com/us/products/other-products/skf-ball-transfer-units/miniature-ball-transfer-units)
- [McMaster-Carr low-profile casters](https://www.mcmaster.com/products/low-profile-casters/)
- [iRobot wheel suspension patent US10766324 (5–25 N sprung drive modules)](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10766324)
- [iRobot cleaning-system patent US11363933 (spring-loaded caster, floor-contact sensing)](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11363933)
- [Compression spring assortment kits (Amazon UK)](https://www.amazon.co.uk/compression-spring/s?k=compression+spring)
- [Ø12×3 N42 neodymium discs (first4magnets)](https://www.first4magnets.com/circular-disc-rod-c34/12mm-dia-x-3mm-thick-n42-neodymium-magnet-2-5kg-pull-p3619)
- [PTFE furniture glides (commodity examples)](https://www.amazon.com/teflon-glides/s?k=teflon+glides)
