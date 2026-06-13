# Printing & Assembly

How to print the parts and put Mote together. Wiring is covered separately in
[WIRING.md](WIRING.md); the part files live in
[`step/`](step/) (editable in CAD), [`stl/`](stl/) (mesh) and
[`3mf/`](3mf/) (mesh + print setup).

> ⚠️ Print settings below are sensible starting points, not measured-optimal —
> confirm against your printer/filament. The `.3mf` files carry their own plate
> setup; prefer them if your slicer reads 3mf.

## Printing

All parts fit a 256 mm bed (the chassis is 235 mm — see
[design rationale](README.md#chassis-diameter-235mm)).

Suggested defaults: **PLA**, 0.2 mm layer height, 3 walls, 15–20% infill, brim on
the large flat plates to prevent corner lift.

| Part (qty) | Material | Orientation | Supports |
| --- | --- | --- | --- |
| Chassis Base (1) | PLA | flat | none |
| Chassis Top (1) | PLA | flat | none |
| Motor Support (1–2) | PLA | as exported | likely none |
| Pi Bottom + Pi Top (1 each) | PLA | flat | none |
| Waveshare Mount (1) | PLA | flat | none |
| C1 Lidar Mount (1) | PLA | flat | check overhangs |
| Camera Mount (1) | PLA | flat | check overhangs |
| Battery Mount (1) | PLA | flat | none |
| Wheel Inner (2) | PLA | hub down | none |
| Wheel Tyre (2) | **TPU** (grip) | flat | none |
| Caster (1) | PLA | — | — |
| SO Base ORP (1, optional) | PLA | flat | none |

Notes:
- **Wheels are two-part**: a rigid `Wheel Inner` hub plus a `Wheel Tyre` —
  printing the tyre in TPU gives traction. ⚠️ Confirm the intended tyre material.
- **Caster is unresolved** — printed and several off-the-shelf options have all
  been unsatisfactory so far; treat this part as provisional. A bought ball
  caster of the right height may be preferable (see Assembly step 6).
- Drive wheels and hubs are intentionally printed (removed from the
  [BOM](BOM.md) for that reason).

## Assembly

**Hardware:** the M3 button-head hex set from the [BOM](BOM.md) (screws, nuts,
washers) covers all fasteners. Standoffs are buffered to **50 mm** (set by servo
height 45.2 mm / lidar 41.3 mm). ⚠️ Exact screw lengths and standoff count per
joint aren't captured here yet — fill in as you build.

Suggested order:

1. **Drive train.** Mount each STS3215 to the `Motor Support`, then press a
   `Wheel Inner` onto the servo horn and fit the `Wheel Tyre` over it. Wheels are
   centred and inset so the footprint stays circular. Left servo is **ID 7**,
   right is **ID 9** (assign with `pixi run setup-ids` — see the
   [README](../README.md#4-configure-the-servos)).
2. **Lower plate.** Fasten the `Motor Support`/servos and the `Battery Mount` to
   the `Chassis Base`.
3. **Power bank sandwich.** Seat the power bank in the `Battery Mount` between the
   plates to keep the centre of mass low, then stand the **50 mm** standoffs up
   from the base.
4. **Electronics deck.** Mount the Pi in the `Pi Bottom`/`Pi Top` holder and the
   servo board on the `Waveshare Mount`, then attach both to the `Chassis Top`.
5. **Close it up.** Fix the `Chassis Top` onto the standoffs.
6. **Caster.** Fit the front `Caster` to the underside of the base for the third
   contact point (provisional — see Printing notes).
7. **Sensors.** Mount the `C1 Lidar Mount` and `Camera Mount` to the sensor slots
   — these follow the ORP **3.5 mm / 20 mm grid**, so they relocate on the grid.
8. **Wire it.** Follow [WIRING.md](WIRING.md): bank Out1 → servo board (barrel),
   bank Out2 → Pi, servo board USB → Pi, lidar/camera USB → Pi. Route cables
   through the standoff gap and keep the lidar's 360° view clear.
9. **(Optional) SO-101 arm.** The `SO Base ORP` adapter mounts the
   [SO-101 follower arm](https://github.com/TheRobotStudio/SO-ARM100) on the ORP
   grid.

Then bring the software up per the [main README](../README.md#5-launch).
