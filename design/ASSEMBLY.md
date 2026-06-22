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

| Part (qty)                  | Material       |
| --------------------------- | -------------- |
| Chassis Base (1)            | PLA            |
| Chassis Top (1)             | PLA            |
| Motor Support (2)           | PLA            |
| Pi Bottom + Pi Top (1 each) | PLA            |
| Waveshare Mount (1)         | PLA            |
| C1 Lidar Mount (1)          | PLA            |
| Camera Mount (1)            | PLA            |
| Battery Mount (1)           | PLA            |
| Wheel Inner (2)             | PLA            |
| Wheel Tyre (2)              | **TPU** (grip) |
| Caster (1)                  | PLA            |
| SO Base ORP (1, optional)   | PLA            |

Notes:

- **Wheels are two-part**: a rigid `Wheel Inner` hub plus a `Wheel Tyre` printed
  in **TPU** for traction.
- **Caster is unresolved** — printed and several off-the-shelf options have all
  been unsatisfactory so far; treat this part as provisional. A bought ball
  caster of the right height may be preferable (see Assembly step 6).

## Assembly

**Hardware:** the M3 button-head hex set from the [BOM](BOM.md) (screws, nuts)
covers all fasteners. Most joints take **M3×12**: the screw passes through a ~6
mm plate into a part with a ~6 mm captive-nut pocket. Use **M3×10** for thinner
stacks.

> **Note:** I am not convinced yet that screws and nuts are the optimal choice
> here. Nuts have a tendency to loosen up due to vibrations while driving around
> so we may want something more reliable.

Suggested order:

1. **Servos.** Mount each STS3215 to the `Motor Support`, then press a
   `Wheel Inner` onto the servo horn and fit the `Wheel Tyre` over it. Wheels
   are centred and inset so the footprint stays circular. It's easiest to set
   the servo ID's before going further. Left servo is **ID 7**, right is **ID
   9** (assign with `pixi run setup-ids` — see the
   [README](../README.md#4-configure-the-servos)).
2. **Lower plate.** Fasten the `Motor Support`/servos, the `Battery Mount`, the
   `Waveshare Mount`, and the `C1 Lidar Mount`, to the `Chassis Base`.
3. **Caster.** Fit the front `Caster` to the underside of the base for the third
   contact point (provisional — see Printing notes).
4. **Sensors.** Mount the camera in the `Camera Mount`.
5. **Top plate.** Attach the `Pi Bottom` holder and the `Camera/Camera Mount` to
   the `Chassis Top`.
6. **Close it up.** Fix the `Chassis Top` onto the standoffs.
7. **Power bank.** Seat the power bank in the `Battery Mount` between the
   plates.
8. **Wire it.** Follow [WIRING.md](WIRING.md): bank Out1 → servo board (barrel),
   bank Out2 → Pi, servo board USB → Pi, lidar/camera USB → Pi. Route cables
   through the standoff gap and keep the lidar's 360° view clear.
9. **(Optional) SO-101 arm.** The `SO Base ORP` adapter mounts the
   [SO-101 follower arm](https://github.com/TheRobotStudio/SO-ARM100) on the ORP
   grid.

Then bring the software up per the [main README](../README.md#5-launch).
