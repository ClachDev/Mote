# Mote Design

## Design Decisions

### Chassis diameter: 230mm

The chassis uses a 230mm circular plate, adapted from the [ORP 360mm circular
plate](https://openroboticplatform.com/part:12). 230mm fits comfortably on a
standard 256mm 3D printer bed and gives a good outer hole layout from the ORP
grid — 240mm places holes right at the plate edge, and 250mm+ would be tight on
the bed. All components fit within the 230mm footprint, and the power bank
(152mm longest dimension) fits comfortably within the circle.

### Power: 5V only

Mote standardises on 5V (USB-C power bank) and does not offer a 12V variant.
The 12V config requires a DC-DC converter to power the Pi, adding cost and
complexity. A 20000mAh power bank outperforms a typical 12V Li-ion pack on
capacity (~100Wh vs ~60Wh), charges via standard USB-C, and has a built-in BMS.
When the SO-101 arm is added, it runs on 5V in its standard config, so one power
bank powers the whole robot.

The power chain is: **power bank → Pi (USB-C) → MCB (USB-C to DC barrel jack)**.
The MCB does not draw power directly from the power bank.

### Power bank form factor

The power bank is sandwiched between the two lower chassis layers to keep the
centre of mass low. The inter-layer gap is set by the servo height of
**45.2mm**, so the power bank's smallest cross-sectional dimension must be
≤45.2mm. Standard square power banks measure ~50mm on each side and do not fit —
a slim/flat form factor is required. The bank must also provide at least 85W
total output across two simultaneous ports (Pi + MCB).

## Requirements

- Only two motors with differential drive. This matches real robots more closely
  than the three motors LeKiwi has.
- Normal wheels, not holonomics. Those are noisy and expensive.
- The wheels should be centered and inset so the footprint is circular.
- The chassis should follow the [ORP](https://openroboticplatform.com/) standard
  for interoperable parts.
- Standard layouts for sensor configurations
  - Two cameras
  - Lidar
  - 3D camera
  - Combinations of the above
- It should use "standard" parts, i.e. either Dynamixel or Feetech servos.
- The chassis should be compatible with the SO-101 arm from TheRobotStudio.
- Mote is intended as a comparison platform between the classical ROS/Nav2
  navigation approach and the LeRobot learned policy approach.

## Bill of Materials

See [BOM.md](BOM.md).
