# Mote Design

## Design Decisions

### Chassis diameter: 235mm

The chassis uses a 235mm circular plate, adapted from the [ORP 360mm circular
plate](https://openroboticplatform.com/part:12). 235mm fits comfortably on a
standard 256mm 3D printer bed and gives a good outer hole layout from the ORP
grid — 240mm places holes right at the plate edge, and 250mm+ would be tight on
the bed. All components fit within the 235mm footprint, and the power bank
(152mm longest dimension) fits comfortably within the circle.

### Power: 5V only

Mote standardises on 5V (USB-C power bank) and does not offer a 12V variant.
The 12V config requires a DC-DC converter to power the Pi, adding cost and
complexity. A power bank outperforms a typical 12V Li-ion pack on
capacity (~100Wh vs ~60Wh), charges via standard USB-C, and has a built-in BMS.

The power bank directly connects to the Pi (USB-C to C) and the MCB (USB-C to C
to DC).

### Power bank form factor

The power bank is sandwiched between the two lower chassis layers to keep the
centre of mass low. The servo height is 45.2mm and the lidar is 41.3mm. For some
extra space and nice numbers I've buffered the standoffs to 50mm. This is then
the limiting factor in choosing a power bank. I've found that availability
changes depending on country so the most important thing to look for is a height
less than 50mm, and two USB-C ports. I found that with the UGREEN 140W bank
which advertises 100W on In/Out1 and 45W on Out2, I need to connect the MCB to
Out1 to stop them stalling. The Pi runs fine on the 45W port.

## Requirements

- Only two motors with differential drive. This matches real robots more closely
  than the three motors LeKiwi has.
- Normal wheels, not holonomics. Those are noisy and expensive.
- The wheels should be centered and inset so the footprint is circular.
- The chassis should follow the [ORP](https://openroboticplatform.com/) standard
  for interoperable parts.
- Standard sensor configurations
  - Camera
  - Lidar
- It should use "standard" parts, i.e. either Dynamixel or Feetech servos.
- The chassis should be compatible with the SO-101 arm from TheRobotStudio.
- Mote is intended as a comparison platform between the classical ROS/Nav2
  navigation approach and the LeRobot learned policy approach.

## Bill of Materials

See [BOM.md](BOM.md).

## Build guides

- [WIRING.md](WIRING.md) — connections, power topology and budget, IMU pinout.
- [ASSEMBLY.md](ASSEMBLY.md) — print settings per part and assembly steps.
