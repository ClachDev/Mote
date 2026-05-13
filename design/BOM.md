# Bill of Materials

Prices in GBP. Sensor configuration add-ons are listed separately — the base build uses two USB cameras matching LeKiwi.

## Power: 5V only

AuldBot standardises on 5V. See the README for the full rationale.

## Base Build

### Structure
| Part | Qty | Unit price | Link |
|------|-----|-----------|------|
| Chassis plates (3D printed) | — | (filament only) | — |
| Drive wheel + hub (STS3215 25T horn compatible) | 2 | TBD | TBD |
| Passive front caster | 1 | ~£6 | TBD |
| M3 button head hex socket screw set (751pcs, M3×6–35, nuts, washers) | 1 | ~£6 | [Amazon UK](https://www.amazon.co.uk/dp/B0DSDJCRNB) |

### Electronics
| Part | Qty | Unit price | Link |
|------|-----|-----------|------|
| Raspberry Pi 5 (4GB) | 1 | ~£99 | [CPC Farnell](https://cpc.farnell.com/raspberry-pi/rpi5-4gb-single/raspberry-pi-5-4gb/dp/SC20210) |
| Waveshare Serial Bus Servo Driver Board | 1 | ~£15 | [Amazon UK](https://www.amazon.co.uk/dp/B0CJ6TP3TP) |
| Feetech STS3215 7.4V servo — 1/191 gear (C044) | 2 | ~£12 | [Alibaba](https://www.alibaba.com/product-detail/Low-Cost-Feetech-STS3215-Servo-7_1601611431055.html) |
| UGREEN Nexode 140W Power Bank 25000mAh, 2× USB-C + 1× USB-A (160×81×27mm) | 1 | ~£50 | [Amazon UK](https://www.amazon.co.uk/dp/B0BJQ7F16T) |
| USB-C to USB-C cable (2-pack, short) | 1 | ~£6 | [Amazon UK](https://www.amazon.co.uk/dp/B0DPBRW455) |
| USB-A to USB-C cable (3-pack) | 1 | ~£7 | [Amazon UK](https://www.amazon.co.uk/dp/B0BX6BPPNP) |
| USB-C to DC barrel cable (DSD TECH SH-CP05A) | 1 | ~£10 | [Amazon UK](https://www.amazon.co.uk/dp/B0B9G1KFL3) |
| microSD card (32GB+) | 1 | ~£20 | TBD |

### Sensors (base config)
| Part | Qty | Unit price | Link |
|------|-----|-----------|------|
| UGREEN 1080p/30fps USB Webcam 85° FOV | 2 | ~£15 | [Amazon UK](https://www.amazon.co.uk/dp/B0C76ZD7KV) |
| SLAMTEC RPLIDAR C1 360° DTOF, 10Hz, 12m | 1 | ~£56 | [AliExpress](https://www.aliexpress.com/item/1005006641728089.html) |

**Base total (excluding print cost): ~£243**


---

## SO-101 Follower Arm

The chassis is compatible with the [SO-101 follower arm](https://github.com/TheRobotStudio/SO-ARM100). Refer to that project for its BOM and assembly instructions.

---

---

## Optional Extras

### Improved Odometry

| Part | Qty | Unit price | Link | Notes |
|------|-----|-----------|------|-------|
| BNO085 9-DOF IMU breakout | 1 | ~£15 | [AliExpress](https://www.aliexpress.com/item/1005010674706575.html) | Requires soldering iron to attach header pins. Enables wheel slip detection and improved odometry when fused with wheel encoders via `robot_localization`. Connect via I2C (VCC, GND, SDA, SCL) to the Pi GPIO header. |

---

## Notes

- Drive wheel spec (hub compatible with STS3215 25-tooth spline horn) is **TBD** — needs confirming before ordering.
- Sensor config slots on the chassis follow the ORP 3.5mm / 20mm grid standard.
