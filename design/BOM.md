# Bill of Materials

Prices in GBP. Sensor configuration add-ons are listed separately — the base
build uses a single USB camera.

## Base Build

### Structure

| Part                                                                 | Qty | Unit price      | Link                                                |
| -------------------------------------------------------------------- | --- | --------------- | --------------------------------------------------- |
| Chassis plates (3D printed)                                          | —   | (filament only) | —                                                   |
| M3 button head hex socket screw set (751pcs, M3×6–35, nuts, washers) | 1   | ~£6             | [Amazon UK](https://www.amazon.co.uk/dp/B0DSDJCRNB) |

### Electronics

| Part                                                           | Qty | Unit price | Link                                                                                                  |
| -------------------------------------------------------------- | --- | ---------- | ----------------------------------------------------------------------------------------------------- |
| Raspberry Pi 5 (4GB)                                           | 1   | ~£99       | [CPC Farnell](https://cpc.farnell.com/raspberry-pi/rpi5-4gb-single/raspberry-pi-5-4gb/dp/SC20210)     |
| Waveshare Serial Bus Servo Driver Board                        | 1   | ~£15       | [Amazon UK](https://www.amazon.co.uk/dp/B0CJ6TP3TP)                                                   |
| Feetech STS3215 7.4V servo — 1/191 gear (C044)                 | 2   | ~£12       | [Alibaba](https://www.alibaba.com/product-detail/Low-Cost-Feetech-STS3215-Servo-7_1601611431055.html) |
| UGREEN Nexode 140W Power Bank 25000mAh, 2× USB-C (160×81×27mm) | 1   | ~£50       | [Amazon UK](https://www.amazon.co.uk/dp/B0BJQ7F16T)                                                   |
| USB cables (1× A to C 0.3m, 1× C to C 0.15m, 1× C to C 0.3m)   | —   | ~£5        | [AliExpress](https://www.aliexpress.com/item/1005008756186185.html)                                   |
| USB-C to DC 5.5×2.1mm barrel cable (5V)                        | 1   | ~£2        | [AliExpress](https://www.aliexpress.com/item/1005007387571379.html)                                   |
| SanDisk Extreme 64GB microSD (UHS-I, V30, A2, C10, U3)         | 1   | ~£20       | [Amazon UK](https://www.amazon.co.uk/dp/B09X7C7LL1)                                                   | A2 rating is the key spec for Pi 5 performance |

### Sensors (base config)

| Part                                    | Qty | Unit price | Link                                                                |
| --------------------------------------- | --- | ---------- | ------------------------------------------------------------------- |
| UGREEN 1080p/30fps USB Webcam 85° FOV   | 1   | ~£15       | [Amazon UK](https://www.amazon.co.uk/dp/B0C76ZD7KV)                 |
| SLAMTEC RPLIDAR C1 360° DTOF, 10Hz, 12m | 1   | ~£56       | [AliExpress](https://www.aliexpress.com/item/1005006641728089.html) |

**Base total (excluding print cost): ~£292**

---

## SO-101 Follower Arm

The chassis is compatible with the [SO-101 follower
arm](https://github.com/TheRobotStudio/SO-ARM100). Refer to that project for its
BOM and assembly instructions.

---

## In Testing

Parts not yet confirmed working on Mote — may be added to the base build or
optional extras once validated.

| Part                      | Qty | Unit price | Link                                                                | Notes                                                                                                                                                           |
| ------------------------- | --- | ---------- | ------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| BNO085 9-DOF IMU breakout | 1   | ~£15       | [AliExpress](https://www.aliexpress.com/item/1005010674706575.html) | Requires soldering iron to attach header pins. Intended for wheel slip detection and improved odometry when fused with wheel encoders via `robot_localization`. |
| M3 nyloc nuts (~100pcs)   | 1   | ~£3        | —                                                                   | Vibration-resistant swap for plain nuts at joints where the nut sits bare (servo lugs; anywhere outside a hex pocket). See [research/fastening.md](research/fastening.md). |

---

## Notes

- Sensor config slots on the chassis follow the ORP 3.5mm / 20mm grid standard.
