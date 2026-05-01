# Third Party Libraries

## SCServo

C++ SDK for Feetech STS-series servos, provided by Waveshare for the
[Serial Bus Servo Driver Board](https://www.waveshare.com/wiki/Serial_Bus_Servo_Driver_Board).

Download the Linux SDK from:
https://files.waveshare.com/wiki/Bus_Servo_Driver_HAT_A/SCServo_Linux.rar

Extract and copy the `SCServo_Linux` folder into `third_party/SCServo_Linux/`.

Note: the `.rar` is from the Bus Servo Driver HAT (A) wiki page, not the Serial Bus
Servo Driver Board, but the SDK is compatible as both use the same Feetech STS protocol.

The Arduino version (`SCServo.rar` from https://files.waveshare.com/upload/7/78/SCServo.rar)
will NOT compile on Linux — it depends on `HardwareSerial` and `Arduino.h`.

The SDK is not managed as a git submodule as no official public repository exists.
