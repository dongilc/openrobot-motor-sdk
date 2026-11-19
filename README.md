# OpenRobot Motor SDK

English | [한국어](README.ko.md)

Libraries and examples for driving OpenRobot motor controllers over CAN and
EtherCAT, from the first spin to a multi-joint robot.

| Part | Platforms | Checked with |
|---|---|---|
| [CAN protocol](docs/can_protocol.md) | all | firmware 261119 source |
| [EtherCAT interface](docs/ethercat_protocol.md) | all | firmware 261119 source, [ESI file](ethercat/OpenRobot_MC.xml) |
| [Python package and examples](python/) | Windows, Linux, macOS | PCAN and EtherCAT on Windows |
| [C++ library and examples](cpp/) | Windows, Linux, macOS | PCAN on Windows |
| [ROS 2 (ros2_control)](ros2/) | Linux (Jazzy) | PCAN through SocketCAN |
| [Arduino library](arduino/) | boards with an MCP2515 shield | Arduino Zero, Seeed CAN-BUS Shield |
| [Android field check app](android/) | Android 7.0+ with USB OTG | CANable (slcan) |

CAN adapters: slcan (CANable and compatible) and PEAK PCAN.

## Controllers and firmware

| Board | Firmware |
|---|---|
| SPN-MC V2 / V2R2, SPN-MC D14, SPN-MC D17_D20, VESCAT HIGH V2R2 / V2R3 / V2R4 | 261119 or later recommended |

Firmware and the Motor Tool are released at
[openrobot-motor-tool](https://github.com/dongilc/openrobot-motor-tool/releases).
Older firmware works with this SDK, but has no command watchdog: every command
keeps running when the host stops sending (see
[CAN protocol, section 8](docs/can_protocol.md#8-safety-notes)).

## Examples, by step

0. Connect: find motors, read status
1. Current, speed and position commands
2. MIT impedance control
3. Several motors in one loop
4. Safety: faults, missed replies, Ctrl+C

Every example that moves asks for confirmation first and releases the motor
on exit.

## Safety

- Position and speed commands keep running when commands stop. Supervise your
  own loop and send Motor off (0x80) on any error.
- Keep an emergency stop that cuts motor power, independent of the host.
- Angles and speeds are motor side, except in MIT mode (output side).

## License

MIT, see [LICENSE](LICENSE). The SDK only talks to the controller over CAN or
EtherCAT; the controller firmware is distributed separately under its own
license.
