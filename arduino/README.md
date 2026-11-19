# OpenRobotMotor (Arduino)

Drive OpenRobot SPN motor controllers over CAN from an Arduino with an
MCP2515 CAN shield or module.
Protocol reference: [../docs/can_protocol.md](../docs/can_protocol.md).

The protocol code is the same as the [C++ library](../cpp/); this library
adds a transport for the MCP2515.

## Hardware

| Item | Notes |
|---|---|
| Arduino board | checked: Arduino Zero; builds for UNO and other boards with SPI |
| CAN interface | MCP2515 with a **16 MHz** crystal, e.g. Seeed CAN-BUS Shield |
| Bus | 1 Mbit/s, CAN_H to CAN_H, CAN_L to CAN_L, 120 ohm at both ends |

- 1 Mbit/s needs a 16 MHz crystal. Many small blue MCP2515 modules have an
  8 MHz crystal and cannot run at 1 Mbit/s.
- Chip select: Seeed CAN-BUS Shield V2 uses pin 9, V1.x uses pin 10. Set
  `CAN_CS_PIN` at the top of each example.
- Seeed shield: use the CAN_H / CAN_L screw terminal. The DB9 pinout depends
  on solder jumpers on the shield and may not match a CiA (PCAN) cable.
- Common ground between the Arduino and the controllers is recommended.

## Install

1. Install the **autowp-mcp2515** library (Library Manager: "mcp2515" by autowp).
2. Copy the `OpenRobotMotor` folder into your Arduino `libraries` folder.

With arduino-cli:

    arduino-cli lib install autowp-mcp2515
    arduino-cli compile -b arduino:samd:arduino_zero_edbg --library OpenRobotMotor -u -p COM6 OpenRobotMotor/examples/Scan

## Examples

Open the Serial Monitor at 115200 baud, line ending "Newline".

| Example | What it shows | Moves |
|---|---|---|
| `Scan` | find motors, read state; send any key to scan again | no |
| `Speed` | Speed (DPS) or Speed (eRPM) (`USE_ERPM`) both ways, then release | yes |
| `Mit` | MIT impedance control, sine on the output shaft at 200 Hz | yes |
| `MultiMotor` | several motors in one loop (`MOTOR_IDS`) | yes |

Run `Scan` first. Examples that move wait until you type `yes`; any key
stops them. Set `MOTOR_ID` (or `MOTOR_IDS`) to your motor IDs.

## Use in your own code

```cpp
#include <OpenRobotMotor.h>
using namespace openrobot;

Mcp2515Transport can(9);       // CS pin
MotorBus bus(can);
Motor motor(bus, 1);

void setup() {
    can.begin(CAN_1000KBPS, MCP_16MHZ);
}

void loop() {
    Status s;
    if (motor.speed_dps(360, &s) == OK) { // Speed (DPS), deg/s, motor side
        // s.speed_dps, s.current_a, s.temperature_c
    }
    delay(10);
}
```

Every command waits for the controller's reply (20 ms timeout) and returns
`OK` or `NO_REPLY`. Other CAN hardware (UNO R4, ESP32 TWAI, Teensy FlexCAN):
implement `openrobot::Transport` (send, recv, now_ms) the way
`Mcp2515Transport` does.

## Safety

When commands stop, the controller's reaction depends on the command and
the firmware (protocol document, section 8). With firmware 261119 or later,
current and MIT commands switch to damping after 100 ms by default, while
position and speed commands keep running; older firmware keeps running every command.
An Arduino that resets or hangs during a position or speed command leaves
the motor running. Keep an emergency stop on the motor power, and call
`motor.off()` when your sketch is done.

## Troubleshooting

| Symptom | Cause |
|---|---|
| "MCP2515 not found" | wrong `CAN_CS_PIN`, shield not seated |
| "No motor answered" | wiring, termination, bit rate, crystal, motor power |
| no reply and the controllers see nothing | CAN_H/CAN_L not on the same bus (the MCP2515 gets no ACK) |

## Maintainers

`src/openrobot_motor/protocol.hpp` and `bus.hpp` are copies of
`../cpp/include/openrobot_motor/`. After changing those, run
`python sync_core.py`.
