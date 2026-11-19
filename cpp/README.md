# openrobot_motor (C++)

Header-only C++11 library for OpenRobot motor controllers over CAN.
Protocol reference: [../docs/can_protocol.md](../docs/can_protocol.md).

| Header | Use | Needs |
|---|---|---|
| `openrobot_motor/protocol.hpp` | frame encoding and decoding | nothing (no heap, no exceptions, no STL) |
| `openrobot_motor/bus.hpp` | request/reply, fault notices, `Motor` class | a `Transport` for your CAN hardware |
| `openrobot_motor/pc_transport.hpp` | PCAN, slcan, SocketCAN on a PC | C++11 standard library |

`protocol.hpp` and `bus.hpp` also build for microcontrollers (checked with
`-Werror` on Cortex-M0 and M4); implement `Transport` for your CAN peripheral.

## Build

    cmake -S . -B build
    cmake --build build --config Release

Builds the examples and `test_protocol`. Windows (MSVC), Linux and macOS.

## Adapters

| Adapter | Windows | Linux | macOS |
|---|---|---|---|
| PEAK PCAN-USB | `pcan PCAN_USBBUS1` (PEAK driver) | `socketcan can0`, or `pcan` with libpcanbasic | `pcan PCAN_USBBUS1` (PCBUSB from MacCAN) |
| slcan (CANable and compatible) | `slcan COM5` | `slcan /dev/ttyACM0` | `slcan /dev/tty.usbmodem1101` |

PCAN-Basic is loaded at run time, so it is only needed on machines that use
a PCAN adapter.

## Examples

    scan  pcan PCAN_USBBUS1          # find motors, read state; nothing moves
    speed pcan PCAN_USBBUS1 1 360    # Speed (DPS): +/-360 deg/s, then hold
    speed pcan PCAN_USBBUS1 1 3000 erpm   # Speed (eRPM): +/-3000 eRPM
    mit   pcan PCAN_USBBUS1 1        # MIT sine on the output shaft, 200 Hz
    multi pcan PCAN_USBBUS1 1 2 3    # several motors in one loop

Examples that move ask for confirmation first. Ctrl+C stops them.

## Use in your own code

```cpp
#include "openrobot_motor/pc_transport.hpp"
using namespace openrobot;

auto t = open_transport("pcan", "PCAN_USBBUS1");
MotorBus bus(*t);
Motor m(bus, 1);                 // released (0x80) when m goes out of scope
Status s;
if (m.speed_dps(360, &s) == OK) printf("%d deg/s\n", s.speed_dps);
```

Pass `release_on_exit = false` (`Motor m(bus, 1, false)`) for read-only use,
so inspecting a joint does not release a position it is holding.

## Safety

When commands stop, the controller's reaction depends on the command and
the firmware (protocol document, section 8). With firmware 261119 or later,
current and MIT commands switch to damping after 100 ms by default, while
position and speed commands keep running; older firmware keeps running every command.
Supervise your loop and release on error, as the examples do.

## Tests

`tests/test_vectors.inc` is generated from the Python implementation
(`python tests/gen_vectors.py`); `test_protocol` checks every frame byte for
byte against it.
