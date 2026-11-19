# openrobot-motor (Python)

Drive OpenRobot motor controllers over CAN (and EtherCAT) from Windows, Linux or macOS.
Protocol reference: [../docs/can_protocol.md](../docs/can_protocol.md).

## Install

    pip install -e .            # from this folder (PyPI release later)

Python 3.8 or newer.

## Try it without hardware

Every example runs against a built-in simulator:

    cd examples
    python 00_scan.py --sim
    python 20_mit.py --sim

## Connect an adapter

| Adapter | Windows | Linux | macOS |
|---|---|---|---|
| slcan (CANable and compatible) | `-i slcan -c COM5` | `-i slcan -c /dev/ttyACM0` | `-i slcan -c /dev/tty.usbmodem1101` |
| PEAK PCAN-USB | `-i pcan` (PEAK driver; `-c PCAN_USBBUS2` for a second adapter) | `-i socketcan -c can0` | `-i pcan -c PCAN_USBBUS1` (PCBUSB library from MacCAN) |
| any SocketCAN device | | `-i socketcan -c can0` | |

The bus runs at 1 Mbit/s by default. On Linux with SocketCAN, set it first:

    sudo ip link set can0 up type can bitrate 1000000

## Examples

| File | What it shows | Moves |
|---|---|---|
| `00_scan.py` | find motors, read state, limits, faults | no |
| `10_current.py` | current (torque) control | yes |
| `11_speed.py` | Speed (DPS) `--dps` or Speed (eRPM) `--erpm`, hold, release | yes |
| `12_position.py` | position with speed limit, output-shaft degrees (`--gear`) | yes |
| `20_mit.py` | MIT impedance control, sine trajectory | yes |
| `30_multi_motor.py` | several motors in one loop (`--ids 1 2 3`) | yes |
| `40_safety.py` | fault notices, missed replies, Ctrl+C | yes |

Examples that move ask for confirmation first. Run `00_scan.py` before
anything else.

### EtherCAT

For boards with the EtherCAT interface, `openrobot_motor.ethercat` runs a
master with pysoem ([interface description](../docs/ethercat_protocol.md)).

    pip install pysoem          # Windows: also install Npcap; Linux: root or CAP_NET_RAW
    python ecat_00_scan.py --list

| File | What it shows | Moves |
|---|---|---|
| `ecat_00_scan.py` | find slaves, read settings and state | no |
| `ecat_11_speed.py` | Speed (DPS) both ways, hold, release | yes |
| `ecat_20_mit.py` | MIT impedance control, slow sine on the output | yes |

The PDO cycle is 2 ms by default (`--period`); shorter cycles are not supported.

## Use in your own code

```python
from openrobot_motor import MotorBus, Motor

with MotorBus.open("slcan", "COM5") as bus, Motor(bus, 1) as m:
    print(m.status())
    m.speed_dps(360)        # Speed (DPS), deg/s, motor side
# leaving the block releases the motor, also on Ctrl+C or an error
```

## Safety

- Leaving a `with Motor(...)` block always sends Motor off.
- When commands stop, the controller's reaction depends on the command and the
  firmware (protocol document, section 8): position and speed commands keep
  running. Keep your own supervision, as `40_safety.py` does.
- Angles and speeds are on the motor side, except in MIT mode (output side).

## Tests

    pip install -e .[test]
    pytest tests
