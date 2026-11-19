# OpenRobot EtherCAT interface

English | [한국어](ethercat_protocol.ko.md)

How an EtherCAT master drives OpenRobot SPN motor controllers fitted with the
EtherCAT interface (LAN9252, SOES slave stack). The command codes are the
same as the [OpenRobot CAN protocol](can_protocol.md); the values travel in
fixed process data (PDO) instead of request/reply frames.

Checked against firmware 261119. ESI file:
[ethercat/OpenRobot_MC.xml](../ethercat/OpenRobot_MC.xml).

## 1. Slave

| Item | Value |
|---|---|
| Vendor ID | 0x00524F42 |
| Product code | 0x00000001 |
| Mailbox | CoE (SDO), 128-byte mailbox |
| RxPDO (master → slave) | 0x1600 → object 0x7000, SM2, 17 bytes, fixed mapping |
| TxPDO (slave → master) | 0x1A00 → object 0x6000, SM3, 20 bytes, fixed mapping |
| Synchronisation | free run, master cycle 2 ms (500 Hz) or longer |
| Motor ID | SDO 0x2070:03 (the CAN ID of the same board) |

All values are little-endian.

### Bring-up

1. Scan the bus (INIT).
2. Request PRE-OP explicitly and wait for it; some masters leave the slave in
   INIT after the scan. SDOs work from PRE-OP on.
3. Map the process data, request SAFE-OP, start the cyclic exchange.
4. Request OP. Send `control_mode = 0x80` (Motor off) first.

### Cycle time

**Supported cycle: 2 ms (500 Hz) or longer.** The slave runs in free run and
polls the ESC between motor control interrupts, so how many master cycles it
answers depends on the CPU left over while the motor turns. Measured on
SPN-MC V2, motor turning at up to 3600 deg/s, 2 ms master cycle:

| Firmware | Process data handled |
|---|---|
| 261101 | TxPDO refreshed at about 40 Hz |
| 261102 and later | every cycle |

A shorter cycle is not supported: while the motor turns the slave skips
cycles. A skipped cycle keeps the previous TxPDO values and applies the RxPDO
of the next one; nothing is lost from the bus side (no working counter
errors).

## 2. RxPDO, master → slave (17 bytes)

| Byte | Field | Type | Scale |
|---|---|---|---|
| 0 | control_mode | u8 | command code, section 4 |
| 1..4 | target_position | i32 | × 10000 |
| 5..8 | target_velocity | i32 | × 10000 |
| 9..12 | target_torque | i32 | × 10000 |
| 13..14 | kp | u16 | × 100 |
| 15..16 | kd | u16 | × 1000 |

A frame with `control_mode` outside 0x80..0xC0 (other than 0) is dropped as
corrupt. `control_mode = 0` is accepted but does nothing: the previous command
keeps running and the PDO timeout does not fire (section 5).

**Rate limit per PDO cycle** (defence against corrupted SPI frames): the
slave moves `target_torque` by at most 5.0 (× 10000 = 50000 raw) and
`target_velocity` by at most 10.0 (100000 raw) per received PDO. A step is
reached over several cycles, e.g. 360 deg/s takes 36 cycles.

## 3. TxPDO, slave → master (20 bytes)

| Byte | Field | Type | Scale, unit |
|---|---|---|---|
| 0 | control_mode | u8 | active command code (0 = none) |
| 1 | fault_code | u8 | fault, see [CAN protocol section 7](can_protocol.md#7-faults) |
| 2..5 | actual_position | i32 | × 10000 deg, motor multi-turn angle; **in MIT mode output-side, from the MIT zero** |
| 6..9 | actual_velocity | i32 | × 10000 deg/s, **motor side in every mode** |
| 10..13 | actual_torque | i32 | × 10000 **A** (motor current, not Nm) |
| 14..15 | bus_voltage | u16 | × 100 V |
| 16 | temp_motor | i8 | °C |
| 17 | temp_mosfet | i8 | °C |
| 18..19 | encoder_pos | u16 | single-turn encoder, 14 bit |

Torque on the output ≈ current × output-side Kt (read SDO 0x2020:03).

## 4. Commands (`control_mode`)

| Code | Name | Fields used | Meaning |
|---|---|---|---|
| 0x80 | Motor off | | release; also clears a Big Error fault |
| 0x81 | Motor stop | | Speed (DPS) 0: holds the present position |
| 0x88 | Motor start | | duty 0 |
| 0xA1 | Current | target_torque: A; kp: damping 0..100 | current (torque) control, damped when kp > 0 |
| 0xA2 | Speed (DPS, position-tracked) | target_velocity: deg/s, motor side | see [CAN 0xA2](can_protocol.md); eRPM is not available over EtherCAT |
| 0xA3 | Position, direct | target_position: deg, motor multi-turn | no speed profile, large steps limited as for [CAN 0xA3](can_protocol.md#0xa3-position-control-direct); Big Error check |
| 0xA4 | Position with speed limit | target_position: deg; target_velocity: limit 1..25000 deg/s | profiled move; an out-of-range limit is ignored |
| 0xA5 | Duty | target_torque: -1..1 | PWM duty |
| 0xC0 | MIT | target_position rad, target_velocity rad/s, target_torque Nm, kp Nm/rad, kd Nm·s/rad | output side; torque = kp (p - p_act) + kd (v - v_act) + t_ff |

The values are taken every cycle; there is no separate "enter" step as on
CAN. Set the MIT zero with SDO 0x2050:05 = 1 (the present output position
becomes 0 rad, not stored).

### Big Error

0xA3 and 0xC0 (with kp > 0) compare the command with the actual position on
the output side. If they differ by more than the threshold (SDO 0x2030:08,
default 180°) the slave latches fault 26 and ignores every command except
0x80, which clears it. Start MIT at the present position (set the MIT zero,
then command p = 0) and move from there.

## 5. Loss of communication

| Situation | Slave behaviour |
|---|---|
| No PDO for longer than the PDO timeout (SDO 0x2030:07, default 1000 ms) while EtherCAT is the command source | stops the motor **in every mode, also position** (action below) |
| Master leaves OP | outputs are zeroed, no further PDO is processed, so the timeout above fires |
| PDOs keep arriving with `control_mode = 0` | the previous command continues; the timeout never fires |
| 0xA2 Speed (DPS) | also stops on its own when the target is not refreshed for 0.5 s |

Action on a PDO timeout:

| Board | Action |
|---|---|
| SPN-MC V2 / V2R2, D14, D17_D20, VESCAT HIGH V2R2 / V2R3 / V2R4, firmware 261119 or later | the command watchdog action set with `or_wd` (default damping, see [CAN protocol section 8](can_protocol.md#8-safety-notes)) |
| other boards, older firmware | releases the motor (fault stop) |

On CAN, by contrast, position and speed commands run on when commands stop
(CAN protocol section 8). PDO timeout state is shown in SDO 0x2030:02.

After a timeout, PDOs resume control by themselves; send 0x80 first so the
slave starts from a known state.

## 6. SDO objects

Writes take effect at once. Settings are stored to flash later, when the motor
is idle (released), so a write never stalls the control loop.

| Index | Sub | Name | Type | Access |
|---|---|---|---|---|
| 0x2000 | 1, 2 | Current_Kp, Current_Ki | REAL32 | RW |
| 0x2001 | 1..5 | Speed_Kp, _Ki, _Kd, _Kd_filter, Speed_min_erpm | REAL32 | RW |
| 0x2002 | 1..4 | Position_Kp, _Ki, _Kd, _Kd_filter | REAL32 | RW |
| 0x2010 | 1 | Encoder_offset | REAL32 | RW |
| 0x2010 | 2 | Encoder_CPR | U32 | RO |
| 0x2020 | 1, 2 | Max_current, Abs_max_current (clamped to the hardware limit) | REAL32 | RW |
| 0x2020 | 3 | Torque_constant: reads the output-side Kt; a write sets the motor-side Kt (0 = from the motor setup) | REAL32 | RW |
| 0x2020 | 4, 5 | Motor_poles (U8), Flux_linkage | | RW |
| 0x2030 | 1 | Fault_code; write 0 to clear | U8 | RW |
| 0x2030 | 2 | PDO_timeout_flag | U8 | RO |
| 0x2030 | 3, 4 | Temp_FET, Temp_motor | REAL32 | RO |
| 0x2030 | 5, 6 | Temp_limit_FET, Temp_limit_motor | REAL32 | RW |
| 0x2030 | 7 | PDO_timeout_ms | U32 | RW |
| 0x2030 | 8 | Big_error_threshold, output deg | REAL32 | RW |
| 0x2040 | 1, 2 | Control_mode, Control_source | U8 | RO |
| 0x2050 | 1..4 | MIT_V_max, MIT_T_max, MIT_Kt_motor, MIT_Gear_ratio | REAL32 | RW |
| 0x2050 | 5 | MIT_set_zero: write 1 | U8 | RW |
| 0x2060 | 1, 2 | DPS_Vmax, DPS_Amax | REAL32 | RW |
| 0x2070 | 1, 2 | FW_version, HW_version | string | RO |
| 0x2070 | 3 | Motor_ID (refused while the motor runs) | U8 | RW |
| 0x2090 | 1..6 | encoder health (some builds) | | RO |
| 0x20A0 | 1..7 | output encoder (some builds) | | RO |
| 0x2100 | 1..4 | VESC packet tunnel (motor tool) | | |

## 7. Safety

- The slave stops the motor when PDOs stop (section 5), but a master that
  keeps sending stale commands keeps the motor moving. Supervise your own
  loop and send 0x80 on any error.
- Start every session with 0x80, and leave it with 0x80 before stopping the
  cyclic exchange.
- Keep an independent emergency stop on the motor power.
