# OpenRobot CAN protocol

English | [한국어](can_protocol.ko.md)

Command set of OpenRobot motor controllers on classic CAN. Every value in this
document is taken from the firmware CAN handler (`comm_can.c`,
`decode_msg_sid()`), firmware 261119.

Current boards: SPN-MC V2 / V2R2, SPN-MC D14, SPN-MC D17_D20,
VESCAT HIGH V2R2 / V2R3 / V2R4. With firmware 261119 or later these have the
command watchdog (section 8) and the new position control (0xA3, 0xA4).
Where other boards or older firmware behave differently, it is noted.

## 1. Bus

| Item | Value |
|---|---|
| Frame | Classic CAN 2.0A, standard 11-bit ID, DLC 8 |
| Bit rate | 1 Mbit/s factory default (changeable on the board) |
| Request ID | `0x140 + motor_id` |
| Reply ID | `0x140 + motor_id` (same ID as the request) |
| Byte order | Little-endian, except the packed MIT frame (0xC0) |
| `data[0]` | Command code; the reply echoes it |

- One request gives one reply, sent right after the command is handled.
- A board answers only its own `motor_id`. There is no broadcast ID.
- Requests and replies share the ID. Use `data[0]` to match a reply to its
  request, and do not treat your own transmitted frame as a reply if the
  adapter echoes transmitted frames.
- Unused reply bytes are 0.
- Wait for the reply (or a timeout, 10 ms is ample) before sending the next
  command to the same motor. Commands to different motors may be interleaved.

### Finding motors

Send `0x92` (read multi-turn angle) to every ID from 1 to 32 (or the range in
use) and collect the replies. This is safe: `0x92` only reads.

## 2. Units and the common status reply

Motion commands (0x80, 0x81, 0x88, 0xA1 to 0xA6) and `0x9C` reply with the same
status layout:

| Byte | Type | Meaning | Scale |
|---|---|---|---|
| 0 | u8 | command echo | |
| 1 | i8 | motor temperature | 1 °C |
| 2..3 | i16 | motor current (torque axis, signed) | 33/2048 A (0.01611 A) |
| 4..5 | i16 | motor shaft speed | 1 deg/s |
| 6..7 | u16 | motor shaft angle, single turn | 0..16383 = 0..360° |

All values are on the **motor side** (before the gearbox), in the user
direction set on the board. The 0xC0 MIT reply is the exception (section 6).

The motor temperature is valid only when a motor thermistor is fitted.

## 3. Command summary

| Code | Name | Changes motion | Writes flash |
|---|---|---|---|
| 0x80 | Motor off (release) | yes | |
| 0x81 | Motor stop (hold) | yes | |
| 0x88 | Motor start | yes | |
| 0xA1 | Current (torque) control | yes | |
| 0xA2 | Speed control: DPS (position-tracked) or eRPM (VESC speed loop) | yes | |
| 0xA3 | Position control (direct) | yes | |
| 0xA4 | Position control with speed limit | yes | |
| 0xA5 | Duty control | yes | |
| 0xA6 | Current brake | yes | |
| 0x9C | Read status | | |
| 0x9D | Read control mode and phase currents | | |
| 0x92 | Read multi-turn angle | | |
| 0x93 | Read output encoder (some builds) | | |
| 0x90 | Read encoder | | |
| 0x9E | Read encoder status | | |
| 0x9B | Clear fault | | |
| 0xB0 | Read fault history | | |
| 0x30 | Read position PID | | |
| 0x31 | Write position PID to RAM | | |
| 0x32 | Write position PID to flash | | yes |
| 0xB1 | Read current limits | | |
| 0xB2 | Write current limits to flash | | yes |
| 0x91 | Write encoder zero offset | | yes |
| 0x19 | Set current angle as zero | | yes |
| 0xC0 | MIT control | yes | |
| 0xC1 | MIT enter | yes | |
| 0xC2 | MIT exit (release) | yes | |
| 0xC3 | MIT set zero (runtime) | | |
| 0xC4 | Read MIT parameters | | |
| 0xC5 | Write MIT parameters to flash | | yes |
| 0xBF | Fault notice (sent by the board) | | |

Flash writes stall the controller for a moment. Do not send them while the
motor is running, and do not send them periodically.

## 4. Motion commands

Every command below replies with the common status (section 2).

### 0x80 Motor off
`[0x80, 0, 0, 0, 0, 0, 0, 0]` → the motor is released (no torque, free shaft).

### 0x81 Motor stop
`[0x81, 0, ...]` → speed control at 0 deg/s, i.e. the motor actively holds still.

### 0x88 Motor start
`[0x88, 0, ...]` → duty 0. If another control mode was active it is released
first.

### 0xA1 Current (torque) control

| Byte | Type | Meaning |
|---|---|---|
| 4..5 | i16 | current command, 33/2048 A per LSB |
| 6..7 | i16 | damping, 0.01 per LSB, clamped to 0..100; 0 = pure current control |

Torque ≈ Kt × current. The sign follows the user direction.

### 0xA2 Speed control (DPS, eRPM)

| Byte | Type | Meaning |
|---|---|---|
| 1 | u8 | 0 = **DPS**, 1 = **eRPM** |
| 4..7 | i32 | DPS: motor speed, 0.01 deg/s per LSB; eRPM: electrical RPM, 1 per LSB |

One command, two different controllers. The names DPS and eRPM are used for
them throughout this SDK.

| | Speed (DPS, position-tracked) | Speed (eRPM, VESC speed loop) |
|---|---|---|
| How | an acceleration-limited profile advances a target angle at the commanded speed; the position PID (0x30) follows that angle | the VESC built-in speed PID |
| Unit | deg/s, motor side | electrical RPM = mechanical RPM × pole pairs |
| Under load, at low speed | no accumulated angle error: the shaft follows the moving target angle | holds the speed only; an angle lost to a load is not made up |
| Speed 0 | holds the present position | holds speed 0, no position |
| Commands stop | keeps turning (section 8) | released about 1 s later |
| SDK | `speed_dps()` | `speed_erpm()` |

The pole pair count is not readable over this protocol; read it from the
motor setup to convert eRPM.

**eRPM is ignored while another motion mode is active.** The board keeps
running DPS, position, MIT and the other OpenRobot modes until it is
released, and an eRPM command does not end them (measured: eRPM right after
DPS kept the DPS speed). Send 0x80 first; the SDK's `speed_erpm()` does so
when the previous motion command was not a release or eRPM.

### 0xA3 Position control (direct)

| Byte | Type | Meaning |
|---|---|---|
| 4..7 | i32 | multi-turn motor angle target, 0.01° per LSB |

The target goes to the position loop without a speed profile. Large steps are
limited:

- A step larger than 60° (motor side) is carried out as a speed-limited move
  at 10000 deg/s, then handed back to the position loop. Threshold and speed
  are set with the terminal command `or_limit`.
- With the new position control on (`or_ctl`), the threshold drops to the
  step that would need about 3 A of position-error current, if that is
  smaller, and the move follows the 0xA4 trapezoid. 0xA3 commands less than 50 ms apart are treated as a stream:
  the controller measures their period and replays them on its own clock,
  about 1.5 periods behind, so arrival jitter does not reach the motor.

For single point-to-point moves, use 0xA4. The new position control is
described in [Inertia Tune and the New Position Control](https://github.com/dongilc/openrobot-motor-tool/blob/master/docs/position_control.md).

### 0xA4 Position control with speed limit

| Byte | Type | Meaning |
|---|---|---|
| 2..3 | i16 | speed limit, 1 deg/s per LSB, valid 1..25000 |
| 4..7 | i32 | multi-turn motor angle target, 0.01° per LSB |

A speed limit outside 1..25000 is ignored without notice: the board still
replies but does not move.

With the new position control on, the move is a trapezoid with the
acceleration set by `or_ctl amax` (default 25000 deg/s²).

### 0xA5 Duty control

| Byte | Type | Meaning |
|---|---|---|
| 4..5 | i16 | duty cycle, 1/10000 per LSB (−10000..10000 = −100..100 %) |

### 0xA6 Current brake

| Byte | Type | Meaning |
|---|---|---|
| 4..5 | i16 | brake current, 33/2048 A per LSB (magnitude is used) |

Opposes rotation and lets go once the shaft has stopped. It does not hold
position.

## 5. Read and configuration commands

### 0x9C Read status
Request `[0x9C, 0...]`. Reply: common status (section 2). Does not change
anything.

### 0x9D Read control mode and phase currents

| Byte | Type | Meaning |
|---|---|---|
| 1 | u8 | control mode (table below) |
| 2..3 | i16 | phase A current, 1/64 A |
| 4..5 | i16 | phase B current, 1/64 A |
| 6..7 | i16 | phase C current, 1/64 A |

Control mode: 0 none, 1 released, 2 current brake, 3 duty, 4 current,
5 damped current, 6 position, 7 position (multi-turn), 8 speed (timeout),
9 speed, 10 position with speed limit, 11 trajectory, 12 MIT.

### 0x92 Read multi-turn angle

| Byte | Type | Meaning |
|---|---|---|
| 1..7 | i56 | motor multi-turn angle, 0.01° per LSB (sign-extend bit 55) |

### 0x93 Read output encoder
Answered only by builds with an output-shaft encoder; other builds do not reply.

| Byte | Type | Meaning |
|---|---|---|
| 1..2 | u16 | output single-turn angle, 360/65536° per LSB |
| 3..6 | i32 | output multi-turn angle, 0.01° per LSB |
| 7 | u8 | bit 0 encoder valid, bit 1 calibration table loaded, bit 2 dual table loaded |

### 0x90 Read encoder

| Byte | Type | Meaning |
|---|---|---|
| 1 | u8 | encoder error bits |
| 2..3 | u16 | angle after zero offset, 14-bit |
| 4..5 | u16 | raw angle, 14-bit |
| 6..7 | u16 | zero offset, 14-bit |

### 0x9E Read encoder status
For encoders that report diagnostics.

| Byte | Type | Meaning |
|---|---|---|
| 1..2 | u16 | air gap, µm |
| 3..4 | u16 | signal level |
| 5 | u8 | read error rate, 0.5 % per LSB |
| 6 | u8 | CRC status |
| 7 | u8 | bit 0 error, bit 1 warning |

### 0x9B Clear fault
Reply byte 1 = fault code after clearing (0 when cleared). Also clears a
command-watchdog trip (section 8); the watchdog action (damping) goes on until
the next motion command, so a joint under gravity is not dropped.

### 0xB0 Read fault history
Reply bytes 1..7 = the last seven fault codes (section 7).

### 0x30, 0x31, 0x32 Position PID

| Byte | Type | Meaning |
|---|---|---|
| 2..3 | u16 | Kp × 1000 |
| 4..5 | u16 | Ki × 100000 |
| 6..7 | u16 | Kd × 100000 |

0x30 reads, 0x31 writes to RAM (lost at power-off), 0x32 writes to flash. The
write commands echo the values. While the new position control is on, the
position loop takes its gains from the measured inertia instead.

### 0xB1, 0xB2 Current limits

| Byte | Type | Meaning |
|---|---|---|
| 1 | u8 | gate driver over-current mode |
| 2..3 | u16 | motor current limit, 0.01 A |
| 4..5 | u16 | absolute current limit (fault trip), 0.01 A |
| 6..7 | u16 | battery current limit, 0.01 A |

0xB1 reads, 0xB2 writes to flash. The negative limits are set to minus the
same value. Keep the board's factory values unless the motor requires
otherwise.

### 0x91 Write encoder zero offset
Bytes 6..7 = new offset, 14-bit. Stored in flash; the multi-turn angle is reset.
Reply bytes 6..7 = the offset actually stored.

### 0x19 Set current angle as zero
Takes the present shaft angle as the new zero and stores it in flash. The
multi-turn angle is reset. Reply bytes 6..7 = the stored offset (14-bit).

## 6. MIT mode

Impedance control on the **output side** (after the gearbox):

    torque = Kp × (p_des − p) + Kd × (v_des − v) + t_ff

Sequence: `0xC1` enter → `0xC0` periodically → `0xC2` exit.

### 0xC0 MIT control (bit-packed, big-endian)

| Bits | Field | Range | Width |
|---|---|---|---|
| data[1], data[2] | p_des | −12.56..12.56 rad | 16 |
| data[3], data[4] high 4 bits | v_des | −v_max..v_max rad/s | 12 |
| data[4] low 4 bits, data[5] | Kp | 0..500 Nm/rad | 12 |
| data[6] | Kd | 0..5 Nm·s/rad × Kd scale | 8 |
| data[7] | t_ff | −t_max..t_max Nm | 8 |

Encoding: `raw = round((x − min) / (max − min) × (2^bits − 1))`, clamped.
v_max and t_max are stored on the board (0xC4); the factory values are 45 rad/s
and 33 Nm.

Zero is not exactly representable in the signed fields: with an even span the
nearest code to 0 lies half a step off, and `x = 0` encodes as the middle code
2^(bits−1) (p 32768, v 2048, t_ff 128).

- SPN-MC V2 firmware 261109 and later decode the middle code as exactly 0.
  Every other code decodes as before, so hosts need no change.
- Older firmware (and other boards) decode it as +max / (2^bits − 1): t_ff 0 is
  +t_max / 255 (+0.13 Nm with t_max 33 Nm), v_des 0 is +v_max / 4095. With
  Kp = Kd = 0 that small torque turns a free output slowly.

Reply:

| Byte | Type | Meaning |
|---|---|---|
| 1 | i8 | motor temperature, °C |
| 2..3 | i16 | motor current, 33/2048 A |
| 4..5 | i16 | **output** speed, 1 deg/s |
| 6..7 | i16 | **output** position, 12.56/32767 rad per LSB, relative to the MIT zero |

### 0xC1 MIT enter
Arms the drive and **holds the output where it is** with the position
controller (or keeps the reference of a position command already running).
The hold lasts until the first `0xC0`, which switches to MIT control at once.
This differs from the original MIT (Mini Cheetah) protocol, where enter starts
from an all-zero command and the joint goes limp: here a joint under gravity
does not sag between enter and the first command.

Reply byte 1 = the Kd scale in force (trust it only if 0xC4 reports a non-zero
scale; older firmware leaves it undefined).

### 0xC2 MIT exit
Releases the motor: no torque, the output turns freely. A joint carrying a load
under gravity drops, so bring it to rest first or switch to a position command
instead of exiting.

### 0xC3 MIT set zero
Bytes 6..7 = i16 offset, 0.01° output side. The present position becomes this
value (0 = present position is zero). Runtime only, not stored.

### 0xC4 Read / 0xC5 Write MIT parameters

| Byte | Type | Write (0xC5) | Read reply |
|---|---|---|---|
| 1 | u8 | v_max, rad/s | v_max |
| 2 | u8 | t_max, Nm | t_max |
| 3..4 | u16 | motor-side Kt, 0.001 Nm/A (0 = from motor setup) | **output-side** Kt = Kt × gear ratio, 0.001 Nm/A |
| 5..6 | u16 | gear ratio × 100 | gear ratio × 100 |
| 7 | u8 | Kd scale 1..8, 0 = keep | Kd scale in force (0 = firmware without scale, use 1) |

Both reply with code 0xC4. A write while the motor is running is refused; the
reply then carries the old values, so compare them against what you sent.

## 7. Faults

When a fault stops the motor, the board sends once, unprompted:

    ID 0x140 + motor_id, data = [0xBF, fault_code, 0, 0, 0, 0, 0, 0]

A latched fault stays until cleared with 0x9B.

| Code | Fault |
|---|---|
| 0 | none |
| 1 | over voltage |
| 2 | under voltage |
| 3 | gate driver |
| 4 | over current |
| 5 | over temperature, power stage |
| 6 | over temperature, motor |
| 7 | gate driver over voltage |
| 8 | gate driver under voltage |
| 9 | MCU under voltage |
| 10 | watchdog reset at boot |
| 11 | encoder (SPI) |
| 12 | encoder amplitude low (sin/cos) |
| 13 | encoder amplitude high (sin/cos) |
| 14 | flash corruption |
| 15..17 | current sensor offset high, phase 1..3 |
| 18 | unbalanced currents |
| 19 | brake |
| 20..22 | resolver LOT, DOS, LOS |
| 23 | flash corruption, app configuration |
| 24 | flash corruption, motor configuration |
| 25 | encoder no magnet |
| 26 | position error too large |

## 8. Safety notes

- **Loss of communication (command watchdog).** Position and speed commands
  are meant to be sent once and persist, so they keep running. Current, brake,
  duty and MIT control are watched: if no motion command arrives for 100 ms,
  the board stops by itself.

  | Last command | When commands stop |
  |---|---|
  | 0xA3, 0xA4 position | holds the target position |
  | 0x81 stop | holds the present position (speed 0) |
  | 0xA2 Speed (DPS) | **keeps turning** at that speed |
  | 0xA2 Speed (eRPM) | released about 1 s after the last command |
  | 0xA1 current, 0xA6 brake, 0xA5 duty, 0x88 start | watched: after 100 ms, damping |
  | 0xC0 MIT | watched: after 100 ms, damping |
  | 0x80, 0xC2 | stays released |

  - *Damping*: torque against the output speed only, `t = −kd × v` (default
    kd 2.0 Nm·s/rad, output side). A moving output comes to rest without a
    jolt; a joint under gravity sinks slowly instead of dropping. At standstill
    the torque is zero.
  - After a trip the board ignores motion commands until it is cleared:
    0x9B keeps damping until the next motion command, 0xC1 holds the present
    position (as MIT enter always does), 0x80 releases the motor. Status reads
    (0x9C, 0x92, ...) do not count as commands and do not keep the watchdog
    alive.
  - The timeout (0 = off), the watched modes (position and speed can be added)
    and the action (damping, damping then hold once stopped, or release) are
    set with the terminal command `or_wd` and stored on the board. EtherCAT
    uses the same action when its PDO timeout expires.
  - Boards without the watchdog (see the top of this document) and older
    firmware keep executing the last command in every mode.

  The host should still supervise its own loop and send 0x80 (or a position
  hold) on any error; every example in this repository does so. For position
  and speed control, a fault that cuts the bus itself (connector, cable,
  adapter) is not caught by the watchdog: provide an independent way to cut
  motor power (emergency stop).
- **Power supply.** Fast moves, reversals and braking draw short current peaks
  from the supply and push energy back into it. A bench supply with a current
  limit, or one that cannot sink current, can then sag or rise and trip the
  board (under voltage, gate driver fault). Size the supply for the peaks or
  use a battery, and set the input current limits (`l_in_current_max`,
  `l_in_current_min` in the motor configuration) to what the supply can
  deliver and absorb.
- Send 0x80 before closing the program, including on Ctrl+C.
- Position and angle values are motor side except in MIT mode. Divide by the
  gear ratio for the output shaft.
