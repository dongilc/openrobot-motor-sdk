# Motor Check (Android)

A field check app for OpenRobot motor controllers: plug a CAN adapter into
the phone, find the motors on the bus, read their state, faults and
settings, and jog a motor briefly.
Protocol reference: [../docs/can_protocol.md](../docs/can_protocol.md).

## What you need

- An Android phone with USB host (OTG), Android 7.0 or newer
- An slcan adapter (CANable or compatible) and an OTG cable or adapter
- The motors on a 1 Mbit/s bus with 120 ohm at both ends

PEAK PCAN adapters are not supported yet on Android.

## What it does

| Part | Shows or does |
|---|---|
| CAN adapter | finds USB serial adapters, opens the bus at 1 Mbit/s |
| Motors on the bus | scans IDs 1..32 |
| Motor | mode, temperature, current, speed, motor and output angle, 10 times a second |
| Faults | the last seven fault codes with names; Clear fault (0x9B) |
| Settings | gear ratio, Kt, MIT ranges, current limit (read only) |
| Jog | five modes, below |
| MOTOR OFF | releases the selected motor (0xC2, 0x80) |

| Jog mode | Command | Use |
|---|---|---|
| DPS | 0xA2 mode 0 | Speed (DPS, position-tracked), deg/s: turns while a button is held; releasing stops, then releases |
| eRPM | 0xA2 mode 1 | Speed (eRPM, VESC speed loop), electrical RPM: same buttons |
| Position | 0xA4 | each press moves one step (output degrees) from where the shaft is, with a speed limit; the motor holds there |
| Current | 0xA1 | applies a current (torque) while a button is held, at most 30 % of the motor current limit |
| MIT hold | 0xC1, 0xC3, 0xC0 | holds the output where it is with kp and kd; sliders set the targets p (±180° from the hold point), v (±10 rad/s) and t_ff (bounded like Current) |

Jog stays disabled until you tick "The shaft is free to turn". Changing the
jog mode or the motor releases the motor. Leaving the app ends DPS, eRPM,
Current and MIT hold; a Position hold stays until MOTOR OFF. Fault notices (0xBF) from any motor
appear under Messages.

## Build

Android SDK 35 and JDK 17:

    ./gradlew assembleDebug          # app/build/outputs/apk/debug/app-debug.apk
    ./gradlew testDebugUnitTest      # protocol checked against the Python reference

On Windows use `gradlew.bat`. To keep build output out of a synced folder:
`./gradlew assembleDebug -PbuildRoot=/path/outside`.

`tools/gen_vectors.py` regenerates the unit test vectors from
`python/openrobot_motor/protocol.py`.

## Safety

When commands stop, the controller's reaction depends on the command and
the firmware (protocol document, section 8). With firmware 261119 or later,
current and MIT commands switch to damping after 100 ms by default, while
position and speed commands keep running; older firmware keeps running every command.
If the phone or the cable is disconnected during a DPS jog, the motor keeps
turning. Keep an emergency stop on the motor power.
