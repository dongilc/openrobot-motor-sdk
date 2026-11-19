# ROS 2 (ros2_control)

`openrobot_motor_hardware` is a ros2_control hardware interface for OpenRobot
motor controllers. One hardware component drives every actuator on one CAN
bus. Checked with ROS 2 Jazzy on Ubuntu 24.04.
Protocol reference: [../docs/can_protocol.md](../docs/can_protocol.md).

## Build

The package uses the SDK's C++ headers (`../cpp/include`), so build it from
inside the SDK tree, or link the SDK into your workspace:

    mkdir -p ~/ws/src && cd ~/ws/src
    ln -s /path/to/openrobot_motor_sdk/ros2/openrobot_motor_hardware .
    cd ~/ws && colcon build --symlink-install
    source install/setup.bash

## Bring up the CAN bus

    sudo ip link set can0 up type can bitrate 1000000

PCAN-USB and CANable (gs_usb firmware) appear as `can0` through SocketCAN.
`interface:=pcan` (PEAK PCAN-Basic for Linux) and `interface:=slcan
channel:=/dev/ttyACM0` also work.

## Run the example

Two actuators, IDs 1 and 2:

    ros2 launch openrobot_motor_hardware two_joints.launch.py id1:=1 id2:=2

The joints hold where they are when the controller starts. Then send a
trajectory (output shaft, rad):

    ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
      control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [joint1, joint2], \
      points: [{positions: [0.5, -0.5], time_from_start: {sec: 2}}, \
               {positions: [0.0, 0.0], time_from_start: {sec: 4}}]}}"

Velocity mode:

    ros2 launch openrobot_motor_hardware two_joints.launch.py mode:=velocity id1:=1 id2:=2
    ros2 topic pub --once /velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.0, -1.0]}"

The same with Speed (eRPM, VESC speed loop) instead of Speed (DPS); give the
motor pole pairs:

    ros2 launch openrobot_motor_hardware two_joints.launch.py mode:=velocity_erpm pole_pairs1:=21 pole_pairs2:=21 id1:=1 id2:=2

Stopping the launch (Ctrl+C) releases every motor.

## Joint parameters

| Parameter | Meaning | Default |
|---|---|---|
| `id` | CAN ID of the controller (1..32) | required |
| `mode` | `mit`, `velocity`, `velocity_erpm` or `position` | `mit` |
| `kp`, `kd` | MIT gains, Nm/rad and Nm s/rad | 10, 0.5 |
| `max_velocity` | position mode speed limit, rad/s output | 1.0 |
| `pole_pairs` | velocity_erpm mode: motor pole pairs (rad/s to eRPM) | required there |
| `zero_on_activate` | MIT: the position at start becomes 0 rad | false |

Hardware parameters: `interface` (`socketcan`, `pcan`, `slcan`), `channel`,
`timeout_ms` (reply timeout, default 5), `max_missed` (missed replies in a
row before the hardware stops, default 3).

| Mode | Command interfaces | Controller command |
|---|---|---|
| `mit` | position, velocity, effort (feed-forward) | 0xC0 MIT, torque = kp (p - p_act) + kd (v - v_act) + effort |
| `velocity` | velocity | 0xA2 Speed (DPS, position-tracked) |
| `velocity_erpm` | velocity | 0xA2 Speed (eRPM, VESC speed loop); released first when coming from another mode |
| `position` | position | 0xA4 position with `max_velocity` (absolute multi-turn angle: send targets relative to where the joint is) |

Every joint reports position (rad), velocity (rad/s) and effort (Nm =
current x Kt), all on the output shaft. The gear ratio and Kt are read from
the controller at start. In MIT mode the position is relative to the MIT
zero and limited to +/-12.56 rad; in the other modes it is the multi-turn
angle divided by the gear ratio.

## Timing

Every request waits for its reply, so one control cycle costs one CAN round
trip per joint in MIT mode and two in velocity and position modes (the
second reads the multi-turn angle). Keep `joints x round trip` well inside
the controller period. Through USB/IP (PCAN-USB attached to WSL) a round
trip measured about 1.2 ms; two joints at 200 Hz ran, with an occasional
6 ms cycle reported as an overrun.

## Safety

- A missed reply three times in a row, or a fault notice (0xBF) from any
  controller, puts the hardware component into error, which releases every
  motor.
- Ctrl+C on the launch deactivates the hardware and releases every motor.
  A killed process (SIGKILL, power loss of the computer) cannot.
- When commands stop, position and speed commands keep running (protocol
  document, section 8; MIT switches to damping after 100 ms with firmware
  261119 or later). If the computer hangs in velocity or position mode, the motors keep
  going: keep an emergency stop on the motor power.
