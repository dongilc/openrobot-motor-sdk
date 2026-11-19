"""Two actuators on one CAN bus.

    ros2 launch openrobot_motor_hardware two_joints.launch.py id1:=1 id2:=2
    ros2 launch openrobot_motor_hardware two_joints.launch.py mode:=velocity
    ros2 launch openrobot_motor_hardware two_joints.launch.py mode:=velocity_erpm pole_pairs1:=21 pole_pairs2:=21

Modes mit and position start joint_trajectory_controller; modes velocity
and velocity_erpm start velocity_controller. The joints hold where they are on start.
"""
import os

import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def setup(context):
    share = FindPackageShare("openrobot_motor_hardware").perform(context)
    args = {k: LaunchConfiguration(k).perform(context)
            for k in ("interface", "channel", "mode", "id1", "id2", "pole_pairs1", "pole_pairs2")}
    robot = xacro.process_file(os.path.join(share, "description", "two_joints.urdf.xacro"),
                               mappings=args).toxml()
    controllers = os.path.join(share, "config", "controllers.yaml")
    motion = "velocity_controller" if args["mode"].startswith("velocity") else "joint_trajectory_controller"
    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot}]),
        Node(package="controller_manager", executable="ros2_control_node",
             parameters=[controllers], remappings=[("~/robot_description", "/robot_description")],
             output="screen"),
        Node(package="controller_manager", executable="spawner",
             arguments=["joint_state_broadcaster", motion, "--param-file", controllers]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("interface", default_value="socketcan", description="socketcan, pcan or slcan"),
        DeclareLaunchArgument("channel", default_value="can0"),
        DeclareLaunchArgument("mode", default_value="mit",
                              description="mit, velocity, velocity_erpm or position"),
        DeclareLaunchArgument("pole_pairs1", default_value="0", description="velocity_erpm: joint1 motor pole pairs"),
        DeclareLaunchArgument("pole_pairs2", default_value="0", description="velocity_erpm: joint2 motor pole pairs"),
        DeclareLaunchArgument("id1", default_value="1"),
        DeclareLaunchArgument("id2", default_value="2"),
        OpaqueFunction(function=setup),
    ])
