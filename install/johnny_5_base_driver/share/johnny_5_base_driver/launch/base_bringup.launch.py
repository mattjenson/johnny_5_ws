#!/usr/bin/env python3
"""
Base bring-up for Johnny-5 on ROS 2 Jazzy (Pi 5).

• Runs xacro → robot_state_publisher
• Starts the Python base_driver node
"""

import os
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ── Locate the Xacro file ────────────────────────────────────────────
    desc_pkg = get_package_share_directory("johnny_5_description")
    xacro_file = os.path.join(desc_pkg, "urdf", "johnny_5.xacro")

    # ── robot_state_publisher (needs ONE long string) ────────────────────
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": ParameterValue(
                Command(['xacro', ' ', xacro_file]),
                value_type=str
            )
        }],
        output="screen",
    )

    # ── Python motor driver ─────────────────────────────────────────────
    base_driver = Node(
        package="johnny_5_base_driver",
        executable="base_driver",
        output="screen",
    )

    return LaunchDescription([robot_state_pub, base_driver])