#!/usr/bin/env python3
"""
Test launch file for Johnny 5 base hardware
Includes teleop for manual testing
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('johnny_5_base_hw'),
            'config',
            'base_hw_params.yaml'
        ]),
        description='Path to the configuration file'
    )
    
    # Include base hardware launch
    base_hw_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('johnny_5_base_hw'),
                'launch',
                'base_hw.launch.py'
            ])
        ]),
        launch_arguments={
            'config_file': LaunchConfiguration('config_file'),
            'log_level': 'debug'
        }.items()
    )
    
    # Teleop twist keyboard for manual testing
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        remappings=[
            ('cmd_vel', 'cmd_vel')
        ],
        prefix='xterm -e',  # Run in separate terminal
        output='screen'
    )
    
    return LaunchDescription([
        config_file_arg,
        base_hw_launch,
        teleop_node,
    ])
