#!/usr/bin/env python3
"""
Launch file for Johnny 5 base hardware interface
Starts motor driver, base controller, and odometry publisher
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('johnny_5_base_hw'),
            'config',
            'base_hw_params.yaml'
        ]),
        description='Path to the configuration file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error)'
    )
    
    # Get launch configurations
    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')
    
    # Motor driver node
    motor_driver_node = Node(
        package='johnny_5_base_hw',
        executable='motor_driver',
        name='motor_driver',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=2.0,
        output='screen'
    )
    
    # Base controller node
    base_controller_node = Node(
        package='johnny_5_base_hw',
        executable='base_controller',
        name='base_controller',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=2.0,
        output='screen'
    )
    
    # Odometry publisher node
    odometry_publisher_node = Node(
        package='johnny_5_base_hw',
        executable='odometry_publisher',
        name='odometry_publisher',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=2.0,
        output='screen'
    )
    
    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        log_level_arg,
        
        LogInfo(
            condition=None,
            msg='Starting Johnny 5 base hardware interface...'
        ),
        
        motor_driver_node,
        base_controller_node,
        odometry_publisher_node,
    ])
