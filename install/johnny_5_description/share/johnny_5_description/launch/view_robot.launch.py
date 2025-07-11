import os
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import (
    get_package_share_directory, PackageNotFoundError
)

def have_display() -> bool:
    """Return True if an X/Wayland display is available."""
    return bool(os.environ.get('DISPLAY', ''))

def make_joint_state_publisher():
    """GUI if present *and* a display is available; else CLI version."""
    if have_display():
        try:
            get_package_share_directory('joint_state_publisher_gui')
            return Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='joint_state_publisher_gui',
                output='screen'
            )
        except PackageNotFoundError:
            pass  # fall through to CLI version
    # Headless fallback
    return Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

def generate_launch_description():
    pkg_share = get_package_share_directory('johnny_5_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'johnny_5.xacro')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro', ' ', xacro_file]),
                value_type=str
            )
        }],
        output='screen'
    )

    joint_state_publisher = make_joint_state_publisher()

    # Only launch RViz if a display is present
    nodes = [robot_state_publisher, joint_state_publisher]

    if have_display():
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ))

    return LaunchDescription(nodes)