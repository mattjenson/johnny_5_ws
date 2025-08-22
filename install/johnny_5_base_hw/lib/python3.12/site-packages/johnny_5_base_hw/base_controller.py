#!/usr/bin/env python3
"""
Base Controller for Johnny 5 tracked drive system
Converts geometry_msgs/Twist commands to individual motor speeds for differential drive
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math


class BaseController(Node):
    """
    Converts Twist commands to differential drive motor speeds for tracked vehicle
    """
    
    def __init__(self):
        super().__init__('base_controller')
        
        # Declare parameters with defaults
        self.declare_parameter('wheel_separation', 0.35)  # Distance between tracks in meters
        self.declare_parameter('wheel_radius', 0.05)      # Effective wheel radius in meters
        self.declare_parameter('max_linear_speed', 1.0)   # Max linear speed in m/s
        self.declare_parameter('max_angular_speed', 2.0)  # Max angular speed in rad/s
        self.declare_parameter('motor_max_rpm', 100.0)    # Max motor RPM
        
        # Get parameters
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.motor_max_rpm = self.get_parameter('motor_max_rpm').value
        
        # Convert max RPM to max wheel speed (m/s)
        self.max_wheel_speed = (self.motor_max_rpm * 2 * math.pi * self.wheel_radius) / 60.0
        
        # Subscribe to velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publish motor commands
        self.motor_cmd_pub = self.create_publisher(
            Float64MultiArray,
            'motor_commands',
            10
        )
        
        self.get_logger().info(f'Base controller initialized:')
        self.get_logger().info(f'  Wheel separation: {self.wheel_separation:.3f} m')
        self.get_logger().info(f'  Wheel radius: {self.wheel_radius:.3f} m')
        self.get_logger().info(f'  Max linear speed: {self.max_linear_speed:.3f} m/s')
        self.get_logger().info(f'  Max angular speed: {self.max_angular_speed:.3f} rad/s')
        self.get_logger().info(f'  Max wheel speed: {self.max_wheel_speed:.3f} m/s')
        
    def cmd_vel_callback(self, msg):
        """
        Convert Twist message to motor speeds for differential drive
        
        For a differential drive robot:
        - left_wheel_speed = linear_vel - (angular_vel * wheel_separation / 2)
        - right_wheel_speed = linear_vel + (angular_vel * wheel_separation / 2)
        """
        
        # Extract linear and angular velocities
        linear_vel = msg.linear.x  # Forward/backward velocity (m/s)
        angular_vel = msg.angular.z  # Rotational velocity (rad/s)
        
        # Clamp velocities to maximum values
        linear_vel = max(-self.max_linear_speed, min(self.max_linear_speed, linear_vel))
        angular_vel = max(-self.max_angular_speed, min(self.max_angular_speed, angular_vel))
        
        # Calculate wheel speeds for differential drive
        left_wheel_speed = linear_vel - (angular_vel * self.wheel_separation / 2.0)
        right_wheel_speed = linear_vel + (angular_vel * self.wheel_separation / 2.0)
        
        # Convert wheel speeds to motor speed percentages (-1.0 to 1.0)
        left_motor_speed = left_wheel_speed / self.max_wheel_speed
        right_motor_speed = right_wheel_speed / self.max_wheel_speed
        
        # Clamp motor speeds to valid range
        left_motor_speed = max(-1.0, min(1.0, left_motor_speed))
        right_motor_speed = max(-1.0, min(1.0, right_motor_speed))
        
        # Create and publish motor command message
        motor_cmd_msg = Float64MultiArray()
        motor_cmd_msg.data = [left_motor_speed, right_motor_speed]
        
        self.motor_cmd_pub.publish(motor_cmd_msg)
        
        self.get_logger().debug(
            f'Cmd: lin={linear_vel:.2f} ang={angular_vel:.2f} -> '
            f'Motors: L={left_motor_speed:.2f} R={right_motor_speed:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = BaseController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in base controller: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
