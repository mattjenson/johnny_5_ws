#!/usr/bin/env python3
"""
Odometry Publisher for Johnny 5 tracked drive system
Estimates robot pose based on motor commands and publishes odometry
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import tf_transformations
import math
import time


class OdometryPublisher(Node):
    """
    Publishes odometry information based on motor speeds for differential drive
    """
    
    def __init__(self):
        super().__init__('odometry_publisher')
        
        # Declare parameters
        self.declare_parameter('wheel_separation', 0.35)  # Distance between tracks
        self.declare_parameter('wheel_radius', 0.05)      # Effective wheel radius
        self.declare_parameter('motor_max_rpm', 100.0)    # Max motor RPM
        self.declare_parameter('publish_tf', True)        # Whether to publish TF
        self.declare_parameter('odom_frame', 'odom')      # Odometry frame name
        self.declare_parameter('base_frame', 'base_link') # Base frame name
        
        # Get parameters
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.motor_max_rpm = self.get_parameter('motor_max_rpm').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        
        # Convert max RPM to max wheel speed
        self.max_wheel_speed = (self.motor_max_rpm * 2 * math.pi * self.wheel_radius) / 60.0
        
        # Robot pose (x, y, theta)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Robot velocities
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        # Timing
        self.last_time = time.time()
        self.last_motor_speeds = [0.0, 0.0]
        
        # Subscribe to motor commands to estimate odometry
        self.motor_cmd_sub = self.create_subscription(
            Float64MultiArray,
            'motor_commands',
            self.motor_command_callback,
            10
        )
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for publishing odometry
        self.odom_timer = self.create_timer(0.05, self.publish_odometry)  # 20 Hz
        
        self.get_logger().info('Odometry publisher initialized')
        self.get_logger().info(f'  Publishing TF: {self.publish_tf}')
        self.get_logger().info(f'  Frames: {self.odom_frame} -> {self.base_frame}')
        
    def motor_command_callback(self, msg):
        """Process motor commands to update odometry estimation"""
        if len(msg.data) != 2:
            return
            
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt > 0.0:
            # Get motor speeds (-1.0 to 1.0)
            left_motor_speed = msg.data[0]
            right_motor_speed = msg.data[1]
            
            # Convert to wheel speeds (m/s)
            left_wheel_speed = left_motor_speed * self.max_wheel_speed
            right_wheel_speed = right_motor_speed * self.max_wheel_speed
            
            # Calculate robot velocities from differential drive kinematics
            self.linear_vel = (left_wheel_speed + right_wheel_speed) / 2.0
            self.angular_vel = (right_wheel_speed - left_wheel_speed) / self.wheel_separation
            
            # Update robot pose
            self.update_pose(dt)
            
        self.last_time = current_time
        self.last_motor_speeds = [msg.data[0], msg.data[1]]
        
    def update_pose(self, dt):
        """Update robot pose based on current velocities"""
        # Simple integration (could be improved with more sophisticated methods)
        delta_x = self.linear_vel * math.cos(self.theta) * dt
        delta_y = self.linear_vel * math.sin(self.theta) * dt
        delta_theta = self.angular_vel * dt
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
    def publish_odometry(self):
        """Publish odometry message and optionally TF transform"""
        current_time = self.get_clock().now()
        
        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        
        # Set position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Set orientation (convert theta to quaternion)
        quat = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion()
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        
        # Set velocities
        odom_msg.twist.twist.linear.x = self.linear_vel
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = self.angular_vel
        
        # Set covariance (simplified - should be tuned based on actual robot)
        odom_msg.pose.covariance[0] = 0.1   # x
        odom_msg.pose.covariance[7] = 0.1   # y
        odom_msg.pose.covariance[35] = 0.1  # yaw
        odom_msg.twist.covariance[0] = 0.1  # linear x
        odom_msg.twist.covariance[35] = 0.1 # angular z
        
        # Publish odometry
        self.odom_pub.publish(odom_msg)
        
        # Publish TF transform if enabled
        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = current_time.to_msg()
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.translation.z = 0.0
            
            tf_msg.transform.rotation = odom_msg.pose.pose.orientation
            
            self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = OdometryPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in odometry publisher: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
