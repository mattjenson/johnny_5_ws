#!/usr/bin/env python3
"""
Motor Driver for Pololu Dual VNH5019 Motor Driver Shield
Controls two DC track motors via PWM and direction pins
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
import RPi.GPIO as GPIO
import time


class PololuVNH5019:
    """Hardware interface for Pololu Dual VNH5019 Motor Driver"""
    
    def __init__(self):
        # Motor A (Left Track) GPIO pins
        self.M1_PWM = 18      # PWM pin for motor 1 speed
        self.M1_INA = 23      # Direction pin A for motor 1
        self.M1_INB = 24      # Direction pin B for motor 1
        self.M1_EN = 25       # Enable pin for motor 1 (optional)
        
        # Motor B (Right Track) GPIO pins  
        self.M2_PWM = 19      # PWM pin for motor 2 speed
        self.M2_INA = 5       # Direction pin A for motor 2
        self.M2_INB = 6       # Direction pin B for motor 2
        self.M2_EN = 26       # Enable pin for motor 2 (optional)
        
        # Current sense pins (analog, not used in this basic implementation)
        # self.M1_CS = 1      # Current sense for motor 1
        # self.M2_CS = 0      # Current sense for motor 2
        
        self.setup_gpio()
        
    def setup_gpio(self):
        """Initialize GPIO pins for motor control"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Setup motor 1 pins
        GPIO.setup(self.M1_PWM, GPIO.OUT)
        GPIO.setup(self.M1_INA, GPIO.OUT)
        GPIO.setup(self.M1_INB, GPIO.OUT)
        GPIO.setup(self.M1_EN, GPIO.OUT)
        
        # Setup motor 2 pins
        GPIO.setup(self.M2_PWM, GPIO.OUT)
        GPIO.setup(self.M2_INA, GPIO.OUT)
        GPIO.setup(self.M2_INB, GPIO.OUT)
        GPIO.setup(self.M2_EN, GPIO.OUT)
        
        # Create PWM instances (1000 Hz frequency)
        self.pwm1 = GPIO.PWM(self.M1_PWM, 1000)
        self.pwm2 = GPIO.PWM(self.M2_PWM, 1000)
        
        # Start PWM with 0% duty cycle
        self.pwm1.start(0)
        self.pwm2.start(0)
        
        # Enable both motors
        GPIO.output(self.M1_EN, GPIO.HIGH)
        GPIO.output(self.M2_EN, GPIO.HIGH)
        
    def set_motor_speed(self, motor, speed):
        """
        Set motor speed and direction
        
        Args:
            motor (int): Motor number (1 for left track, 2 for right track)
            speed (float): Speed from -1.0 to 1.0 (negative = reverse)
        """
        # Clamp speed to valid range
        speed = max(-1.0, min(1.0, speed))
        
        # Convert speed to PWM duty cycle (0-100%)
        duty_cycle = abs(speed) * 100
        
        if motor == 1:  # Left track
            if speed >= 0:  # Forward
                GPIO.output(self.M1_INA, GPIO.HIGH)
                GPIO.output(self.M1_INB, GPIO.LOW)
            else:  # Reverse
                GPIO.output(self.M1_INA, GPIO.LOW)
                GPIO.output(self.M1_INB, GPIO.HIGH)
            self.pwm1.ChangeDutyCycle(duty_cycle)
            
        elif motor == 2:  # Right track
            if speed >= 0:  # Forward
                GPIO.output(self.M2_INA, GPIO.HIGH)
                GPIO.output(self.M2_INB, GPIO.LOW)
            else:  # Reverse
                GPIO.output(self.M2_INA, GPIO.LOW)
                GPIO.output(self.M2_INB, GPIO.HIGH)
            self.pwm2.ChangeDutyCycle(duty_cycle)
    
    def stop_all_motors(self):
        """Stop both motors immediately"""
        self.set_motor_speed(1, 0.0)
        self.set_motor_speed(2, 0.0)
        
    def cleanup(self):
        """Clean up GPIO resources"""
        self.stop_all_motors()
        self.pwm1.stop()
        self.pwm2.stop()
        GPIO.cleanup()


class MotorDriverNode(Node):
    """ROS2 node for motor driver interface"""
    
    def __init__(self):
        super().__init__('motor_driver')
        
        # Initialize motor driver hardware
        self.motor_driver = PololuVNH5019()
        
        # Subscribe to motor commands
        self.motor_cmd_sub = self.create_subscription(
            Float64MultiArray,
            'motor_commands',
            self.motor_command_callback,
            10
        )
        
        # Emergency stop subscription
        self.emergency_stop_sub = self.create_subscription(
            Bool,
            'emergency_stop',
            self.emergency_stop_callback,
            10
        )
        
        # Safety timer - stop motors if no commands received for 1 second
        self.safety_timer = self.create_timer(1.0, self.safety_timeout_callback)
        self.last_command_time = time.time()
        
        self.get_logger().info('Motor driver node initialized')
        
    def motor_command_callback(self, msg):
        """
        Process motor command messages
        Expected format: [left_speed, right_speed] where speeds are -1.0 to 1.0
        """
        if len(msg.data) != 2:
            self.get_logger().warn('Motor command must contain exactly 2 values [left, right]')
            return
            
        left_speed = msg.data[0]
        right_speed = msg.data[1]
        
        # Validate speed ranges
        if abs(left_speed) > 1.0 or abs(right_speed) > 1.0:
            self.get_logger().warn(f'Motor speeds must be in range [-1.0, 1.0], got [{left_speed}, {right_speed}]')
            return
            
        # Set motor speeds
        self.motor_driver.set_motor_speed(1, left_speed)   # Left track
        self.motor_driver.set_motor_speed(2, right_speed)  # Right track
        
        self.last_command_time = time.time()
        
        self.get_logger().debug(f'Motor speeds set: left={left_speed:.2f}, right={right_speed:.2f}')
        
    def emergency_stop_callback(self, msg):
        """Handle emergency stop commands"""
        if msg.data:
            self.get_logger().warn('Emergency stop activated!')
            self.motor_driver.stop_all_motors()
            
    def safety_timeout_callback(self):
        """Safety timeout - stop motors if no commands received recently"""
        if time.time() - self.last_command_time > 1.0:
            self.motor_driver.stop_all_motors()
            self.get_logger().debug('Safety timeout - motors stopped')
            
    def destroy_node(self):
        """Clean up when node is destroyed"""
        self.motor_driver.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MotorDriverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in motor driver: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
