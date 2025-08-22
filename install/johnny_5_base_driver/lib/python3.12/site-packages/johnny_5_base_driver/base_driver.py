#!/usr/bin/env python3
"""
Johnny-5 tracked-base driver (Pi 5, Ubuntu 24.04)
  • subscribes to /cmd_vel
  • drives BTS7960 H-bridge via PWM on two GPIOs
  • publishes /joint_states and /odom
"""

import math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import lgpio                               # Pi-5 friendly GPIO lib

# ─────────────── USER-TUNEABLE CONSTANTS ──────────────────────── #
L_PWM = 18         # BCM pin numbers (HW-PWM capable but we’ll use sw PWM)
R_PWM = 23
PWM_FREQ =  2000   # 2 kHz is plenty for a motor; lgpio sw-PWM stays solid
MAX_WHEEL_RPM = 25
WHEEL_RADIUS  = 0.0315   # metres  (63 mm sprocket)
WHEEL_BASE    = 0.25     # metres  (centre-to-centre)
# ───────────────────────────────────────────────────────────────── #

MAX_RAD_S = MAX_WHEEL_RPM * 2 * math.pi / 60

class BaseDriver(Node):
    def __init__(self):
        super().__init__("base_driver")
        # open the default gpiochip (RP1 exposes several; 0 works for PWM pins)
        self.chip = lgpio.gpiochip_open(0)
        for pin in (L_PWM, R_PWM):
            lgpio.gpio_claim_output(self.chip, pin, 0)

        # ROS2 comms
        self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, 10)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.odom_pub  = self.create_publisher(Odometry, "/odom", 10)
        self.tf_pub    = TransformBroadcaster(self)

        self.last_time = self.get_clock().now()
        self.x = self.y = self.yaw = 0.0
        self.left_pos = self.right_pos = 0.0
        self.left_cmd = self.right_cmd = 0.0
        # NEW: status logging throttling
        self._last_status_log = time.time()

        self.timer = self.create_timer(0.02, self.update)  # 50 Hz loop

    # ──────────── callbacks ──────────────────────────────────────
    def cmd_cb(self, msg: Twist):
        v, w = msg.linear.x, msg.angular.z
        self.left_cmd  = (v - w*WHEEL_BASE/2) / WHEEL_RADIUS
        self.right_cmd = (v + w*WHEEL_BASE/2) / WHEEL_RADIUS
        self.left_cmd  = max(-MAX_RAD_S, min(MAX_RAD_S, self.left_cmd))
        self.right_cmd = max(-MAX_RAD_S, min(MAX_RAD_S, self.right_cmd))
        # NEW: immediate log of received command and resulting wheel targets
        self.get_logger().info(
            f"/cmd_vel rx: v={v:.3f} m/s w={w:.3f} rad/s -> left={self.left_cmd:.2f} rad/s right={self.right_cmd:.2f} rad/s"
        )

    # ─────────── main loop ───────────────────────────────────────
    def update(self):
        now = self.get_clock().now()
        dt  = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # drive H-bridge (simple sign + duty)
        self._set_pwm(L_PWM,  self.left_cmd)
        self._set_pwm(R_PWM,  self.right_cmd)

        # naive dead-reckon odom (no encoders yet)
        v_l = self.left_cmd  * WHEEL_RADIUS
        v_r = self.right_cmd * WHEEL_RADIUS
        v   = (v_l + v_r) / 2.0
        w   = (v_r - v_l) / WHEEL_BASE
        self.x   += v * math.cos(self.yaw) * dt
        self.y   += v * math.sin(self.yaw) * dt
        self.yaw += w * dt

        # joint states
        self.left_pos  += self.left_cmd  * dt
        self.right_pos += self.right_cmd * dt
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = ["left_track_joint", "right_track_joint"]
        js.position = [self.left_pos, self.right_pos]
        js.velocity = [self.left_cmd, self.right_cmd]
        self.joint_pub.publish(js)

        # odom msg
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id  = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw/2)
        odom.pose.pose.orientation.w = math.cos(self.yaw/2)
        odom.twist.twist.linear.x  = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

        # TF
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_pub.sendTransform(t)

        # NEW: periodic status log (1 Hz)
        if time.time() - self._last_status_log > 1.0:
            self._last_status_log = time.time()
            self.get_logger().info(
                f"status: pose=({self.x:.2f},{self.y:.2f},{self.yaw:.2f} rad) v={v:.2f} m/s w={w:.2f} rad/s wheel_cmds=({self.left_cmd:.2f},{self.right_cmd:.2f})"
            )

    # ─────────── helper ──────────────────────────────────────────
    def _set_pwm(self, pin: int, rad_s: float):
        duty_pct = min(abs(rad_s) / MAX_RAD_S, 1.0) * 100.0  # lgpio wants 0–100
        lgpio.tx_pwm(self.chip, pin, PWM_FREQ, duty_pct)

def main():
    rclpy.init()
    rclpy.spin(BaseDriver())
    rclpy.shutdown()

if __name__ == "__main__":
    main()