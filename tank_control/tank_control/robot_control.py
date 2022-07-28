import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry

from tf2_ros import TransformBroadcaster

import RPi.GPIO as GPIO
import time
import math

def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr

    return q

class TankControl(Node):

    def __init__(self):
        super().__init__('tank_control')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher = self.create_publisher( Odometry, 'wheel/odom', 10)

        self.pub_timer = self.create_timer(0.04, self.pub_callback)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.robot_state_theta = 0.0
        self.robot_state_x_pos = 0.0
        self.robot_state_y_pos = 0.0
        self.robot_state_v = 0.0
        self.robot_state_w = 0.0

        self.robot_linear = 0.0
        self.robot_angular = 0.0

        self.left_motor_ticks = 0.0
        self.right_motor_ticks = 0.0
        self.p_left_motor_ticks = 0.0
        self.p_right_motor_ticks = 0.0

        self.left_motor = GPIO.PWM(12, 50)
        self.right_motor = GPIO.PWM(13, 50)
        self.left_motor.start(0)
        self.right_motor.start(0)

    def listener_callback(self, msg):
        #self.get_logger().info('I heard: "%f, %f, %f"' % (msg.linear.x, msg.linear.y, msg.linear.z))
        if msg.linear.x >= 0.5:
            # forward
            self.left_motor.ChangeDutyCycle(100.0)
            self.right_motor.ChangeDutyCycle(100.0)

            GPIO.output(21, GPIO.LOW)
            GPIO.output(20, GPIO.HIGH)
            GPIO.output(23, GPIO.LOW)
            GPIO.output(24, GPIO.HIGH)

            #GPIO.output(21, GPIO.HIGH)
            #GPIO.output(20, GPIO.LOW)
            #GPIO.output(23, GPIO.HIGH)
            #GPIO.output(24, GPIO.LOW)

            self.left_motor_ticks += 10.0
            self.right_motor_ticks += 10.0
            self.robot_linear = 0.10
            self.robot_angular = 0.0

        if msg.linear.x <= -0.5:
            # backward
            self.left_motor.ChangeDutyCycle(100.0)
            self.right_motor.ChangeDutyCycle(100.0)

            GPIO.output(21, GPIO.HIGH)
            GPIO.output(20, GPIO.LOW)
            GPIO.output(23, GPIO.HIGH)
            GPIO.output(24, GPIO.LOW)

            #GPIO.output(21, GPIO.LOW)
            #GPIO.output(20, GPIO.HIGH)
            #GPIO.output(23, GPIO.LOW)
            #GPIO.output(24, GPIO.HIGH)

            self.left_motor_ticks += -10.0
            self.right_motor_ticks += -10.0
            self.robot_linear = -0.10
            self.robot_angular = 0.0

        if msg.angular.z >= 0.5:
            self.left_motor.ChangeDutyCycle(20.0)
            self.right_motor.ChangeDutyCycle(20.0)
 
            GPIO.output(21, GPIO.HIGH)
            GPIO.output(20, GPIO.LOW)
            GPIO.output(23, GPIO.LOW)
            GPIO.output(24, GPIO.HIGH)

            self.left_motor_ticks += -2.0
            self.right_motor_ticks += 2.0
            self.robot_linear = 0.0
            self.robot_angular = 0.10

        if msg.angular.z <= -0.5:
            self.left_motor.ChangeDutyCycle(20.0)
            self.right_motor.ChangeDutyCycle(20.0)
            
            GPIO.output(21, GPIO.LOW)
            GPIO.output(20, GPIO.HIGH)
            GPIO.output(23, GPIO.HIGH)
            GPIO.output(24, GPIO.LOW)

            self.left_motor_ticks += 2.0
            self.right_motor_ticks += -2.0
            self.robot_linear = 0.0
            self.robot_angular = -0.10

        time.sleep(0.1)
        self.robot_linear = 0.0
        self.robot_angular = 0.0
        self.left_motor.ChangeDutyCycle(0.0)
        self.right_motor.ChangeDutyCycle(0.0)

    def update_odometry(self):
        ROBOT_WHEEL_RADIUS = 0.025
        ROBOT_MOTOR_PPR = 20.0
        ROBOT_WHEEL_SEPARATION = 0.70

        dl_ticks = self.left_motor_ticks - self.p_left_motor_ticks
        dr_ticks = self.right_motor_ticks - self.p_right_motor_ticks
        self.p_left_motor_ticks = self.left_motor_ticks 
        self.p_right_motor_ticks = self.right_motor_ticks

        delta_l = (2.0 * math.pi * ROBOT_WHEEL_RADIUS * dl_ticks) / ROBOT_MOTOR_PPR
        delta_r = (2.0 * math.pi * ROBOT_WHEEL_RADIUS * dr_ticks) / ROBOT_MOTOR_PPR
        delta_center = (delta_l + delta_r) / 2.0

        self.robot_state_x_pos += delta_center * math.cos(self.robot_state_theta)
        self.robot_state_y_pos += delta_center * math.sin(self.robot_state_theta)
        self.robot_state_theta += (delta_r - delta_l) / ROBOT_WHEEL_SEPARATION
        self.robot_state_v = self.robot_linear
        self.robot_state_w = self.robot_angular
        #self.get_logger().info('x_theta "%f"' %  self.robot_state_theta)

    def pub_callback(self):
        self.update_odometry()

        #robot_orientation = quaternion_from_euler(0, 0, self.robot_state_theta)
        robot_orientation = quaternion_from_euler(0, 0, self.robot_state_theta)
        timestamp = self.get_clock().now().to_msg()

        # transforms
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.robot_state_x_pos
        t.transform.translation.y = self.robot_state_y_pos
        t.transform.translation.z = 0.0325
        t.transform.rotation = robot_orientation

        # odometry twist
        odom_msg = Odometry()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.header.stamp = timestamp
        odom_msg.pose.pose.position.x = self.robot_state_x_pos
        odom_msg.pose.pose.position.y = self.robot_state_y_pos
        odom_msg.pose.pose.position.z = 0.325
        odom_msg.pose.pose.orientation = robot_orientation
        odom_msg.twist.twist.linear.x = self.robot_state_v
        odom_msg.twist.twist.angular.z = self.robot_state_w

        # broadcast and publish
        #self.tf_broadcaster.sendTransform(t)
        self.publisher.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    GPIO.setup(12, GPIO.OUT)
    GPIO.setup(21, GPIO.OUT)
    GPIO.setup(20, GPIO.OUT)
    GPIO.setup(23, GPIO.OUT)
    GPIO.setup(24, GPIO.OUT)
    GPIO.setup(13, GPIO.OUT)

    minimal_subscriber = TankControl()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    GPIO.cleanup()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
