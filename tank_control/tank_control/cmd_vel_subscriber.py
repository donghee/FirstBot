import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

import RPi.GPIO as GPIO
import time

class CmdVelSubscriber(Node):

    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.left_motor = GPIO.PWM(12, 50)
        self.right_motor = GPIO.PWM(13, 50)
        self.left_motor.start(0)
        self.right_motor.start(0)

    def listener_callback(self, msg):
        #self.get_logger().info('I heard: "%f, %f, %f"' % (msg.linear.x, msg.linear.y, msg.linear.z))
        if msg.linear.x >= 0.5:
            # forward
            #GPIO.output(12, GPIO.HIGH)
            #GPIO.output(13, GPIO.HIGH)
            self.left_motor.ChangeDutyCycle(100.0)
            self.right_motor.ChangeDutyCycle(100.0)

            GPIO.output(21, GPIO.LOW)
            GPIO.output(20, GPIO.HIGH)
            GPIO.output(23, GPIO.LOW)
            GPIO.output(24, GPIO.HIGH)

        if msg.linear.x <= -0.5:
            # backward
            #GPIO.output(12, GPIO.HIGH)
            #GPIO.output(13, GPIO.HIGH)
            self.left_motor.ChangeDutyCycle(100.0)
            self.right_motor.ChangeDutyCycle(100.0)
            
            GPIO.output(21, GPIO.HIGH)
            GPIO.output(20, GPIO.LOW)
            GPIO.output(23, GPIO.HIGH)
            GPIO.output(24, GPIO.LOW)
            
        if msg.angular.z >= 0.5:
            #GPIO.output(12, GPIO.HIGH)
            #GPIO.output(13, GPIO.HIGH)
            self.left_motor.ChangeDutyCycle(50.0)
            self.right_motor.ChangeDutyCycle(50.0)
            
            GPIO.output(21, GPIO.LOW)
            GPIO.output(20, GPIO.HIGH)
            GPIO.output(23, GPIO.HIGH)
            GPIO.output(24, GPIO.LOW)

        if msg.angular.z <= -0.5:
            #GPIO.output(12, GPIO.HIGH)
            #GPIO.output(13, GPIO.HIGH)
            self.left_motor.ChangeDutyCycle(50.0)
            self.right_motor.ChangeDutyCycle(50.0)
            
            GPIO.output(21, GPIO.HIGH)
            GPIO.output(20, GPIO.LOW)
            GPIO.output(23, GPIO.LOW)
            GPIO.output(24, GPIO.HIGH)

        time.sleep(0.1)
        #GPIO.output(12, GPIO.LOW)
        #GPIO.output(13, GPIO.LOW)
        self.left_motor.ChangeDutyCycle(0.0)
        self.right_motor.ChangeDutyCycle(0.0)
 

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

    minimal_subscriber = CmdVelSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    GPIO.cleanup()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
