import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import RPi.GPIO as GPIO
import threading
import time

class IRNavigation(Node):

    def __init__(self):
        super().__init__('ir_navigation')
        self.ir_pin = 21 # Change this to the GPIO pin connected to your IR sensor
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.ir_pin, GPIO.IN)
        self.home_pose = PoseStamped()  # Set the home position coordinates
        self.home_pose.header.frame_id = 'map'
        self.home_pose.pose.position.x = 0.0
        self.home_pose.pose.position.y = 0.0
        self.home_pose.pose.orientation.w = 1.0
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.goal_published = False

    def timer_callback(self):
        ir_value = GPIO.input(self.ir_pin)
        if ir_value == 1 and not self.goal_published:
            self.get_logger().info('IR Sensor: Triggered, waiting for 20 seconds before sending the robot to the home position.')
            thread = threading.Thread(target=self.delayed_goal_publish)
            thread.start()

    def delayed_goal_publish(self):
        time.sleep(10)  # Add 20-second delay
        consistent_readings = 0
        num_checks = 10  # Check the sensor value 10 times

        for _ in range(num_checks):
            ir_value = GPIO.input(self.ir_pin)
            if ir_value == 1:
                consistent_readings += 1
            time.sleep(0.5)  # Sleep for 0.5 seconds between checks

        if consistent_readings == num_checks:
            self.publish_goal()

    def publish_goal(self):
        self.goal_publisher.publish(self.home_pose)
        self.goal_published = True

def main(args=None):
    rclpy.init(args=args)

    ir_navigation = IRNavigation()

    rclpy.spin(ir_navigation)

    ir_navigation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()