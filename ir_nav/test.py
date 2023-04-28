import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
import random
import math


class RandomGoalPublisher(Node):

    def __init__(self):
        super().__init__('random_goal_publisher')
        qos = QoSProfile(depth=1)
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', qos)
        self.timer_period = 5.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.publish_random_goal)
        self.goal_published = False

    def publish_random_goal(self):
        if not self.goal_published:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = random.uniform(-2, 2)
            goal_pose.pose.position.y = random.uniform(-2, 2)
            yaw = random.uniform(-math.pi, math.pi)
            goal_pose.pose.orientation.z = math.sin(yaw / 2)
            goal_pose.pose.orientation.w = math.cos(yaw / 2)

            self.get_logger().info(
                f'Publishing goal: x={goal_pose.pose.position.x}, y={goal_pose.pose.position.y}, yaw={yaw}')
            self.publisher.publish(goal_pose)
            self.goal_published = True


def main(args=None):
    rclpy.init(args=args)

    random_goal_publisher = RandomGoalPublisher()

    executor = MultiThreadedExecutor()
    rclpy.spin(random_goal_publisher, executor=executor)

    random_goal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()