import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import atan2, sqrt
import math
import time

class ArcFollowerNode(Node):
    def __init__(self):
        super().__init__('arc_follower_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def move_forward(self, distance):
        twist_msg = Twist()
        twist_msg.linear.x = 1.0
        twist_msg.angular.z = 0.0
        self.get_logger().info(f"Publishing twist message: {twist_msg}")
        self.publisher.publish(twist_msg)

    def move_at_anlge(self, linear=0.1, angle=0.1):
        twist_msg = Twist()
        twist_msg.linear.x = linear
        twist_msg.angular.z = angle
        self.get_logger().info(f"Publishing twist message: {twist_msg}")
        self.publisher.publish(twist_msg)


def main(args=None):
    rclpy.init()
    arc_follower = ArcFollowerNode()
    while True:
        arc_follower.move_at_anlge(0.1, 0.2)
        time.sleep(0.1)
    rclpy.spin(arc_follower)
    rclpy.shutdown()

if __name__ == '__main__':\
    main()