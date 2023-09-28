import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from rclpy.node import Node
from handy_msgs.action import Nav
import rtk_navigation.utils as utils
from sensor_msgs.msg import NavSatFix


positions = [
    (0.0, 0.0),
    (20.0, 0.0),
    (20.0, 2.0),
    (0.0, 2.0),
    (0.0, 4.0),
    (20.0, 4.0),
    (20.0, 6.0),
    (0.0, 6.0),
    (0.0, 8.0),
    (20.0, 8.0),
    (20.0, 10.0),
    (0.0, 10.0),
    (0.0, 0.0)
]

nav_path = [
    (51.987699999800924, 5.6629501482363755),
    (51.98787997324742, 5.6629501482363755),
    (51.98787997324742, 5.662979372740387),
    (51.987699999800924, 5.662979372740387),
    (51.987699999800924, 5.6630085972443975),
    (51.98787997324742, 5.6630085972443975),
    (51.98787997324742, 5.663037821748408),
    (51.987699999800924, 5.663037821748408),
    (51.987699999800924, 5.66306704625242),
    (51.98787997324742, 5.66306704625242),
    (51.98787997324742, 5.663096270756431),
    (51.987699999800924, 5.663096270756431),
    (51.987699999800924, 5.6629501482363755)
]


class NaviAction(Node):
    def __init__(self, gps=False):
        super().__init__('navigation')
        self.dot = None
        self.gps = gps
        self._action_client = ActionClient(self, Nav, '/navigation')
        self.gps_sub = self.create_subscription(NavSatFix, "/rtk/dot", self.dot_callback, 10)
        
    def dot_callback(self, msg):
        if self.dot is None: 
            self.get_logger().info(f"SET TO: {msg}")
            self.dot = msg
            self.send_goal(10)
    
    def send_goal(self, order):
        goal_msg = Nav.Goal()
        if self.gps == True:
            for i in nav_path:
                x, y = utils.gps_to_local((self.dot.latitude, self.dot.longitude), i)
                pose = PoseStamped()
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0
                pose.pose.orientation = Quaternion()
                goal_msg.initial_path.poses.append(pose)
        else:
            for i in positions:
                pose = PoseStamped()
                pose.pose.position.x = i[1]
                pose.pose.position.y = i[0]
                pose.pose.position.z = 0.0
                pose.pose.orientation = Quaternion()
                goal_msg.initial_path.poses.append(pose)
        print(goal_msg)
        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))


def navy(args=None):
    node = NaviAction(gps=True)
    rclpy.spin(node)
    node.shutdown()

def main():
    rclpy.init()
    navy()