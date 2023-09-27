import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from rclpy.node import Node
from handy_msgs.action import Nav
from nav_msgs.msg import Path

positions = [(5.0, 5.0), (10.0, 5.0), (5.0, 0.0), (-5.0, 0.0) ,(-10.0, 5.0), (-5.0, 5.0), (0.0, 0.0)]

class NaviAction(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Nav, '/navigation')
        self.path = Path()
        

    def send_goal(self, order):
        goal_msg = Nav.Goal()
        for i in positions:
            pose = PoseStamped()
            pose.pose.position.x = i[0]
            pose.pose.position.y = i[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation = Quaternion()
            goal_msg.initial_path.poses.append(pose)
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


def main(args=None):
    rclpy.init(args=args)
    action_client = NaviAction()
    action_client.send_goal(10)
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
