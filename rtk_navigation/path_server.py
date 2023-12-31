import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Quaternion, Pose
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
import message_filters
import threading
import numpy as np
from handy_msgs.action import Nav

print("RUNNING PATH SERVER")

pose = Pose()
gps = NavSatFix()
datum = NavSatFix()
points = None
navpath = Path()

class GetThePosition(Node):
    def __init__(self):
        super().__init__('get_the_position')
        self.path = Path()
        self.inited_waypoints = False
        self._odom_sub = message_filters.Subscriber(self, Odometry, '/fix/odom')
        self._gps_sub = message_filters.Subscriber(self, NavSatFix, '/fix/gps')
        self._datum_sub = message_filters.Subscriber(self, NavSatFix, '/fix/datum/gps')
        self.pose_sub = message_filters.ApproximateTimeSynchronizer(
            [self._odom_sub, self._gps_sub, self._datum_sub], 10, slop=10
        )
        self.pose_sub.registerCallback(self.pose_callback)
        self.path_pub = self.create_publisher(Path, "/fix/waypoints", 10)
        self.path_timer = self.create_timer(2.0, self.path_callback)

    def path_callback(self, msg=None):
        # self.get_logger().info('PUBLISHING PATH')
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path.header.frame_id = "map"
        if points is not None and self.inited_waypoints == False:
            for i in points:
                pose = PoseStamped()
                pose.header = self.path.header
                pose.pose.position.x = i.pose.position.x
                pose.pose.position.y = i.pose.position.y
                self.path.poses.append(pose)
            self.inited_waypoints = True
        self.path_pub.publish(self.path)
   
    def pose_callback(self, odom_msg, gps_msg, datum_msg):
        global pose
        global gps
        global datum
        pose.position = odom_msg.pose.pose.position
        pose.orientation = odom_msg.pose.pose.orientation
        gps = gps_msg
        datum = datum_msg


class GoToPosition(Node):
    def __init__(self):
        super().__init__('go_to_position')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ang_then_lin = True
        self.current_pose_ = Point()
        self.target_pose_ = Point()
        self.current_orientation_ = Quaternion()
        self._goal_handle = None
        self._goal_lock = threading.Lock()

        self._action_server = ActionServer(self, Nav, '/navigation', 
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            callback_group=ReentrantCallbackGroup())

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def cancel_callback(self, goal):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                self._goal_handle.abort()
            self._goal_handle = goal_handle
        goal_handle.execute()
     
    async def execute_callback(self, goal_handle):
        global pose
        global gps
        global points
        self.get_logger().info('Executing goal...')
        feedback_msg = Nav.Feedback()
        points = goal_handle.request.initial_path.poses
        if goal_handle.request.abort == True:
            self.stop_moving()
            goal_handle.abort()
            return Nav.Result()
        new_points = []
        for i in points:
            new_points.append((i.pose.position.x, i.pose.position.y))
        for i in points:
            target = i.pose.position
            self.target_pose_ = target
            self.get_logger().info(f"going to: {target}, currently at: {pose.position}")
            while True:
                if goal_handle.is_cancel_requested:
                    self.stop_moving()
                    goal_handle.canceled()
                    return Nav.Result()
                if not goal_handle.is_active:
                    self.stop_moving()
                    return Nav.Result()
                self.current_pose_ = pose.position
                self.current_orientation_ = pose.orientation
                twist = Twist()
                if not self.ang_then_lin:
                    distance, twist.linear.x, twist.angular.z = self.get_nav_params()
                    self.publisher_.publish(twist)
                else:
                    distance, twist.linear.x, twist.angular.z = self.get_nav_params()
                    if twist.angular.z > 0.01:
                        twist.linear.x = 0.0
                    # else:
                    self.publisher_.publish(twist)
                feedback_msg.longitude = gps.longitude
                feedback_msg.latitude = gps.latitude
                goal_handle.publish_feedback(feedback_msg)
                if distance < 0.1: 
                    break
        self.stop_moving()
        goal_handle.succeed()
        result = Nav.Result()
        return result

    def stop_moving(self):
        self.get_logger().info('Stopping...')
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def get_nav_params(self, angle_max=0.4, velocity_max=0.3):
        distance = math.sqrt(
            (self.target_pose_.x - self.current_pose_.x) ** 2 + 
            (self.target_pose_.y - self.current_pose_.y) ** 2)
        # calculate the desired velocity based on the distance to the target position
        velocity = 0.2 * distance
        # calculate the initial heading towards the target position
        preheading = math.atan2(
            self.target_pose_.y - self.current_pose_.y, 
            self.target_pose_.x - self.current_pose_.x
            )
        # calculate the current orientation of the robot using quaternions
        orientation = yaw = math.atan2(2 * (self.current_orientation_.w * self.current_orientation_.z + 
                                            self.current_orientation_.x * self.current_orientation_.y), 
                                       1 - 2 * (self.current_orientation_.y**2 + self.current_orientation_.z**2))
        # calculate the difference between the initial heading and the robot's orientation
        heading = preheading - orientation
        # correct the heading to ensure the robot turns the shortest distance towards the target
        heading_corrected = np.arctan2(np.sin(heading), np.cos(heading))
        # limit the angular error and velocity to the maximum allowable values
        angular = max(-angle_max, min(heading_corrected, angle_max))
        velocity = max(-velocity_max, min(velocity, velocity_max))
        return distance, velocity, angular



def main(args=None):
    rclpy.init(args=args)
    try:
        getpos=GetThePosition()
        gotopos = GoToPosition()
        gotopos.log_file = False
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(getpos)
        executor.add_node(gotopos)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            getpos.destroy_node()
            gotopos.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':\
    main()