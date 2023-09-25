import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
import message_filters

def distance(coord1, coord2):
    radius_earth = 6_367_449
    lat1, lon1 = map(math.radians, coord1)
    lat2, lon2 = map(math.radians, coord2)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    distance = radius_earth * c
    return distance

def bearing(coord1, coord2):
    lat1, lon1 = math.radians(coord1[0]), math.radians(coord1[1])
    lat2, lon2 = math.radians(coord2[0]), math.radians(coord2[1])
    diffLong = lon2 - lon1
    x = math.sin(diffLong) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(diffLong))
    initial_bearing = math.atan2(x, y)
    initial_bearing = math.degrees(initial_bearing)
    bearing = (initial_bearing + 360) % 360
    radians = math.radians(bearing)
    radians = math.pi - radians
    return radians

def odom_from_two_gps(gps1, gps2):
    odom = Odometry()
    dis = distance([gps1.latitude, gps1.longitude], [gps2.latitude, gps2.longitude])
    rad = bearing([gps1.latitude, gps1.longitude], [gps2.latitude, gps2.longitude])
    odom.pose.pose.position.x = dis * math.cos(rad)
    odom.pose.pose.position.y = dis * math.sin(rad)
    return odom

def generate_half_ellipse(start_point, end_point, num_points=10):
    # Calculate the center of the ellipse
    center_x = (start_point[0] + end_point[0]) / 2
    center_y = (start_point[1] + end_point[1]) / 2
    # Calculate the major and minor axes lengths
    major_axis = math.dist(start_point, end_point) / 2
    # Calculate the angle between start_point and end_point
    angle = math.atan2(end_point[1] - start_point[1], end_point[0] - start_point[0])
    # Generate points
    half_ellipse_points = [
        (
            round(center_x + major_axis * math.cos(math.pi * i / (num_points - 1) + angle), 2),
            round(center_y + major_axis * math.sin(math.pi * i / (num_points - 1) + angle), 2)
        )
        for i in range(num_points)
    ]
    # Sort points by angle
    half_ellipse_points.sort(key=lambda point: math.atan2(point[1] - center_y, point[0] - center_x))
    return half_ellipse_points

def quaternion_to_euler(q):
    (x, y, z, w) = (q.x, q.y, q.z, q.w)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    # return [yaw, pitch, roll]
    # return yaw, math.degrees(yaw)
    return math.degrees(yaw)


class Transformerr(Node):
    def __init__(self, args):
        super().__init__("rtk_navigation")
        self.target_position = None

        self.odom_sub = message_filters.Subscriber(self, Odometry, '/rtk/odom')
        self.fix_sub = message_filters.Subscriber(self, NavSatFix, '/rtk/fix')
        self.dot_sub = message_filters.Subscriber(self, NavSatFix, '/rtk/dot')
        self.odom_sub = message_filters.ApproximateTimeSynchronizer([self.odom_sub, self.fix_sub, self.dot_sub], 10, slop=10)
        self.odom_sub.registerCallback(self.sunc_callback)

    #     self.odom_sub = self.create_subscription(Odometry, "/rtk/odom", self.navigation_callback, 10)

    # def navigation_callback(self, msg):
    #     # self.get_logger().info('PUBLISHING ODOM')
    #     position = msg.pose.pose.position
    #     orientation = msg.pose.pose.orientation
    #     self.get_logger().info(f"x: {position.x}, y: {position.y}, r: {quaternion_to_euler(orientation)}")

    def sunc_callback(self, odom_sub, fix_sub, dot_sub):
        if self.target_position is None: 
            self.target_position = fix_sub
            return
        odom = odom_from_two_gps(dot_sub, self.target_position)
        self.get_logger().info(f"odom: {odom.pose.pose.position.x}, {odom.pose.pose.position.y}")

def main(args=None):
    rclpy.init(args=args)
    navfix = Transformerr(args=args)
    rclpy.spin(navfix)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

