import math
from nav_msgs.msg import Odometry

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

def main(args=None):
    print("STARTING RTK NAVIGATION")

if __name__ == '__main__':
    main()

