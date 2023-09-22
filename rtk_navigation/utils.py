import math
import numpy as np

def gps_to_local(origin, point):
    """
    Convert GPS coordinates to local coordinates relative to an origin point.

    Parameters:z
        origin (tuple): A tuple containing the latitude and longitude of the origin point in decimal degrees.
        point (tuple): A tuple containing the latitude and longitude of the point to be converted in decimal degrees.

    Returns:
        tuple: A tuple containing the local coordinates (x, y) of the point relative to the origin point.
    """

    # Convert decimal degrees to radians
    lat1, lon1 = math.radians(origin[0]), math.radians(origin[1])
    lat2, lon2 = math.radians(point[0]), math.radians(point[1])

    # Calculate Earth radius at origin latitude
    R = 6378137 / math.sqrt(1 - 0.006694 * math.pow(math.sin(lat1), 2))

    # Calculate local coordinates
    x = (lon2 - lon1) * R * math.cos(lat1)
    y = (lat2 - lat1) * R

    return (x, y)

def gps_to_local_array(origin, points):
    """
    Convert GPS coordinates to local coordinates relative to an origin point.

    Parameters:
        origin (tuple): A tuple containing the latitude and longitude of the origin point in decimal degrees.
        points (list): A list of tuples, where each tuple contains the latitude and longitude of a point to be converted in decimal degrees.

    Returns:
        list: A list of tuples, where each tuple contains the local coordinates (x, y) of the corresponding point relative to the origin point.
    """

    local_points = []
    for point in points:
        local_point = gps_to_local(origin, point)
        local_points.append(local_point)

    return local_points


def local_to_gps(origin, point):
    """
    Convert local coordinates relative to an origin point to GPS coordinates.

    Parameters:
        origin (tuple): A tuple containing the latitude and longitude of the origin point in decimal degrees.
        local (tuple): A tuple containing the local coordinates (x, y) of the point relative to the origin point.

    Returns:
        tuple: A tuple containing the GPS coordinates (latitude, longitude) of the point.
    """

    # Convert decimal degrees to radians
    lat1, lon1 = math.radians(origin[0]), math.radians(origin[1])

    # Calculate Earth radius at origin latitude
    R = 6378137 / math.sqrt(1 - 0.006694 * math.pow(math.sin(lat1), 2))

    # Calculate latitude and longitude
    lat2 = lat1 + (point[1] / R)
    lon2 = lon1 + (point[0] / (R * math.cos(lat1)))

    # Convert radians to decimal degrees
    lat2, lon2 = math.degrees(lat2), math.degrees(lon2)

    return (lat2, lon2)

def local_to_gps_array(origin, points):
    """
    Convert GPS coordinates to local coordinates relative to an origin point.

    Parameters:
        origin (tuple): A tuple containing the latitude and longitude of the origin point in decimal degrees.
        points (list): A list of tuples, where each tuple contains the local coordinates (x, y) of the point relative to the origin point.

    Returns:
        list: A list of tuples, where each tuple contains the GPS coordinates (latitude, longitude) of the point.
    """

    gps_points = []
    for point in points:
        gps_point = local_to_gps(origin, point)
        gps_points.append(gps_point)

    return gps_points

def distance(coord1, coord2):
    """
    Calculate the great-circle distance (in meters) between two points on the Earth's surface.

    Parameters:
    coord1 (tuple): A tuple representing the first GPS coordinate in the form (latitude, longitude).
    coord2 (tuple): A tuple representing the second GPS coordinate in the form (latitude, longitude).

    Returns:
    float: The distance (in meters) between the two coordinates.

    """
    # Radius of the Earth in meters
    radius_earth = 6378137
    
    # Convert latitude and longitude to radians
    lat1, lon1 = map(math.radians, coord1)
    lat2, lon2 = map(math.radians, coord2)
    
    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    distance = radius_earth * c
    
    return distance

def bearing(coord1, coord2, degree=False):
    """
    Calculate the angle (in degrees) between two GPS points.

    Parameters:
    coord1 (tuple): A tuple representing the first GPS coordinate in the form (latitude, longitude).
    coord2 (tuple): A tuple representing the second GPS coordinate in the form (latitude, longitude).
    degree (bool): If True, the result will be returned in degrees. If False (default), the result will be in radians.

    Returns:
    float: The angle (in degrees or radians) between the two coordinates.
    
    Example:
    >>> bearing((52.205, 0.119), (48.857, 2.351))
    1.373405989846147

    Reference:
    This function is based on the "Haversine formula" and the "Bearing formula" from the "Navigation Formulas" section of
    the "Movable Type Scripts" website: https://www.movable-type.co.uk/scripts/latlong.html
    """
    # Convert latitude and longitude to radians
    lat1, lon1 = map(math.radians, coord1)
    lat2, lon2 = map(math.radians, coord2)
    
    # Calculate the difference in longitude
    dlon = lon2 - lon1
    
    # Calculate the angle in degrees using the arctan2 function
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    rads = math.atan2(y, x)
    angle = math.degrees(rads) if degree else rads
    
    return angle

def displace(start_coord, offset_coord):
    """
    Calculate the new latitude and longitude that are displaced by the given offsets.

    Parameters:
    start_coord (tuple): A tuple representing the starting GPS coordinate in the form (latitude, longitude).
    offset_coord (tuple): A tuple representing the offset in the form (latitude_offset, longitude_offset) in meters.

    Returns:
    tuple: A tuple representing the new latitude and longitude in the form (latitude, longitude).

    """
    # Earth's radius in meters
    radius_earth = 6378137
    
    # Convert starting latitude and longitude to radians
    lat, lon = map(math.radians, start_coord)
    
    # Calculate offset in latitude and longitude
    lat_offset, lon_offset = offset_coord
    dlat = lat_offset / radius_earth
    dlon = lon_offset / (radius_earth * math.cos(math.pi * lat / 180))
    
    # Offset position in decimal degrees
    lat_o = lat + dlat * 180/math.pi
    lon_o = lon + dlon * 180/math.pi
    
    # Convert back to degrees and return as tuple
    return math.degrees(lat_o), math.degrees(lon_o)

def rotate_points(points, theta):
    """
    Rotate a 2D vector by a given angle (in degrees).

    Parameters:
    vector (tuple or list): A tuple or list representing a 2D vector in the form (x, y).
    theta (float): The angle (in degrees) by which to rotate the vector.

    Returns:
    tuple: A tuple representing the rotated vector in the form (x, y).

    """
    # Convert vector to numpy array and transpose
    points = np.array(points).T
    
    # Convert angle to radians
    theta = np.radians(theta)
    
    # Create rotation matrix
    rotation_matrix = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])
    
    # Apply rotation and transpose back to row vector
    output = (rotation_matrix @ points).T
    
    # Return as tuple
    return tuple(output.squeeze())

def rotate_cords(coords, theta):
    """
    Rotate an array of GPS coordinates by a given angle (in degrees).

    Parameters:
    coords (numpy.ndarray): A numpy array of GPS coordinates, where each row is a coordinate in the form (latitude, longitude).
    theta (float): The angle (in degrees) by which to rotate the coordinates.

    Returns:
    numpy.ndarray: A numpy array of rotated GPS coordinates, where each row is a coordinate in the form (latitude, longitude).

    """
    # Convert angle to radians
    theta = np.radians(theta)
    
    # Create rotation matrix
    rotation_matrix = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])
    
    # Convert coordinates to radians
    coords_rad = np.radians(coords)
    
    # Convert coordinates to cartesian coordinates
    x = np.cos(coords_rad[:, 0]) * np.cos(coords_rad[:, 1])
    y = np.cos(coords_rad[:, 0]) * np.sin(coords_rad[:, 1])
    z = np.sin(coords_rad[:, 0])
    xyz = np.vstack((x, y, z)).T
    
    # Apply rotation matrix to cartesian coordinates
    xyz_rotated = (rotation_matrix @ xyz.T).T
    
    # Convert rotated cartesian coordinates back to GPS coordinates
    lat = np.arcsin(xyz_rotated[:, 2] / np.sqrt(xyz_rotated[:, 0]**2 + xyz_rotated[:, 1]**2 + xyz_rotated[:, 2]**2))
    lon = np.arctan2(xyz_rotated[:, 1], xyz_rotated[:, 0])
    
    # Convert back to degrees and return as numpy array
    return np.degrees(np.vstack((lat, lon)).T)