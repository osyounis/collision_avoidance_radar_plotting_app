"""
Relative motion calculations for radar plotting.

This modules contains functions which calculates:
    - SRM (Speed of Relative Motion)
    - DRM (Direction of Relative Motion)
    - Line equations for radar plotting

Author: Omar Younis
Date: 28/10/2025    [dd/mm/yyyy]
"""

from datetime import datetime
import numpy as np

from .coordinates import bearing_to_cartesian, cartesian_to_bearing


def find_line_equation(point_1: tuple, point_2: tuple, cartesian: bool = True) -> tuple:
    """
    Finds the slope and intercept of a line passing through two points

    Args:
        point_1: First point(x, y) if cartesian=True, else (bearing, range)
        point_2: Second point(x, y) if cartesian=True, else (bearing, range)
        cartesian: If True, points are already in cartesian coordinates. Defaults to True.

    Returns:
        Tuple: (slope, intercept) for the line equation y = mx + c
    """
    # Checking if we need to convert points to cartesian or not
    if cartesian:
        point_1_x = point_1[0]
        point_1_y = point_1[1]
        point_2_x = point_2[0]
        point_2_y = point_2[1]
    else:
        point_1_x, point_1_y = bearing_to_cartesian(point_1[0], point_1[1])
        point_2_x, point_2_y = bearing_to_cartesian(point_2[0], point_2[1])

    # Calculates the slope and the intercept of the line between the two points
    line_slope = (point_2_y - point_1_y) / (point_2_x - point_1_x)
    line_intercept = point_1_y - (line_slope * point_1_x)

    return line_slope, line_intercept

def find_srm(r_point: tuple, m_point: tuple) -> float:
    """
    Calculates Speed of Relative Motion (SRM).

    Args:
        r_point: First point on radar of the target ship (bearing, range, time_string)
        m_point: Second point on radar of the target ship (bearing, range, time_string)

    Returns:
        Speed of relative motion in knots
    """
    # Converting points to x, y coordinates
    r_x, r_y = bearing_to_cartesian(r_point[0], r_point[1])
    m_x, m_y = bearing_to_cartesian(m_point[0], m_point[1])

    # Finding the time difference between when the two points showed up on radar
    r_time = datetime.strptime(r_point[2], "%H:%M")
    m_time = datetime.strptime(m_point[2], "%H:%M")
    time_delta = (m_time - r_time).total_seconds() / 3600   # Time is in hours

    # Calculating the distance between the 2 points. Then we use the distance
    # and time to find the target's relative speed. For reference, the distance
    # between 2 points is found using the following equation:
    #   sqrt(((X_2 - X_1) ** 2) + (Y_2 - Y_1) ** 2)
    distance = np.sqrt(((m_x - r_x) ** 2) + ((m_y - r_y) **2))
    speed = distance / time_delta

    return speed

def find_drm(r_point: tuple, m_point: tuple) -> float:
    """
    Calculates the Direction of Relative Motion (DRM).

    Args:
        r_point: First point on radar of the target ship (bearing, range, time_string)
        m_point: Second point on radar of the target ship (bearing, range, time_string)

    Returns:
        Bearing in degrees for the direction of relative motion.
    """
    # Converts point to x, y, coordinate system
    r_x, r_y = bearing_to_cartesian(r_point[0], r_point[1])
    m_x, m_y = bearing_to_cartesian(m_point[0], m_point[1])

    # Temporarily moving points r and m together so that point r is at the
    # origin. This makes it easier to find the bearing. The bearing between the
    # two points when r is at the origin will give the relative direction of
    # motion.
    temp_x = m_x - r_x
    temp_y = m_y - r_y
    bearing, _ = cartesian_to_bearing(temp_x, temp_y)

    return bearing
