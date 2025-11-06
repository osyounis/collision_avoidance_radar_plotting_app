"""
True motion calculations for radar plotting.

This module calculates:
    - DTM (Direction of True Motion)
    - STM (Speed of True Motion)
    - N/C (New Course)
    - N/S (New Speed)

Author: Omar Younis
Date: 28/10/2025    [dd/mm/yyyy]
"""

from datetime import datetime
import numpy as np

from .coordinates import bearing_to_cartesian, cartesian_to_bearing


def find_dtm(m_point: tuple, e_point: tuple) -> float:
    """
    Calculate Direction of True Motion (DTM).

    Args:
        m_point: Second point on radar of the target ship (bearing, range, time_string)
        e_point: E point (bearing, range)

    Returns:
        Bearing in degrees for DTM
    """
    # Converting points to x, y coordinate system
    m_x, m_y = bearing_to_cartesian(m_point[0], m_point[1])
    e_x, e_y = bearing_to_cartesian(e_point[0], e_point[1])

    # Finding new m point if e was located at the origin (0, 0)
    temp_m_x = m_x - e_x
    temp_m_y = m_y - e_y

    bearing, _ = cartesian_to_bearing(temp_m_x, temp_m_y)

    return bearing


def find_stm(r_point: tuple, m_point: tuple, e_point: tuple) -> float:
    """
    Calculates Speed of True Motion (STM).

    Args:
        r_point: First point on radar of the target ship (bearing, range, time_string)
        m_point: Second point on radar of the target ship (bearing, range, time_string)
        e_point: e point (bearing, range)

    Returns:
        Speed of True Motion in knots
    """
    # Converting points to x, y coordinate system
    m_x, m_y = bearing_to_cartesian(m_point[0], m_point[1])
    e_x, e_y = bearing_to_cartesian(e_point[0], e_point[1])

    # Finding the time difference between when the two points appeared on radar
    r_time = datetime.strptime(r_point[2], "%H:%M")
    m_time = datetime.strptime(m_point[2], "%H:%M")
    time_delta = (m_time - r_time).total_seconds() / 3600

    # Finding the distance between the two points to calculate the speed
    distance = np.sqrt(((m_x - e_x) ** 2) + ((m_y - e_y) ** 2))
    speed = distance / time_delta

    return speed


def find_nc(our_course: float, r_nc: tuple) -> float:
    """
    Calculate New Course (N/C).

    Args:
        our_course: Own ship's current course in degrees
        r_nc: Relative new course (bearing, range)

    Returns:
        New course in degrees (0-359)
    """
    # Adding our course to the relative new course
    temp_new_course = our_course + r_nc[0]

    # Making sure the new course is between 0 and 360 degrees
    if temp_new_course < 360:
        return temp_new_course

    return temp_new_course % 360


def find_ns(r_point: tuple, m_point: tuple, e_point: tuple, rs_point: tuple) -> float:
    """
    Calculate New Speed (N/S).

    Args:
        r_point: First point on radar of the target ship (bearing, range, time_string)
        m_point: Second point on radar of the target ship (bearing, range, time_string)
        e_point: e point (bearing, range)
        rs_point: RS point (bearing, range)

    Returns:
        New speed in knots
    """
    # Converting points to x, y coordinate system
    _, e_y = bearing_to_cartesian(e_point[0], e_point[1])
    _, rs_y = bearing_to_cartesian(rs_point[0], rs_point[1])

    # Finding the time difference between when the two points appeared on radar
    r_time = datetime.strptime(r_point[2], "%H:%M")
    m_time = datetime.strptime(m_point[2], "%H:%M")
    time_delta = (m_time - r_time).total_seconds() / 3600

    # Finding the distance between the two points to calculate the speed
    distance = rs_y - e_y
    speed = distance / time_delta

    return speed
