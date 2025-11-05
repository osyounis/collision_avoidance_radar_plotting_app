"""
Closest Point of Approach (CPA) calculations.

This module calculates:
    - CPA point (bearing and range)
    - Time to CPA

Author: Omar Younis
Date: 28/10/2025    [dd/mm/yyyy]
"""

from datetime import datetime, timedelta
import numpy as np

from .coordinates import bearing_to_cartesian, cartesian_to_bearing
from .relative_motion import find_line_equation


def find_cpa_point(r_point: tuple, m_point: tuple) -> tuple[float, float]:
    """
    Find the Closest Point of Approach (CPA).

    Args:
        r_point: First point on radar of the target ship (bearing, range, time_string)
        m_point: Second point on radar of the target ship (bearing, range, time_string)

    Returns:
        Tuple of (bearing, range) for the CPA point
    """
    # Convert points tp x, y coordinate system
    r_x, r_y = bearing_to_cartesian(r_point[0], r_point[1])
    m_x, m_y = bearing_to_cartesian(m_point[0], m_point[1])

    # Find the slope and intercept for the Relative Motion Line
    line_slope, line_intercept = find_line_equation((r_x, r_y), (m_x, m_y))

    # The CPA is the closest distance from the origin (your ship) to the
    # relative motion line, so the CPA line's intercept will be zero.
    perpendicular_line_slope = -1 / line_slope

    # Finding the actual point which is on the Relative Motion Line (RML) and
    # the closest to the origin. In other words where the perpendicular line and
    # the RML intersect.
    cpa_x = line_intercept / (perpendicular_line_slope - line_slope)
    cpa_y = (line_slope * cpa_x) + line_intercept

    # Converting from x, y point to a bearing and range
    cpa_bearing, cpa_range = cartesian_to_bearing(cpa_x, cpa_y)

    return cpa_bearing, cpa_range


def find_time_to_cpa(r_point: tuple, cpa_point: tuple, speed: float) -> datetime:
    """
    Calculate the time that CPA will occur.

    Args:
        r_point: First point on radar of the target ship (bearing, range, time_string)
        cpa_point: CPA point (bearing, range)
        speed: Speed of relative motion (SRM) in knots

    Returns:
        datetime object representing when the CPA will occur
    """
    # Converting points to x, y coordinate system
    r_x, r_y = bearing_to_cartesian(r_point[0], r_point[1])
    cpa_x, cpa_y = bearing_to_cartesian(cpa_point[0], cpa_point[1])

    # Getting the time of when point `r` occurred and converting it to a
    # datetime object.
    r_time = datetime.strptime(r_point[2], "%H:%M")

    # Calculating the distance and time needed to calculate when the CPA will
    # occur (e.g. 15:06)
    distance = np.sqrt(((cpa_x - r_x) ** 2) + ((cpa_y - r_y) ** 2))
    delta_time = (distance / speed) * 60
    time_of_cpa = r_time + timedelta(minutes=delta_time)

    return time_of_cpa
