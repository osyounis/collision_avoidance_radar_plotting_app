"""
Maneuver calculations for collision avoidance.

This module calculates maneuver points and course/speed changes needed to
achieve a desired CPA. This is otherwise known as the standing orders. Usually
the captain specifies that other ships should remain at a certain distance away
from the ship at all times. This is what "achieving a desired CPA" refers to.

Author: Omar Younis
Date: 28/10/2025    [dd/mm/yyyy]
"""

from datetime import datetime
import numpy as np

from .coordinates import bearing_to_cartesian, cartesian_to_bearing
from .relative_motion import find_line_equation


def find_maneuver_point(r_point: tuple, m_point: tuple, maneuver_dist: float) -> tuple:
    """
    Find the maneuver point on the relative motion line.

    Args:
        r_point: First point on radar of the target ship (bearing, range, time_string)
        m_point: Second point on radar of the target ship (bearing, range, time_string)
        maneuver_dist: Desired maneuver distance in nautical miles

    Returns:
        Tuple of (bearing, range) for the maneuver point
    """
    # Convert points to x, y coordinate system
    r_x, r_y = bearing_to_cartesian(r_point[0], r_point[1])
    m_x, m_y = bearing_to_cartesian(m_point[0], m_point[1])

    # Finding the equation of a line
    line_slope, line_intercept = find_line_equation((r_x, r_y), (m_x, m_y))

    # Calculating the a, b, c constants for a quadratic equation:
    # ax^2 + bx + c = 0. Center is at the origin and we substitute our line
    # equation (y = mx + c) into the circle equation. Note that we use the
    # circle equation (x - h)^2 + (y - k)^2 = r^2 and then we find x using:
    # x = -b +- sqrt(b^2 -4ac)/2a.
    a = 1 + (line_slope**2)
    b = 2 * line_slope * line_intercept
    c = (line_intercept**2) - (maneuver_dist**2)

    # Finding the two points on the line that intersect the manuever distance
    # circle (i.e the roots of the quadratic equation)
    x_1 = (-b + np.sqrt((b**2) - (4 * a * c))) / (2 * a)
    x_2 = (-b - np.sqrt((b**2) - (4 * a * c))) / (2 * a)

    # Finding the y values for each of the roots
    y_1 = (line_slope * x_1) + line_intercept
    y_2 = (line_slope * x_2) + line_intercept

    # Finding out which point is closest to the target ship
    distance_1 = np.sqrt(((m_x - x_1) ** 2) + ((m_y - y_1) ** 2))
    distance_2 = np.sqrt(((m_x - x_2) ** 2) + ((m_y - y_2) ** 2))

    if distance_1 < distance_2:
        return cartesian_to_bearing(x_1, y_1)

    return cartesian_to_bearing(x_2, y_2)


def find_nrml_equation(
    r_point: tuple, m_point: tuple, maneuver_point: tuple, new_cpa_dist: float
) -> tuple[float, float]:
    """
    Find the new Relative Motion Line (NRML) equation.

    Args:
        r_point: First point on radar of the target ship (bearing, range, time_string)
        m_point: Second point on radar of the target ship (bearing, range, time_string)
        maneuver_point: Maneuver point (bearing, range)
        new_cpa_dist: Desired new CPA distance in nautical miles.

    Returns:
        Tuple of (slope, intercept) for the NRML equation
    """
    # Converting points to x, y coordinates system
    r_x, r_y = bearing_to_cartesian(r_point[0], r_point[1])
    m_x, m_y = bearing_to_cartesian(m_point[0], m_point[1])
    maneuv_x, maneuv_y = bearing_to_cartesian(maneuver_point[0], maneuver_point[1])

    # Finding the slope of the RML line
    line_slope, _ = find_line_equation((r_x, r_y), (m_x, m_y))

    # Calculating the equation (mainly the slope is needed) of the line that
    # passes through the selected maneuver point to the tangent point that
    # touches the circle with the radius of the new CPA. This will give us two
    # points, so we use the point which creates a slope that is closest to the
    # current CPA line.
    t_a = (maneuv_y**2) + (maneuv_x**2)
    t_b = -(2 * (new_cpa_dist**2) * maneuv_x)
    t_c = (new_cpa_dist**4) - ((new_cpa_dist**2) * (maneuv_y**2))

    # Finding the x-coordinates of the two possible points for the two tangent
    # lines from the maneuver point to the new CPA circle
    tx_1 = (-t_b + np.sqrt((t_b**2) - (4 * t_a * t_c))) / (2 * t_a)
    tx_2 = (-t_b - np.sqrt((t_b**2) - (4 * t_a * t_c))) / (2 * t_a)

    # Finding the y-coordinate for each of the possible x-coordinates
    ty_1 = ((new_cpa_dist**2) - (maneuv_x * tx_1)) / maneuv_y
    ty_2 = ((new_cpa_dist**2) - (maneuv_x * tx_2)) / maneuv_y

    # Finding the slopes for the two possible points. We then check which slope
    # is closest to the RML's slope which is the slope we are interested in.
    slope_1, intercept_1 = find_line_equation((maneuv_x, maneuv_y), (tx_1, ty_1))
    slope_2, intercept_2 = find_line_equation((maneuv_x, maneuv_y), (tx_2, ty_2))
    slope_diff_1 = np.abs(line_slope - slope_1)
    slope_diff_2 = np.abs(line_slope - slope_2)

    if slope_diff_1 < slope_diff_2:
        return slope_1, intercept_1

    return slope_2, intercept_2


def find_arml_equation(m_point: tuple, nrml_equation: tuple) -> tuple[float, float]:
    """
    Find the Actual Relative Motion Line (ARML) equation.

    Args:
        m_point: Second point on radar of the target ship (bearing, range, time_string)
        nrml_equation: NRML equation (slope, intercept)

    Returns:
        Tuple of (slope, intercept) for the ARML equation
    """
    # Converting m point into x, y coordinate system
    m_x, m_y = bearing_to_cartesian(m_point[0], m_point[1])
    line_intercept = m_y - (nrml_equation[0] * m_x)

    return nrml_equation[0], line_intercept


def find_e_point(
    r_point: tuple, m_point: tuple, our_speed: float
) -> tuple[float, float]:
    """
    Find point e (Origin of the True Motion Vector).

    Args:
        r_point: First point on radar of the target ship (bearing, range, time_string)
        m_point: Second point on radar of the target ship (bearing, range, time_string)
        our_speed: Own ship's speed in knots

    Returns:
        Tuple of (bearing, range) for point e.
    """
    # Converting points to x, y coordinate system
    r_x, r_y = bearing_to_cartesian(r_point[0], r_point[1])

    # Finding time taken to travel between points r and m. This will then be
    # used to figure out the distance traveled in that time, which will give us
    # the vertical location of point e.
    r_time = datetime.strptime(r_point[2], "%H:%M")
    m_time = datetime.strptime(m_point[2], "%H:%M")
    time_delta = (m_time - r_time).total_seconds() / 3600

    # Point e's x, y coordinates
    e_x = r_x
    e_y = r_y - (our_speed * time_delta)

    return cartesian_to_bearing(e_x, e_y)


def find_rs_point(e_point: tuple, arml_equation: tuple) -> tuple[float, float]:
    """
    Find rs point (required speed change point).

    Args:
        e_point: e-point (bearing, range)
        arml_equation: ARML equation (slope, intercept)

    Returns:
        Tuple of (bearing, range) for the rs point
    """
    # Converting point to x, y coordinate system
    e_x, _ = bearing_to_cartesian(e_point[0], e_point[1])

    # Finding the x, y coordinates of rs (The vector which determines the speed
    # change needed to avoid a collision with the target)
    rs_x = e_x
    rs_y = (arml_equation[0] * rs_x) + arml_equation[1]

    return cartesian_to_bearing(rs_x, rs_y)


def find_r_nc(
    r_point: tuple,
    m_point: tuple,
    e_point: tuple,
    rs_point: tuple,
    our_speed: float,
    arml_equation: tuple,
) -> tuple[float, float]:
    """
    Find the relative new course vector (r_nc).

    Args:
        r_point: First point on radar of the target ship (bearing, range, time_string)
        m_point: Second point on radar of the target ship (bearing, range, time_string)
        e_point: e point (bearing, range)
        rs_point: rs point(bearing, range)
        our_speed: Own ship's speed in knots
        arml_equation: ARML equation (slope, intercept)

    Returns:
        Tuple of (bearing, range) for the relative new course vector
    """
    # Converting point to x, y coordinate system
    e_x, e_y = bearing_to_cartesian(e_point[0], e_point[1])
    rs_x, rs_y = bearing_to_cartesian(rs_point[0], rs_point[1])

    # Finding the amount of time passed between points r and m
    r_time = datetime.strptime(r_point[2], "%H:%M")
    m_time = datetime.strptime(m_point[2], "%H:%M")

    # Dividing by 3600 because we are converting time delta from seconds to hours
    time_delta = (m_time - r_time).total_seconds() / 3600

    # For the next parts of the calculation, we are going to imagine that point
    # e is actually the origin. Finding new rs point if e was located at (0, 0)
    rs_x_temp = rs_x - e_x
    rs_y_temp = rs_y - e_y

    # Finding new ARML intercept if e was located at the origin
    temp_line_intercept = rs_y_temp - (arml_equation[0] * rs_x_temp)

    # Calculating the a, b, c constants for a quadratic equation:
    # ax^2 + bx + c = 0. Center is at the origin and we substitute our line
    # equation (y = mx + c) into the circle equation. Note that we use the
    # circle equation (x - h)^2 + (y - k)^2 = r^2 and then we find x using:
    # x = -b +- sqrt(b^2 -4ac)/2a.
    a = 1 + (arml_equation[0] ** 2)
    b = 2 * arml_equation[0] * temp_line_intercept
    c = (temp_line_intercept**2) - ((our_speed * time_delta) ** 2)

    # Finding the two points on the line that intersect the circle (i.e the
    # roots of the quadratic equation)
    x_1 = (-b + np.sqrt((b**2) - (4 * a * c))) / (2 * a)
    x_2 = (-b - np.sqrt((b**2) - (4 * a * c))) / (2 * a)

    # Finding the y values for each of the roots
    y_1 = (arml_equation[0] * x_1) + temp_line_intercept
    y_2 = (arml_equation[0] * x_2) + temp_line_intercept

    # Converting points to polar coordinates. Since both points are centered
    # around the origin for the moment, this also gives us the 2 possible vectors.
    r_nc_1 = cartesian_to_bearing(x_1, y_1)
    r_nc_2 = cartesian_to_bearing(x_2, y_2)

    # We get two possible solutions for this problem. To pick the correct bearing
    # we check the r_point's bearing. The bearing which is in the same half
    # of the original bearing is the correct bearing (half is from 0-180 and the
    # the other is 181 - 359).

    # Check if both bearings are in the same hemisphere (0-180 vs 181-359)
    both_lower_half = r_point[0] <= 180 and r_nc_1[0] <= 180
    both_upper_half = r_point[0] > 180 and r_nc_1[0] > 180
    return r_nc_1 if (both_lower_half or both_upper_half) else r_nc_2


def find_rc_point(e_point: tuple, r_nc_vector: tuple) -> tuple[float, float]:
    """
    Find rc point (required course change point).

    Args:
        e_point: e point (bearing, range)
        r_nc_vector: Relative new course vector (bearing, range)

    Returns:
        Tuple of (bearing, range) for the RC point
    """
    # Converting points/vectors to x, y coordinate systems
    e_x, e_y = bearing_to_cartesian(e_point[0], e_point[1])
    r_nc_x, r_nc_y = bearing_to_cartesian(r_nc_vector[0], r_nc_vector[1])

    # Adding the relative x, y components of the vector to point e to get the
    # true coordinates of point rc
    rc_x = r_nc_x + e_x
    rc_y = r_nc_y + e_y

    return cartesian_to_bearing(rc_x, rc_y)
