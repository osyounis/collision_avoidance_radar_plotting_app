"""
Tests maneuver calculations.

Author: Omar Younis
Date: 04/11/2025    [dd/mm/yyyy]
"""


from datetime import datetime

import pytest
import numpy as np

from radar_plotter.core.coordinates import bearing_to_cartesian, cartesian_to_bearing
from radar_plotter.core.relative_motion import find_line_equation
from radar_plotter.core.maneuvers import (
    find_maneuver_point,
    find_nrml_equation,
    find_arml_equation,
    find_e_point,
    find_rs_point,
    find_r_nc,
    find_rc_point
)


def test_find_maneuver_point_basic():
    """Test maneuver point calculation."""
    r_point = (45.0, 11.5, "14:00")
    m_point = (43.0, 9.0, "14:06")
    maneuver_dist = 5.0

    bearing, range_val = find_maneuver_point(r_point, m_point, maneuver_dist)

    # Should return valid bearing and range
    assert 0 <= bearing < 360
    assert isinstance(bearing, float)
    assert isinstance(range_val, float)

    # Range should equal maneuver distance (critical requirement)
    assert np.isclose(range_val, maneuver_dist, atol=0.1), \
        f"Maneuver point should be at {maneuver_dist} NM, got {range_val:.2f} NM"

    # Verify maneuver point lies on the Relative Motion Line (RML)
    # Convert points to Cartesian
    r_x, r_y = bearing_to_cartesian(r_point[0], r_point[1])
    m_x, m_y = bearing_to_cartesian(m_point[0], m_point[1])
    maneuv_x, maneuv_y = bearing_to_cartesian(bearing, range_val)

    # Get RML equation
    rml_slope, rml_intercept = find_line_equation((r_x, r_y), (m_x, m_y), cartesian=True)

    # Check if maneuver point lies on RML: y = mx + c
    expected_y = rml_slope * maneuv_x + rml_intercept
    assert np.isclose(maneuv_y, expected_y, atol=0.2), \
        f"Maneuver point should lie on RML (expected y={expected_y:.2f}, got y={maneuv_y:.2f})"

    # Verify maneuver point is between origin and target (not beyond)
    assert bearing < r_point[0] + 30 and bearing > r_point[0] - 30, \
        f"Maneuver point bearing should be near RML (~{r_point[0]}°), got {bearing:.2f}°"


def test_find_e_point_basic():
    """Test e-point calculation."""
    r_point = (45.0, 11.5, "14:00")
    m_point = (43.0, 9.0, "14:06")
    our_speed = 10.0

    bearing, range_val = find_e_point(r_point, m_point, our_speed)

    # Should return valid bearing and range
    assert 0 <= bearing < 360
    assert range_val >= 0
    assert isinstance(bearing, float)
    assert isinstance(range_val, float)

    # E-point represents our velocity vector: distance traveled = speed × time
    r_time = datetime.strptime(r_point[2], "%H:%M")
    m_time = datetime.strptime(m_point[2], "%H:%M")
    time_delta_hours = (m_time - r_time).total_seconds() / 3600

    # Expected distance traveled
    expected_distance = our_speed * time_delta_hours  # 10 kts × 0.1 hr = 1.0 NM

    # E-point range should match our distance traveled
    assert np.isclose(range_val, expected_distance, atol=0.5), \
        f"E-point should be at {expected_distance:.2f} NM (speed {our_speed} kts \
            x {time_delta_hours:.3f} hr), got {range_val:.2f} NM"

    # E-point bearing should match R-point bearing (same x-coordinate in traditional plotting)
    # E-point is directly below R on the plotting sheet
    assert np.isclose(bearing, r_point[0], atol=5.0), \
        f"E-point bearing should be close to R-point bearing ({r_point[0]}°), got {bearing:.2f}°"


def test_find_nrml_equation_basic():
    """Test NRML equation calculation."""
    r_point = (45.0, 11.5, "14:00")
    m_point = (43.0, 9.0, "14:06")
    maneuver_point = (44.0, 5.0)
    new_cpa_dist = 2.5

    slope, intercept = find_nrml_equation(r_point, m_point, maneuver_point, new_cpa_dist)

    # Should return valid slope and intercept
    assert isinstance(slope, float)
    assert isinstance(intercept, float)

    # NRML must pass through the maneuver point
    maneuv_x, maneuv_y = bearing_to_cartesian(maneuver_point[0], maneuver_point[1])

    # Check: y = mx + c passes through maneuver point
    expected_y = slope * maneuv_x + intercept
    assert np.isclose(maneuv_y, expected_y, atol=0.2), \
        f"NRML should pass through maneuver point (expected y={expected_y:.2f}, \
            got y={maneuv_y:.2f})"

    # NRML must be tangent to circle of radius new_cpa_dist centered at origin
    # Distance from origin to line y = mx + c is: |c| / sqrt(1 + m²)
    distance_to_origin = abs(intercept) / np.sqrt(1 + slope**2)
    assert np.isclose(distance_to_origin, new_cpa_dist, atol=0.3), \
        f"NRML should be tangent to circle of radius {new_cpa_dist} NM \
            (distance = {distance_to_origin:.2f} NM)"

    # NRML slope should be similar to RML slope (within ~45° typically)
    r_x, r_y = bearing_to_cartesian(r_point[0], r_point[1])
    m_x, m_y = bearing_to_cartesian(m_point[0], m_point[1])
    rml_slope, _ = find_line_equation((r_x, r_y), (m_x, m_y), cartesian=True)

    # Slopes should be in same general direction (not wildly different)
    slope_angle_diff = abs(np.degrees(np.arctan(slope)) - np.degrees(np.arctan(rml_slope)))
    assert slope_angle_diff < 60, \
        f"NRML slope should be reasonably close to RML slope (angle diff = {slope_angle_diff:.1f}°)"


def test_find_arml_equation_basic():
    """Test ARML equation calculation."""
    m_point = (43.0, 9.0, "14:06")
    nrml_equation = (-1.5, 2.0)

    slope, intercept = find_arml_equation(m_point, nrml_equation)

    # Slope should match NRML slope (ARML is parallel to NRML)
    assert np.isclose(slope, nrml_equation[0]), \
        f"ARML slope should match NRML slope ({nrml_equation[0]:.2f}), got {slope:.2f}"
    assert isinstance(intercept, float)

    # ARML must pass through M point
    m_x, m_y = bearing_to_cartesian(m_point[0], m_point[1])

    # Check: y = mx + c passes through M point
    expected_y = slope * m_x + intercept
    assert np.isclose(m_y, expected_y, atol=0.2), \
        f"ARML should pass through M point (expected y={expected_y:.2f}, got y={m_y:.2f})"

    # Verify intercept is different from NRML (parallel lines, different intercepts)
    assert not np.isclose(intercept, nrml_equation[1], atol=0.1), \
        "ARML should be parallel to NRML but not the same line (different intercepts)"


def test_find_rs_point_basic():
    """Test RS point calculation."""
    e_point = (45.0, 8.0)
    arml_equation = (-1.5, 2.0)

    bearing, range_val = find_rs_point(e_point, arml_equation)

    # Should return valid bearing and range
    assert 0 <= bearing < 360
    assert range_val >= 0

    # RS point is the intersection of vertical line through E-point and ARML
    # Meaning: RS has same x-coordinate as E, but y is on ARML
    e_x, e_y = bearing_to_cartesian(e_point[0], e_point[1])
    rs_x, rs_y = bearing_to_cartesian(bearing, range_val)

    # RS should have same x-coordinate as E (vertical line through E)
    assert np.isclose(rs_x, e_x, atol=0.2), \
        f"RS point should have same x-coordinate as E point (expected \
            x={e_x:.2f}, got x={rs_x:.2f})"

    # RS should lie on ARML: y = mx + c
    arml_slope, arml_intercept = arml_equation
    expected_rs_y = arml_slope * rs_x + arml_intercept
    assert np.isclose(rs_y, expected_rs_y, atol=0.2), \
        f"RS point should lie on ARML (expected y={expected_rs_y:.2f}, got y={rs_y:.2f})"

    # RS represents required speed change - should be reasonable
    assert range_val < 20.0, \
        f"RS point range should be reasonable (< 20 NM), got {range_val:.2f} NM"


def test_find_r_nc_basic():
    """Test r_nc (relative new course) vector calculation."""
    r_point = (45.0, 11.5, "14:00")
    m_point = (43.0, 9.0, "14:06")
    e_point = (45.0, 8.0)
    rs_point = (45.0, 5.0)
    our_speed = 10.0
    arml_equation = (-1.5, 2.0)

    bearing, range_val = find_r_nc(r_point, m_point, e_point, rs_point, our_speed, arml_equation)

    # Should return valid bearing and range
    assert 0 <= bearing < 360
    assert range_val > 0
    assert isinstance(bearing, float)
    assert isinstance(range_val, float)

    # r_nc vector represents where circle (radius = speed × time) intersects ARML
    # Calculate expected radius of circle
    r_time = datetime.strptime(r_point[2], "%H:%M")
    m_time = datetime.strptime(m_point[2], "%H:%M")
    time_delta_hours = (m_time - r_time).total_seconds() / 3600
    expected_radius = our_speed * time_delta_hours

    # r_nc range should equal expected radius (it's on the circle)
    assert np.isclose(range_val, expected_radius, atol=0.5), \
        f"r_nc range should be {expected_radius:.2f} NM (speed {our_speed} kts \
            x {time_delta_hours:.3f} hr), got {range_val:.2f} NM"

    # r_nc should be in same hemisphere as r_point (bearing selection logic)
    r_hemisphere = "lower" if r_point[0] <= 180 else "upper"
    r_nc_hemisphere = "lower" if bearing <= 180 else "upper"
    assert r_hemisphere == r_nc_hemisphere, \
        f"r_nc bearing ({bearing:.2f}°) should be in same hemisphere as r_point ({r_point[0]}°)"

    # Verify r_nc is geometrically valid relative to E point
    e_x, e_y = bearing_to_cartesian(e_point[0], e_point[1])
    rs_x, rs_y = bearing_to_cartesian(rs_point[0], rs_point[1])
    r_nc_x, r_nc_y = bearing_to_cartesian(bearing, range_val)

    # When centered at E, r_nc should lie on translated ARML
    # Convert to E-centered coordinates (as done in find_r_nc function)
    r_nc_rel_x = r_nc_x - e_x
    r_nc_rel_y = r_nc_y - e_y
    rs_rel_x = rs_x - e_x
    rs_rel_y = rs_y - e_y

    # Calculate translated ARML intercept (as done in find_r_nc)
    temp_arml_intercept = rs_rel_y - (arml_equation[0] * rs_rel_x)

    # Check if r_nc lies on translated ARML: y = mx + c
    expected_y = arml_equation[0] * r_nc_rel_x + temp_arml_intercept
    assert np.isclose(r_nc_rel_y, expected_y, atol=0.3), \
        f"r_nc should lie on translated ARML (expected y={expected_y:.2f}, got y={r_nc_rel_y:.2f})"

    # Verify r_nc distance from E equals expected radius (it's on the circle centered at E)
    distance_from_e = np.sqrt(r_nc_rel_x**2 + r_nc_rel_y**2)
    assert np.isclose(distance_from_e, expected_radius, atol=0.5), \
        f"r_nc distance from E should be {expected_radius:.2f} NM, got {distance_from_e:.2f} NM"


def test_find_rc_point_basic():
    """Test RC point calculation."""
    e_point = (45.0, 8.0)
    r_nc_vector = (50.0, 3.0)

    bearing, range_val = find_rc_point(e_point, r_nc_vector)

    # Should return valid bearing and range
    assert 0 <= bearing < 360
    assert range_val >= 0

    # RC point is vector sum of E point and r_nc vector
    # RC = E + r_nc (in Cartesian coordinates)
    e_x, e_y = bearing_to_cartesian(e_point[0], e_point[1])
    r_nc_x, r_nc_y = bearing_to_cartesian(r_nc_vector[0], r_nc_vector[1])

    # Expected RC point (vector addition)
    expected_rc_x = e_x + r_nc_x
    expected_rc_y = e_y + r_nc_y

    # Convert expected to polar
    expected_bearing, expected_range = cartesian_to_bearing(expected_rc_x, expected_rc_y)

    # Convert actual RC to Cartesian for comparison
    rc_x, rc_y = bearing_to_cartesian(bearing, range_val)

    # Verify RC = E + r_nc
    assert np.isclose(rc_x, expected_rc_x, atol=0.2), \
        f"RC x-coordinate should be E_x + r_nc_x (expected {expected_rc_x:.2f}, got {rc_x:.2f})"
    assert np.isclose(rc_y, expected_rc_y, atol=0.2), \
        f"RC y-coordinate should be E_y + r_nc_y (expected {expected_rc_y:.2f}, got {rc_y:.2f})"

    # Verify in polar coordinates
    assert np.isclose(bearing, expected_bearing, atol=2.0), \
        f"RC bearing should be {expected_bearing:.2f}°, got {bearing:.2f}°"
    assert np.isclose(range_val, expected_range, atol=0.2), \
        f"RC range should be {expected_range:.2f} NM, got {range_val:.2f} NM"
