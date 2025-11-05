"""
Tests the relative motion calculations.

Author: Omar Younis
Date: 04/11/2025    [dd/mm/yyyy]
"""


import pytest
import numpy as np

from radar_plotter.core.relative_motion import find_srm, find_drm, find_line_equation
from radar_plotter.core.coordinates import bearing_to_cartesian


def test_find_srm_basic():
    """Tests SRM calculation with known scenario."""
    r_point = (45.0, 11.5, "14:00")
    m_point = (43.0, 9.0, "14:06")

    srm = find_srm(r_point, m_point)

    # Should return a positive speed
    assert srm > 0
    assert isinstance(srm, float)

    # Calculate expected SRM manually to verify calculation
    # Convert to Cartesian
    r_x, r_y = bearing_to_cartesian(45.0, 11.5)
    m_x, m_y = bearing_to_cartesian(43.0, 9.0)

    # Distance traveled
    distance = np.sqrt(((m_x - r_x) ** 2) + ((m_y - r_y) ** 2))

    # Time interval: 14:00 to 14:06 = 6 minutes = 0.1 hours
    time_hours = 6.0 / 60.0

    # Expected SRM
    expected_srm = distance / time_hours

    # Verify calculated SRM matches expected (within 0.5 kts tolerance)
    assert np.isclose(srm, expected_srm, atol=0.5), \
        f"SRM should be {expected_srm:.2f} kts (distance {distance:.2f} NM / \
            {time_hours:.3f} hr), got {srm:.2f} kts"


def test_find_drm_basic():
    """Test DRM calculation with known scenario."""
    r_point = (45.0, 11.5, "14:00")
    m_point = (43.0, 9.0, "14:06")

    drm = find_drm(r_point, m_point)

    # DRM should be between 0 and 360
    assert 0 <= drm < 360
    assert isinstance(drm, float)

    # Calculate expected DRM manually to verify calculation
    # Convert to Cartesian
    r_x, r_y = bearing_to_cartesian(45.0, 11.5)
    m_x, m_y = bearing_to_cartesian(43.0, 9.0)

    # Vector from R to M (relative motion direction)
    rel_x = m_x - r_x
    rel_y = m_y - r_y

    # Calculate expected bearing
    expected_drm = np.degrees(np.arctan2(rel_x, rel_y))
    if expected_drm < 0:
        expected_drm += 360

    # Verify calculated DRM matches expected (within 2° tolerance)
    assert np.isclose(drm, expected_drm, atol=2.0), \
        f"DRM should be {expected_drm:.2f}°, got {drm:.2f}°"

    # For this scenario: bearing 45° to 43°, range 11.5 to 9.0 NM
    # Target is closing and slightly left drift - DRM should be roughly SW (around 220-240°)
    assert 210 <= drm <= 250, \
        f"For target closing from bearing 45° at decreasing range, DRM should \
            be SW quadrant, got {drm:.2f}°"


def test_find_drm_directions():
    """Test DRM returns correct direction for different scenarios."""
    # Target moving south (decreasing range, same bearing)
    r_point = (0.0, 10.0, "14:00")
    m_point = (0.0, 8.0, "14:06")

    drm = find_drm(r_point, m_point)

    # Should be approximately 180° (south)
    assert 170 < drm < 190


def test_find_line_equation_cartesian():
    """Test line equation calculation with Cartesian points."""
    point_1 = (0.0, 0.0)
    point_2 = (1.0, 1.0)

    slope, intercept = find_line_equation(point_1, point_2, cartesian=True)

    # y = x line should have slope 1, intercept 0
    assert np.isclose(slope, 1.0)
    assert np.isclose(intercept, 0.0)


def test_find_line_equation_polar():
    """Test line equation calculation with polar points."""
    r_point = (0.0, 10.0, "14:00")  # North, 10 NM
    m_point = (90.0, 10.0, "14:06")  # East, 10 NM

    slope, intercept = find_line_equation(r_point, m_point, cartesian=False)

    # Should return valid slope and intercept
    assert isinstance(slope, float)
    assert isinstance(intercept, float)

    # Calculate expected values manually
    # R point: bearing 0° (North), range 10 NM → Cartesian (0, 10)
    # M point: bearing 90° (East), range 10 NM → Cartesian (10, 0)
    r_x, r_y = bearing_to_cartesian(0.0, 10.0)
    m_x, m_y = bearing_to_cartesian(90.0, 10.0)

    # Line from (0, 10) to (10, 0)
    # Slope = (y2 - y1) / (x2 - x1) = (0 - 10) / (10 - 0) = -10 / 10 = -1
    expected_slope = (m_y - r_y) / (m_x - r_x)

    # Intercept: y = mx + b → b = y - mx
    # Using point (0, 10): b = 10 - (-1)(0) = 10
    expected_intercept = r_y - (expected_slope * r_x)

    # Verify calculated values match expected
    assert np.isclose(slope, expected_slope, atol=0.1), \
        f"Slope should be {expected_slope:.2f}, got {slope:.2f}"
    assert np.isclose(intercept, expected_intercept, atol=0.1), \
        f"Intercept should be {expected_intercept:.2f}, got {intercept:.2f}"

    # For this specific case: line from North to East should have slope -1, intercept 10
    assert np.isclose(slope, -1.0, atol=0.1), \
        "Line from North (0,10) to East (10,0) should have slope -1"
    assert np.isclose(intercept, 10.0, atol=0.1), \
        "Line from North (0,10) to East (10,0) should have y-intercept 10"
