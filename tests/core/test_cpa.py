"""
Tests the CPA calculations.

Author: Omar Younis
Date: 04/11/2025    [dd/mm/yyyy]
"""


from datetime import datetime

import pytest
import numpy as np

from radar_plotter.core.cpa import find_cpa_point, find_time_to_cpa
from radar_plotter.core.coordinates import bearing_to_cartesian, cartesian_to_bearing
from radar_plotter.core.relative_motion import find_line_equation



def test_find_cpa_point_basic():
    """Test CPA calculation with known scenario."""
    r_point = (45.0, 11.5, "14:00")
    m_point = (43.0, 9.0, "14:06")

    cpa_bearing, cpa_range = find_cpa_point(r_point, m_point)

    # CPA should be valid
    assert 0 <= cpa_bearing < 360
    assert cpa_range >= 0
    assert isinstance(cpa_bearing, float)
    assert isinstance(cpa_range, float)

    # Verify CPA calculation correctness using geometry
    # CPA is the point on the Relative Motion Line closest to origin
    # Convert to Cartesian
    r_x, r_y = bearing_to_cartesian(r_point[0], r_point[1])
    m_x, m_y = bearing_to_cartesian(m_point[0], m_point[1])

    # Get RML equation
    rml_slope, rml_intercept = find_line_equation((r_x, r_y), (m_x, m_y), cartesian=True)

    # CPA is perpendicular from origin to RML
    # Perpendicular slope = -1/rml_slope
    perp_slope = -1 / rml_slope

    # CPA point is intersection of RML and perpendicular line through origin
    # RML: y = m*x + c, Perpendicular: y = perp_slope * x
    # Solving: m*x + c = perp_slope * x
    expected_cpa_x = rml_intercept / (perp_slope - rml_slope)
    expected_cpa_y = rml_slope * expected_cpa_x + rml_intercept

    # Convert expected CPA to polar
    expected_cpa_bearing, expected_cpa_range = cartesian_to_bearing(expected_cpa_x, expected_cpa_y)

    # Verify calculated CPA matches expected
    assert np.isclose(cpa_bearing, expected_cpa_bearing, atol=2.0), \
        f"CPA bearing should be {expected_cpa_bearing:.2f}°, got {cpa_bearing:.2f}°"
    assert np.isclose(cpa_range, expected_cpa_range, atol=0.2), \
        f"CPA range should be {expected_cpa_range:.2f} NM, got {cpa_range:.2f} NM"


def test_find_cpa_point_should_be_closer():
    """Test that CPA is closer than initial points."""
    r_point = (45.0, 11.5, "14:00")
    m_point = (43.0, 9.0, "14:06")

    _, cpa_range = find_cpa_point(r_point, m_point)

    # CPA range should be less than or equal to both R and M ranges
    assert cpa_range <= r_point[1]
    assert cpa_range <= m_point[1]


def test_find_time_to_cpa_basic():
    """Test time to CPA calculation."""
    r_point = (45.0, 11.5, "14:00")
    cpa_point = (40.0, 1.5)
    srm = 25.0  # knots

    cpa_time = find_time_to_cpa(r_point, cpa_point, srm)

    # Should return a datetime object
    assert isinstance(cpa_time, datetime)

    # CPA time should be after r_point time
    r_time = datetime.strptime(r_point[2], "%H:%M")
    assert cpa_time >= r_time

    # Verify time calculation correctness
    # Time = Distance / Speed
    # Convert to Cartesian
    r_x, r_y = bearing_to_cartesian(r_point[0], r_point[1])
    cpa_x, cpa_y = bearing_to_cartesian(cpa_point[0], cpa_point[1])

    # Distance from R to CPA
    distance_to_cpa = np.sqrt((cpa_x - r_x)**2 + (cpa_y - r_y)**2)

    # Expected time: distance / speed (in hours)
    expected_time_hours = distance_to_cpa / srm
    expected_time_minutes = expected_time_hours * 60

    # Calculate actual time difference
    actual_time_diff_minutes = (cpa_time - r_time).total_seconds() / 60

    # Verify calculated time matches expected (within 1 minute tolerance)
    assert np.isclose(actual_time_diff_minutes, expected_time_minutes, atol=1.0), \
        f"Time to CPA should be {expected_time_minutes:.1f} minutes (distance \
            {distance_to_cpa:.2f} NM ÷ {srm} kts), got {actual_time_diff_minutes:.1f} minutes"


def test_find_time_to_cpa_realistic():
    """Test time to CPA with realistic scenario and verify calculation."""
    r_point = (45.0, 11.5, "14:00")
    cpa_point = (40.0, 1.5)
    srm = 25.0  # knots

    cpa_time = find_time_to_cpa(r_point, cpa_point, srm)
    r_time = datetime.strptime(r_point[2], "%H:%M")

    # Calculate expected time using physics: time = distance / speed
    r_x, r_y = bearing_to_cartesian(r_point[0], r_point[1])
    cpa_x, cpa_y = bearing_to_cartesian(cpa_point[0], cpa_point[1])
    distance = np.sqrt((cpa_x - r_x)**2 + (cpa_y - r_y)**2)

    # Expected time in minutes
    expected_time_minutes = (distance / srm) * 60

    # Actual time difference
    time_diff_minutes = (cpa_time - r_time).total_seconds() / 60

    # Time should match expected calculation (within 1 minute)
    assert np.isclose(time_diff_minutes, expected_time_minutes, atol=1.0), \
        f"Expected {expected_time_minutes:.1f} min, got {time_diff_minutes:.1f} min"

    # Sanity check: with SRM of 25 kts and distance ~10 NM,
    # time should be roughly 24 minutes (10/25 * 60)
    assert 20 <= time_diff_minutes <= 30, \
        f"For ~10 NM distance at 25 kts, time should be ~24 minutes, got \
            {time_diff_minutes:.1f} minutes"

    # Verify CPA time format is correct (HH:MM)
    assert cpa_time.hour >= 0 and cpa_time.hour < 24, "Hour should be 0-23"
    assert cpa_time.minute >= 0 and cpa_time.minute < 60, "Minute should be 0-59"
