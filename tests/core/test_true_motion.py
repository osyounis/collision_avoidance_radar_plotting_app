"""
Tests for true motion calculations.


Author: Omar Younis
Date: 04/11/2025    [dd/mm/yyyy]
"""


from datetime import datetime

import numpy as np

from radar_plotter.core.coordinates import bearing_to_cartesian, cartesian_to_bearing
from radar_plotter.core.true_motion import find_dtm, find_stm, find_nc, find_ns


def test_find_dtm_basic():
    """Test DTM calculation."""
    m_point = (43.0, 9.0, "14:06")
    e_point = (45.0, 8.0)

    dtm = find_dtm(m_point, e_point)

    # Should return valid bearing
    assert 0 <= dtm < 360
    assert isinstance(dtm, float)

    # DTM is the bearing from E to M (target's true motion direction)
    # Calculate expected DTM manually
    m_x, m_y = bearing_to_cartesian(m_point[0], m_point[1])
    e_x, e_y = bearing_to_cartesian(e_point[0], e_point[1])

    # Vector from E to M
    rel_x = m_x - e_x
    rel_y = m_y - e_y

    # Calculate expected bearing
    expected_dtm, _ = cartesian_to_bearing(rel_x, rel_y)

    # Verify calculated DTM matches expected
    assert np.isclose(dtm, expected_dtm, atol=2.0), \
        f"DTM should be {expected_dtm:.2f}°, got {dtm:.2f}°"


def test_find_stm_basic():
    """Test STM calculation."""
    r_point = (45.0, 11.5, "14:00")
    m_point = (43.0, 9.0, "14:06")
    e_point = (45.0, 8.0)

    stm = find_stm(r_point, m_point, e_point)

    # Should return positive speed
    assert stm > 0
    assert isinstance(stm, float)

    # STM is target's true speed: distance from E to M divided by time
    # Calculate distance from E to M
    m_x, m_y = bearing_to_cartesian(m_point[0], m_point[1])
    e_x, e_y = bearing_to_cartesian(e_point[0], e_point[1])
    distance = np.sqrt((m_x - e_x)**2 + (m_y - e_y)**2)

    # Calculate time interval
    r_time = datetime.strptime(r_point[2], "%H:%M")
    m_time = datetime.strptime(m_point[2], "%H:%M")
    time_delta_hours = (m_time - r_time).total_seconds() / 3600

    # Expected STM
    expected_stm = distance / time_delta_hours

    # Verify calculated STM matches expected
    assert np.isclose(stm, expected_stm, atol=0.5), \
        f"STM should be {expected_stm:.2f} kts (distance {distance:.2f} NM ÷ \
            {time_delta_hours:.3f} hr), got {stm:.2f} kts"


def test_find_nc_basic():
    """Test new course calculation."""
    our_course = 0.0
    r_nc = (50.0, 3.0)

    nc = find_nc(our_course, r_nc)

    # Should return valid course
    assert 0 <= nc < 360
    assert isinstance(nc, float)

    # New course = our_course + r_nc bearing (relative to absolute conversion)
    expected_nc = our_course + r_nc[0]
    if expected_nc >= 360:
        expected_nc = expected_nc % 360

    # Verify calculation
    assert np.isclose(nc, expected_nc, atol=0.5), \
        f"New course should be {expected_nc:.2f}° (our course {our_course}° + \
            relative {r_nc[0]}°), got {nc:.2f}°"

    # For this specific test: 0° + 50° = 50°
    assert np.isclose(nc, 50.0, atol=0.5), \
        f"With our course 0° and relative 50°, new course should be 50°, got {nc:.2f}°"


def test_find_nc_wraparound():
    """Test new course with wraparound past 360."""
    our_course = 350.0
    r_nc = (20.0, 3.0)

    nc = find_nc(our_course, r_nc)

    # Should wrap around to valid course
    assert 0 <= nc < 360


def test_find_ns_basic():
    """Test new speed calculation."""
    r_point = (45.0, 11.5, "14:00")
    m_point = (43.0, 9.0, "14:06")
    e_point = (45.0, 8.0)
    rs_point = (45.0, 12.0)  # RS further out than E for positive speed

    ns = find_ns(r_point, m_point, e_point, rs_point)

    # Should return positive speed
    assert ns > 0
    assert isinstance(ns, float)

    # New speed is calculated from y-coordinate difference between RS and E
    # divided by time interval: NS = (RS_y - E_y) / time
    # Get y-coordinates
    _, e_y = bearing_to_cartesian(e_point[0], e_point[1])
    _, rs_y = bearing_to_cartesian(rs_point[0], rs_point[1])

    # Calculate time interval
    r_time = datetime.strptime(r_point[2], "%H:%M")
    m_time = datetime.strptime(m_point[2], "%H:%M")
    time_delta_hours = (m_time - r_time).total_seconds() / 3600

    # Expected new speed
    y_distance = rs_y - e_y
    expected_ns = y_distance / time_delta_hours

    # Verify calculation
    assert np.isclose(ns, expected_ns, atol=0.5), \
        f"New speed should be {expected_ns:.2f} kts (y-distance \
            {y_distance:.2f} NM ÷ {time_delta_hours:.3f} hr), got {ns:.2f} kts"

    # New speed should be reasonable (not negative, not extremely high)
    assert 0 < ns < 50, \
        f"New speed should be reasonable (0-50 kts), got {ns:.2f} kts"
