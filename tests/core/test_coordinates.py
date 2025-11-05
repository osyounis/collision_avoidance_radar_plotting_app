"""
Tests the coordinate conversion functions.

Author: Omar Younis
Date: 30/10/2025    [dd/mm/yyyy]
"""

import pytest
import numpy as np

from radar_plotter.core.coordinates import bearing_to_cartesian, cartesian_to_bearing


def test_bearing_to_cartesian_north():
    """Tests conversion for due North."""
    x, y = bearing_to_cartesian(0.0, 10.0)
    assert np.isclose(x, 0.0, atol=1e-10)
    assert np.isclose(y, 10.0)

def test_bearing_to_cartesian_east():
    """Tests conversion for due East."""
    x, y = bearing_to_cartesian(90.0, 10.0)
    assert np.isclose(x, 10.0)
    assert np.isclose(y, 0.0, atol=1e-10)

def test_bearing_to_cartesian_south():
    """Tests conversion for due South."""
    x, y = bearing_to_cartesian(180.0, 10.0)
    assert np.isclose(x, 0.0, atol=1e-10)
    assert np.isclose(y, -10.0)

def test_bearing_to_cartesian_west():
    """Tests conversion for due West."""
    x, y = bearing_to_cartesian(270.0, 10.0)
    assert np.isclose(x, -10.0)
    assert np.isclose(y, 0.0, atol=1e-10)

def test_cartesian_to_bearing_north():
    """Tests conversion back to bearing for North."""
    bearing, range_val = cartesian_to_bearing(0.0, 10.0)
    assert np.isclose(bearing, 0.0)
    assert np.isclose(range_val, 10.0)

def test_cartesian_to_bearing_east():
    """Test conversion back to bearing to East."""
    bearing, range_val = cartesian_to_bearing(10.0, 0.0)
    assert np.isclose(bearing, 90.0)
    assert np.isclose(range_val, 10.0)

def test_round_trip_conversion():
    """Test that converting back and forth preserves original values."""
    original_bearing = 45.0
    original_range = 5.5

    x, y = bearing_to_cartesian(original_bearing, original_range)
    bearing, range_val = cartesian_to_bearing(x, y)

    # Checking values
    assert np.isclose(bearing, original_bearing)
    assert np.isclose(range_val, original_range)

def test_multiple_round_trips():
    """Test multiple round trip conversions."""
    test_cases = [
        (0.0, 10.0),
        (45.0, 5.5),
        (90.0, 8.2),
        (135.0, 12.0),
        (180.0, 6.7),
        (225.0, 9.1),
        (270.0, 4.3),
        (315.0, 11.5)
    ]

    for bearing_in, range_in in test_cases:
        x, y = bearing_to_cartesian(bearing_in, range_in)
        bearing_out, range_out = cartesian_to_bearing(x, y)

        assert np.isclose(bearing_out, bearing_in, atol=1e-6)
        assert np.isclose(range_out, range_in, atol=1e-6)
