"""
Coordinates conversion utilities for radar plotting.

This modules has functions used to convert between bearing/range (polar)
coordinates and Cartesian (x, y) coordinates.

Author: Omar Younis
Date: 27/10/2025    [dd/mm/yyyy]
"""

import numpy as np


def bearing_to_cartesian(bearing: float, target_range: float) -> tuple:
    """
    Converts bearing and range to Cartesian coordinates (polar to cartesian).

    Args:
        bearing: Bearing in degrees (0° - 359°, where  0° is North)
        target_range: Range/distance in nautical miles

    Returns:
        Tuple: (x, y) coordinates where:
            - x: East-West component(positive = East)
            - y: North-South component (positive = North)
    """
    # Convert radians to degrees using numpy's built-in function
    angle = np.deg2rad(bearing)

    x_coordinate = target_range * np.sin(angle)
    y_coordinate = target_range * np.cos(angle)

    return x_coordinate, y_coordinate


def cartesian_to_bearing(x_coord: float, y_coord: float) -> tuple:
    """
    Converts Cartesian coordinates to bearing and range (polar coordinates).

    Args:
        x_coord: East-West component (positive = East)
        y_coord: North-South component (positive = North)

    Returns:
        Tuple: (bearing, range) where:
            - bearing: Bearing in degrees (0° - 359°)
            - range: Distance in nautical miles
    """
    # Getting range and radian bearing from x, y coordinates
    target_range = np.sqrt((x_coord**2) + (y_coord**2))
    rad_bearing = np.arctan2(x_coord, y_coord)

    # Convert bearing from rad to degrees and check to make sure the value
    # is within 0° to 360°.
    bearing = np.rad2deg(rad_bearing)
    if bearing < 0:
        bearing = 360 + bearing

    return bearing, target_range
