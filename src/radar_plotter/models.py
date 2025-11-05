"""
Data models for radar plotting.

This module defines structured data classes for inputs and outputs.

Author: Omar Younis
Date: 28/10/2025    [dd/mm/yyyy]
"""

from dataclasses import dataclass
from datetime import datetime


@dataclass
class RadarPoint:
    """Represents a single radar plot point."""

    bearing: float  # degrees (0-359)
    range: float  # nautical miles
    time: str  # format: "HH:MM" [24hr]

    def to_tuple(self) -> tuple[float, float, str]:
        """Convert to tuple format for legacy functions."""
        return (self.bearing, self.range, self.time)


@dataclass
class RadarProblem:
    """Represents a complete radar plotting problem."""

    our_course: float  # degrees
    our_speed: float  # knots
    maneuver_dist: float  # nautical miles
    new_cpa_dist: float  # nautical miles (Keep Out Distance)
    r_point: RadarPoint  # Target's 1st appearance on radar
    m_point: RadarPoint  # Target's 2nd appearance on radar


@dataclass
class RadarSolution:
    """Represents the solution to a radar plotting problem."""

    # CPA information
    cpa_bearing: float
    cpa_range: float
    cpa_time: datetime

    # Relative Motion
    srm: float  # Speed of Relative Motion (knots)
    drm: float  # Direction of Relative Motion (degrees)

    # True Motion
    stm: float  # Speed of True Motion (knots)
    dtm: float  # Direction of True Motion (degrees)

    # Maneuver solution
    new_course: float  # N/C (degrees)
    new_speed: float  # N/S (knots)

    # Intermediate points (used for plotting)
    maneuver_point: tuple[float, float]
    e_point: tuple[float, float]
    rs_point: tuple[float, float]
    rc_point: tuple[float, float]
