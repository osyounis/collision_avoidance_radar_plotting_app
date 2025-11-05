"""
Testing the data models.

Author: Omar Younis
Date: 04/11/2025    [dd/mm/yyyy]
"""


from datetime import datetime

import pytest

from radar_plotter.models import RadarPoint, RadarProblem, RadarSolution


def test_radar_point_creation():
    """Test RadarPoint creation."""
    point = RadarPoint(45.0, 11.5, "14:00")

    assert point.bearing == 45.0
    assert point.range == 11.5
    assert point.time == "14:00"


def test_radar_point_to_tuple():
    """Test RadarPoint to_tuple method."""
    point = RadarPoint(45.0, 11.5, "14:00")

    tuple_form = point.to_tuple()

    assert tuple_form == (45.0, 11.5, "14:00")
    assert isinstance(tuple_form, tuple)


def test_radar_problem_creation():
    """Test RadarProblem creation."""
    problem = RadarProblem(
        our_course=0.0,
        our_speed=10.0,
        maneuver_dist=5.0,
        new_cpa_dist=2.5,
        r_point=RadarPoint(45.0, 11.5, "14:00"),
        m_point=RadarPoint(43.0, 9.0, "14:06")
    )

    assert problem.our_course == 0.0
    assert problem.our_speed == 10.0
    assert problem.maneuver_dist == 5.0
    assert problem.new_cpa_dist == 2.5
    assert isinstance(problem.r_point, RadarPoint)
    assert isinstance(problem.m_point, RadarPoint)

    # Verify RadarPoint values are accessible
    assert problem.r_point.bearing == 45.0
    assert problem.r_point.range == 11.5
    assert problem.r_point.time == "14:00"
    assert problem.m_point.bearing == 43.0
    assert problem.m_point.range == 9.0
    assert problem.m_point.time == "14:06"

    # Verify input validation (values should be reasonable)
    assert 0 <= problem.our_course < 360, "Course should be valid bearing"
    assert problem.our_speed > 0, "Speed should be positive"
    assert problem.maneuver_dist > 0, "Maneuver distance should be positive"
    assert problem.new_cpa_dist > 0, "Keep out distance should be positive"


def test_radar_solution_creation():
    """Test RadarSolution creation."""
    solution = RadarSolution(
        cpa_bearing=320.0,
        cpa_range=1.5,
        cpa_time=datetime.strptime("14:27", "%H:%M"),
        srm=25.0,
        drm=232.0,
        stm=20.0,
        dtm=180.0,
        new_course=46.5,
        new_speed=3.6,
        maneuver_point=(44.0, 5.0),
        e_point=(45.0, 8.0),
        rs_point=(45.0, 5.0),
        rc_point=(46.0, 6.0)
    )

    # Test all field values
    assert solution.cpa_bearing == 320.0
    assert solution.cpa_range == 1.5
    assert isinstance(solution.cpa_time, datetime)
    assert solution.srm == 25.0
    assert solution.drm == 232.0
    assert solution.stm == 20.0
    assert solution.dtm == 180.0
    assert solution.new_course == 46.5
    assert solution.new_speed == 3.6

    # Test tuple fields
    assert isinstance(solution.maneuver_point, tuple)
    assert isinstance(solution.e_point, tuple)
    assert isinstance(solution.rs_point, tuple)
    assert isinstance(solution.rc_point, tuple)

    # Verify tuple contents
    assert solution.maneuver_point == (44.0, 5.0)
    assert solution.e_point == (45.0, 8.0)
    assert solution.rs_point == (45.0, 5.0)
    assert solution.rc_point == (46.0, 6.0)

    # Verify value constraints (all solutions should be valid)
    assert 0 <= solution.cpa_bearing < 360, "CPA bearing should be valid"
    assert solution.cpa_range >= 0, "CPA range should be non-negative"
    assert solution.srm > 0, "SRM should be positive"
    assert 0 <= solution.drm < 360, "DRM should be valid bearing"
    assert solution.stm > 0, "STM should be positive"
    assert 0 <= solution.dtm < 360, "DTM should be valid bearing"
    assert 0 <= solution.new_course < 360, "New course should be valid bearing"
    assert solution.new_speed > 0, "New speed should be positive"

    # Verify CPA time is valid
    assert 0 <= solution.cpa_time.hour < 24, "CPA hour should be 0-23"
    assert 0 <= solution.cpa_time.minute < 60, "CPA minute should be 0-59"
