"""
Tests of the main solver.

Author: Omar Younis
Date: 31/10/2025    [dd/mm/yyyy]
"""


from datetime import datetime

from altair import Key
import pytest
import numpy as np

from radar_plotter.models import RadarPoint, RadarProblem
from radar_plotter.solver import solver_radar_problem


def test_solve_scenario_1_with_known_solution():
    """
    Tests solving the default Streamlit scenario with known correct values.
    
    This scenario represents a typical radar plotting problem where:
        - Own ship on course 0° (North) at 10 kts
        - Target first observed at bearing 45°, range 11.5 NM at 14:00
        - Target second observed at bearing 43°, range 9.0 NM at 14:06
        - Want maneuver at 5 NM ahead, keeping out distance 2.5 NM
    """
    problem = RadarProblem(
        our_course=0.0,
        our_speed=10.0,
        maneuver_dist=5.0,
        new_cpa_dist=2.5,
        r_point=RadarPoint(45.0, 11.5, "14:00"),
        m_point=RadarPoint(43.0, 9.0, "14:06")
    )

    solution = solver_radar_problem(problem)

    # Test CPA values with tight tolerances (±0.1 NM, ±1°)
    assert np.isclose(solution.cpa_range, 1.43, atol=0.1), \
        f"CPA range should be 1.43 NM ±0.1 NM, got {solution.cpa_range:.2f}"
    assert np.isclose(solution.cpa_bearing, 322.15, atol=1.0), \
        f"CPA bearing should be 322.15° ±1°, got {solution.cpa_bearing:.2f}"

    # Test time to CPA (should be 14:27, ±1 minute tolerance)
    assert isinstance(solution.cpa_time, datetime)
    expected_time = datetime.strptime("14:27", "%H:%M")
    time_diff_minutes = abs(((solution.cpa_time.hour * 60) + solution.cpa_time.minute)
                            - ((expected_time.hour * 60) + expected_time.minute))
    assert time_diff_minutes <= 1, \
        f"CPA time should be 14:27 ±1 minute, got {solution.cpa_time.strftime('%H:%M')}"

    # Test relative motion values (±0.5 kts, ±1°)
    assert np.isclose(solution.srm, 25.25, atol=0.5), \
        f"SRM should be 25.25 kts ±0.5 kts, got {solution.srm:.2f}"
    assert np.isclose(solution.drm, 232.15, atol=1.0), \
        f"DRM should be 232.15° ±1°, got {solution.drm:.2f}"

    # Test true motion values (±0.5 kts, ±1°)
    assert np.isclose(solution.stm, 20.68, atol=0.5), \
        f"STM should be 20.68 kts ±0.5 kts, got {solution.stm:.2f}"
    assert np.isclose(solution.dtm, 254.59, atol=1.0), \
        f"DTM should be 254.59° ±1°, got {solution.dtm:.2f}"

    # Test maneuver recommendations (±1°, ±0.2 kts)
    assert np.isclose(solution.new_course, 46.50, atol=1.0), \
        f"New course should be 46.50° ±1°, got {solution.new_course:.2f}"
    assert np.isclose(solution.new_speed, 3.58, atol=0.2), \
        f"New speed should be 3.58 kts ±0.2 kts, got {solution.new_speed:.2f}"

    # Test maneuver point (bearing ±1°, range exact at 5.0 NM)
    assert np.isclose(solution.maneuver_point[0], 35.52, atol=1.0), \
        f"Maneuver point bearing should be 35.52° ±1°, got {solution.maneuver_point[0]:.2f}"
    assert np.isclose(solution.maneuver_point[1], 5.00, atol=0.1), \
        f"Maneuver point range should be 5.00 NM, got {solution.maneuver_point[1]:.2f}"

    # Test E-point (±1°, ±0.2 NM)
    assert np.isclose(solution.e_point[0], 48.75, atol=1.0), \
        f"E-point bearing should be 48.75° ±1°, got {solution.e_point[0]:.2f}"
    assert np.isclose(solution.e_point[1], 10.82, atol=0.2), \
        f"E-point range should be 10.82 NM ±0.2 NM, got {solution.e_point[1]:.2f}"

    # Test RS-point (±1°, ±0.2 NM)
    assert np.isclose(solution.rs_point[0], 47.35, atol=1.0), \
        f"RS-point bearing should be 47.35° ±1°, got {solution.rs_point[0]:.2f}"
    assert np.isclose(solution.rs_point[1], 11.06, atol=0.2), \
        f"RS-point range should 11.06 NM ±0.2 NM, got {solution.rs_point[1]:.2f}"

    # Test RC-point (±1°, ±0.2 NM)
    assert np.isclose(solution.rc_point[0], 48.56, atol=1.0), \
        f"RC-point bearing should be 48.56° ±1°, got {solution.rc_point[0]:.2f}"
    assert np.isclose(solution.rc_point[1], 11.82, atol=0.2), \
        f"RC-point range should be 11.82NM ±0.2 NM, got {solution.rc_point[1]:.2f}"


def test_solve_head_on_scenario():
    """
    Test a head-on collision scenario.
    
    Both vessels on opposite courses. Tests that the solver handles dangerous
    head-on situations per maritime collision regulations.
    """
    problem = RadarProblem(
        our_course=0.0,     # Heading True North
        our_speed=15.0,
        maneuver_dist=4.0,
        new_cpa_dist=3.0,
        r_point=RadarPoint(0.0, 10.0, "10:00"),     # Target dead ahead (first pip on radar)
        m_point=RadarPoint(0.0, 7.0, "10:10")       # Target closer, still dead ahead (second pip)
    )

    solution = solver_radar_problem(problem)

    # In head-on scenario, CPA should be very small if no maneuver
    assert solution.cpa_range < 2.0, "Head-on CPA should be near zero"

    # DRM should be approximately 180° (target moving south towards us)
    assert 170 <= solution.drm <= 190, "DRM should indicate target approaching head-on"

    # SRM should be high (combined speed of both vessels)
    assert solution.srm > 20.0, "SRM should be sum of both speeds in head-on scenario"

    # Maneuver should recommend significant course change
    # Both vessels must turn to starboard (right) in head-on situations per maritime rules:
    # - International: COLREGS Rule 14 (33 CFR § 83.14)
    #   Convention on the International Regulations for Preventing Collisions at Sea, 1972
    # - U.S. Inland: 33 CFR § 84.14
    # Both state: "each shall alter her course to starboard so that each
    # shall pass on the port side of the other."
    #
    # Reference: USCG Navigation Rules & Regulations Handbook (2024)
    #            https://www.navcen.uscg.gov/
    assert abs(solution.new_course - problem.our_course) > 15.0, \
        "Head-on scenario requires significant course change"


def test_solve_overtaking_scenario():
    """
    Testing an overtaking scenario.
    
    We're are the faster vessel catching up to the slower target vessel ahead.
    """
    problem = RadarProblem(
        our_course=90.0,        # Heading East
        our_speed=20.0,         # Fast speed
        maneuver_dist=6.0,
        new_cpa_dist=3.0,
        r_point=RadarPoint(85.0, 8.0, "16:00"),     # Slightly off our bow
        m_point=RadarPoint(88.0, 6.5, "16:12")
    )

    solution = solver_radar_problem(problem)

    # Overtaking scenario characteristics
    assert solution.srm > 5.0, "Should have positive SRM (we're gaining)"
    assert solution.stm < solution.srm, "Target slower than our closing rate"

    # Overtaking requires significant course alteration to clear the vessel
    # Per COLREGS Rule 13, overtaking vessel shall keep clear and alter course
    course_change = abs(solution.new_course - problem.our_course)
    assert course_change > 10.0, \
        f"Overtaking requires significant course change, got {course_change:.1f}°"

    # Verify maneuver point is at requested distance
    assert np.isclose(solution.maneuver_point[1], problem.maneuver_dist, atol=0.5), \
        "Maneuver point should be at requested distance"

    # CPA should initially be small (risk of collision)
    assert solution.cpa_range < problem.new_cpa_dist, \
        "Initial CPA should be less than desired CPA (collision risk exists)"

    # Bearing drift: in overtaking, bearing should be changing
    bearing_change = abs(problem.m_point.bearing - problem.r_point.bearing)
    assert bearing_change > 0.5, \
        f"Bearing should be changing in overtaking scenario, got {bearing_change:.1f}°"


def test_solve_crossing_scenario():
    """
    Tests a crossing scenario from starboard (right).
    
    Target crossing from starboard side (right).
    """
    problem = RadarProblem(
        our_course=270.0,       # Heading West
        our_speed=12.0,
        maneuver_dist=5.5,
        new_cpa_dist=3.0,
        r_point=RadarPoint(315.0, 8.0, "08:00"),        # NW of our vessel
        m_point=RadarPoint(320.0, 6.0, "08:10")         # Closing, bearing opening
    )

    solution = solver_radar_problem(problem)

    # Crossing scenario checks - vessels converging
    assert solution.srm > 0, "Should have positive SRM (vessels converging)"

    # Crossing from starboard requires action by give-way vessel
    # Per COLREGS Rule 15: When two vessels are crossing, the vessel which has
    # the other on her starboard side shall keep out of the way
    course_change = abs(solution.new_course - problem.our_course)
    assert course_change > 5.0, \
        f"Crossing from starboard requires course change, got {course_change:.1f}°"

    # Verify maneuver point is at requested distance
    assert np.isclose(solution.maneuver_point[1], problem.maneuver_dist, atol=0.5), \
        f"Maneuver point should be at {problem.maneuver_dist} NM, \
            got {solution.maneuver_point[1]:.2f} NM"

    # Initial CPA should indicate collision risk
    assert solution.cpa_range < problem.new_cpa_dist, \
        f"Initial CPA ({solution.cpa_range:.2f} NM) should be less \
            than desired CPA ({problem.new_cpa_dist} NM)"

    # Bearing drift: in crossing, bearing should be opening (vessel passing ahead)
    bearing_change = problem.m_point.bearing - problem.r_point.bearing
    assert bearing_change > 0, \
        f"Bearing should be opening (vessel crossing ahead), got change of {bearing_change:.1f}°"

    # DRM should indicate the target's relative direction of approach
    assert 0 <= solution.drm < 360, "DRM should be valid bearing"

    # Verify true motion is calculated
    assert solution.stm > 0, "Target should have positive true speed"
    assert 0 <= solution.dtm < 360, "DTM should be valid bearing"


def test_solve_maintains_calculation_consistency():
    """
    Test that calculations are internally consistent.

    Verifies that intermediate points and vectors make sense together.
    """
    problem = RadarProblem(
        our_course=180.0,
        our_speed=14.0,
        maneuver_dist=5.0,
        new_cpa_dist=2.5,
        r_point=RadarPoint(135.0, 10.0, "12:00"),
        m_point=RadarPoint(140.0, 7.5, "12:08")
    )

    solution = solver_radar_problem(problem)

    # Check that maneuver point is at the correct distance
    assert np.isclose(solution.maneuver_point[1], problem.maneuver_dist, atol=0.5), \
        f"Maneuver point should be at {problem.maneuver_dist} NM, \
            got {solution.maneuver_point[1]:.2f} NM"

    # Verify e_point represents our velocity vector correctly
    # e_point should be based on our speed and time interval between R and M
    r_time = datetime.strptime(problem.r_point.time, "%H:%M")
    m_time = datetime.strptime(problem.m_point.time, "%H:%M")
    time_delta_hours = (m_time - r_time).total_seconds() / 3600
    expected_distance_traveled = problem.our_speed * time_delta_hours

    # e_point range should approximate our velocity vector magnitude
    assert solution.e_point[1] > 0, "E-point should have positive range"
    assert np.isclose(solution.e_point[1], expected_distance_traveled, atol=2.0), \
        f"E-point range should be ~{expected_distance_traveled:.2f} NM (our speed \
            x time), got {solution.e_point[1]:.2f} NM"

    # Verify geometric consistency: maneuver point should lie on RML
    # (between R and M points in terms of relative direction)
    maneuver_bearing_diff = abs(solution.maneuver_point[0] - problem.r_point.bearing)
    if maneuver_bearing_diff > 180:
        maneuver_bearing_diff = 360 - maneuver_bearing_diff
    assert maneuver_bearing_diff < 15, \
        f"Maneuver point bearing should be near RML, got {maneuver_bearing_diff:.1f}° deviation"

    # RS and RC points should be farther from origin than E point
    # (they represent vector endpoints from E)
    assert solution.rs_point[1] >= solution.e_point[1] * 0.5, \
        "RS point should be geometrically related to E point"
    assert solution.rc_point[1] >= solution.e_point[1] * 0.5, \
        "RC point should be geometrically related to E point"

    # CPA should be less than both R and M point ranges (closest approach)
    assert solution.cpa_range <= problem.r_point.range, \
        f"CPA ({solution.cpa_range:.2f} NM) should be ≤ R range ({problem.r_point.range} NM)"
    assert solution.cpa_range <= problem.m_point.range, \
        f"CPA ({solution.cpa_range:.2f} NM) should be ≤ M range ({problem.m_point.range} NM)"

    # Time to CPA should be after M point time (CPA is in the future)
    assert solution.cpa_time >= m_time, \
        f"CPA time ({solution.cpa_time:%H:%M}) should be after M time ({m_time:%H:%M})"


def test_solve_with_different_time_intervals():
    """
    Test that solver works with different observation intervals.

    Shorter interval: 3 minutes (vs typical 6-12 minutes)
    """
    problem = RadarProblem(
        our_course=45.0,
        our_speed=10.0,
        maneuver_dist=5.0,
        new_cpa_dist=2.5,
        r_point=RadarPoint(30.0, 12.0, "14:00"),
        m_point=RadarPoint(32.0, 11.0, "14:03")     # Only 3 minute difference
    )

    solution = solver_radar_problem(problem)

    # Verify time interval is correctly used in calculations
    r_time = datetime.strptime(problem.r_point.time, "%H:%M")
    m_time = datetime.strptime(problem.m_point.time, "%H:%M")
    time_delta_minutes = (m_time - r_time).total_seconds()  / 60
    assert time_delta_minutes == 3.0, "Test setup: should be 3 minute intervals"

    # SRM calculation should work with short intervals
    # SRM = distance / time, so with short intervals, even small movements show high speeds
    assert solution.srm > 0, "SRM should be positive (target moving)"

    # Range change: 12.0 -> 11.0 = 1.0 NM in 3 minutes (0.05 hours)
    # Minimum expected SRM ~ 1.0 / 0.05 = 20 kts (rough estimate)
    assert solution.srm >= 15.0, \
        f"With 1 NM range change in 3 min, SRM should be significant, got {solution.srm:.1f} kts"

    # DRM should be calculated correctly
    assert 0 <= solution.drm < 360, "DRM should be a valid bearing"

    # Bearing drift: 30° -> 32° = 2° change
    bearing_change = problem.m_point.bearing - problem.r_point.bearing
    assert bearing_change == 2.0, "Test setup: bearing changed by 2°"

    # E-point should reflect our 3 minute travel
    time_delta_hours = time_delta_minutes / 60
    expected_e_range = problem.our_speed * time_delta_hours     # 10 kts x 0.05hr = 0.5 NM
    assert np.isclose(solution.e_point[1], expected_e_range, atol=0.5), \
        f"E-point should be ~{expected_e_range:.2f} NM (our speed x time), \
            got {solution.e_point[1]:.2f} NM"

    # CPA time should still be calculated correctly
    assert solution.cpa_time >= m_time, \
        "CPA time should be after m_point is observed on radar"

    # Maneuver recommendation should still be valid
    assert 0 <= solution.new_course < 360, "New course should be valid bearing"
    assert solution.new_speed > 0, "New speed should be positive"

    # Maneuver point should still be at requested distance
    assert np.isclose(solution.maneuver_point[1], problem.maneuver_dist, atol=0.5), \
        f"Maneuver point should be at {problem.maneuver_dist} NM, \
            got {solution.maneuver_point[1]:.2f} NM"


def test_solve_returns_all_required_fields():
    """
    Tests that RadarSolution contains all required fields.
    
    This is a basic smoke test to make sure nothing is missing.
    """
    problem = RadarProblem(
        our_course=0.0,
        our_speed=10.0,
        maneuver_dist=5.0,
        new_cpa_dist=2.5,
        r_point=RadarPoint(45.0, 11.5, "14:00"),
        m_point=RadarPoint(43.0, 9.0, "14:06")
    )

    solution = solver_radar_problem(problem)

    # Check all required fields exist
    assert hasattr(solution, 'cpa_bearing')
    assert hasattr(solution, 'cpa_range')
    assert hasattr(solution, 'cpa_time')
    assert hasattr(solution, 'srm')
    assert hasattr(solution, 'drm')
    assert hasattr(solution, 'stm')
    assert hasattr(solution, 'dtm')
    assert hasattr(solution, 'new_course')
    assert hasattr(solution, 'new_speed')
    assert hasattr(solution, 'maneuver_point')
    assert hasattr(solution, 'e_point')
    assert hasattr(solution, 'rs_point')
    assert hasattr(solution, 'rc_point')

    # Check types
    assert isinstance(solution.cpa_bearing, float)
    assert isinstance(solution.cpa_range, float)
    assert isinstance(solution.cpa_time, datetime)
    assert isinstance(solution.srm, float)
    assert isinstance(solution.drm, float)
    assert isinstance(solution.stm, float)
    assert isinstance(solution.dtm, float)
    assert isinstance(solution.new_course, float)
    assert isinstance(solution.new_speed, float)
    assert isinstance(solution.maneuver_point, tuple)
    assert isinstance(solution.e_point, tuple)
    assert isinstance(solution.rs_point, tuple)
    assert isinstance(solution.rc_point, tuple)
