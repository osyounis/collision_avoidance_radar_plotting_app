"""
Pre-defined radar plotting scenarios for testing and examples.

Author: Omar Younis
Date: 30/10/2025    [dd/mm/yyyy]
"""

from .models import RadarProblem, RadarPoint


# Example scenario from the default values from the Streamlit app
SCENARIO_1 = RadarProblem(
    our_course=0.0,
    our_speed=10.0,
    maneuver_dist=5.0,
    new_cpa_dist=5.0,
    r_point=RadarPoint(45.0, 11.5, "14:00"),
    m_point=RadarPoint(43.0, 9.0, "14:06")
)

# Another scenario
SCENARIO_2 = RadarProblem(
    our_course=90.0,
    our_speed=15.0,
    maneuver_dist=6.0,
    new_cpa_dist=3.0,
    r_point=RadarPoint(45.0, 12.0, "10:00"),
    m_point=RadarPoint(42.0, 9.5, "10:08")
)

# Head-on scenario
SCENARIO_3 = RadarProblem(
    our_course=0.0,
    our_speed=18.0,
    maneuver_dist=4.0,
    new_cpa_dist=2.5,
    r_point=RadarPoint(0.0, 10.0, "08:00"),
    m_point=RadarPoint(358.0, 7.5, "08:05")
)

# Crossing bow scenario
SCENARIO_4 = RadarProblem(
    our_course=270.0,
    our_speed=12.0,
    maneuver_dist=5.5,
    new_cpa_dist=3.0,
    r_point=RadarPoint(315.0, 8.0, "16:00"),
    m_point=RadarPoint(320.0, 6.0, "16:10")
)

# List of all scenarios
ALL_SCENARIOS = [SCENARIO_1, SCENARIO_2, SCENARIO_3, SCENARIO_4]
