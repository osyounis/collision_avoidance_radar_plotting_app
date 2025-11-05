"""
Main solver for radar plotting problems.

This module runs all calculations and returns a completed solution.

Author: Omar Younis
Date: 28/10/2025    [dd/mm/yyyy]
"""

from .models import RadarProblem, RadarSolution
from .core.relative_motion import find_srm, find_drm
from .core.cpa import find_cpa_point, find_time_to_cpa
from .core.maneuvers import (
    find_maneuver_point,
    find_nrml_equation,
    find_arml_equation,
    find_e_point,
    find_rs_point,
    find_r_nc,
    find_rc_point,
)
from .core.true_motion import find_dtm, find_stm, find_nc, find_ns


def solver_radar_problem(problem: RadarProblem) -> RadarSolution:
    """
    Solve a complete radar plotting problem.

    Args:
        problem: RadarProblem containing all inputs

    Returns:
        RadarSolution containing all calculated values and points (the answer).
    """
    # Convert to tuple format for functions
    r_point = problem.r_point.to_tuple()
    m_point = problem.m_point.to_tuple()

    # Calculating all points needed to graph the problem on a radar plot.
    # Finding points e, mx, rs, and rc.
    cpa_point = find_cpa_point(r_point, m_point)
    maneuver_point = find_maneuver_point(r_point, m_point, problem.maneuver_dist)
    e_point = find_e_point(r_point, m_point, problem.our_speed)

    # Calculating important lines to find the rest of the required points
    nrml_equ = find_nrml_equation(
        r_point, m_point, maneuver_point, problem.new_cpa_dist
    )
    arml_equ = find_arml_equation(m_point, nrml_equ)
    rs_point = find_rs_point(e_point, arml_equ)
    r_nc = find_r_nc(r_point, m_point, e_point, rs_point, problem.our_speed, arml_equ)
    rc_point = find_rc_point(e_point, r_nc)

    # Finding numerical answers for radar plotting
    drm = find_drm(r_point, m_point)
    srm = find_srm(r_point, m_point)
    tcpa = find_time_to_cpa(r_point, cpa_point, srm)
    dtm = find_dtm(m_point, e_point)
    stm = find_stm(r_point, m_point, e_point)
    nc = find_nc(problem.our_course, r_nc)
    ns = find_ns(r_point, m_point, e_point, rs_point)

    return RadarSolution(
        cpa_bearing=cpa_point[0],
        cpa_range=cpa_point[1],
        cpa_time=tcpa,
        srm=srm,
        drm=drm,
        stm=stm,
        dtm=dtm,
        new_course=nc,
        new_speed=ns,
        maneuver_point=maneuver_point,
        e_point=e_point,
        rs_point=rs_point,
        rc_point=rc_point,
    )
