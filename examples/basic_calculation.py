"""
Basic example: Calculate a radar plotting solution.

Author: Omar Younis
Date: 30/10/2025    [dd/mm/yyyy]
"""

from radar_plotter.models import RadarPoint, RadarProblem
from radar_plotter.solver import solver_radar_problem


def main():
    # Define the problem to solve
    problem = RadarProblem(
        our_course=0.0,
        our_speed=10.0,
        maneuver_dist=5.0,
        new_cpa_dist=2.5,
        r_point=RadarPoint(45.0, 11.5, "14:00"),
        m_point=RadarPoint(43.0, 9.0, "14:06")
    )

    # Solve the problem
    solution = solver_radar_problem(problem)

    # Print results
    print("===== Collision Avoidance Radar Plot Solution =====\n")
    print(f"CPA: {solution.cpa_range:.1f} NM at bearing {solution.cpa_bearing:06.2f}°")
    print(f"Time to CPA: {solution.cpa_time.strftime('%H:%M')}")
    print("\nRelative Motion:")
    print(f"    SRM: {solution.srm:.1f} kts")
    print(f"    DRM: {solution.drm:06.2f}°")
    print("\nRecommended Maneuver:")
    print(f"    New Course: {solution.new_course:06.2f}°")
    print(f"    New Speed: {solution.new_speed:.1f} kts")


if __name__ == "__main__":
    main()
