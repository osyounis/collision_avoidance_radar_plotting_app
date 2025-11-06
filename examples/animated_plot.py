"""
Plotting Example: Displays a radar plot solution. with matplotlib

Author: Omar Younis
Date: 30/10/2025    [dd/mm/yyyy]
"""

from radar_plotter.models import RadarPoint, RadarProblem
from radar_plotter.solver import solver_radar_problem
from radar_plotter.plotting.radar_plot import plot_radar_solution


def main():
    # Defines a problem
    problem = RadarProblem(
        our_course=0.0,
        our_speed=10.0,
        maneuver_dist=5.0,
        new_cpa_dist=2.5,
        r_point=RadarPoint(45.0, 11.5, "14:00"),
        m_point=RadarPoint(43.0, 9.0, "14:06"),
    )

    # Solution to the problem
    solution = solver_radar_problem(problem)

    # Plot the solution (this will display the plot with the vector arrows)
    plot_radar_solution(problem, solution, show=True)


if __name__ == "__main__":
    main()
