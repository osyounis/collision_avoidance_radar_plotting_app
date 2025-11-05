"""
Radar plot visualization functions.

Author: Omar Younis
Date: 28/10/2025    [dd/mm/yyyy]
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.projections.polar import PolarAxes
from typing import cast

from ..models import RadarProblem, RadarSolution
from ..core.coordinates import bearing_to_cartesian, cartesian_to_bearing
from ..core.relative_motion import find_line_equation
from ..core.maneuvers import find_nrml_equation, find_arml_equation


def plot_radar_solution(
    problem: RadarProblem, solution: RadarSolution, show: bool = True
) -> Figure:
    """
    Create a radar plot visualization of the solution with vector arrows.

    Styled to resemble a traditional radar transfer plotting sheet
    (maneuvering board) used in maritime navigation.

    Args:
        problem: The radar problem
        solution: The calculated solution
        show: Whether to display the plot immediately

    Returns:
        matplotlib Figure object
    """
    # Setting up polar graph
    fig, temp_ax = plt.subplots(subplot_kw={"projection": "polar"}, figsize=(10, 10))
    ax = cast(PolarAxes, temp_ax)

    # Plot origin (own ship)
    ax.scatter(0, 0, s=500, marker="+", lw=0.625, c="k")

    # Get points as tuples
    r_point = problem.r_point.to_tuple()
    m_point = problem.m_point.to_tuple()

    # First points to plot (r, m, e)
    first_step_points_t = [r_point[0], m_point[0], solution.e_point[0]]
    first_step_points_r = [r_point[1], m_point[1], solution.e_point[1]]
    first_step_point_labels = [" r", " m", " e"]
    ax.scatter(
        np.deg2rad(first_step_points_t),
        first_step_points_r,
        s=75,
        marker="x",
        lw=1,
        c="k",
    )
    for i, label in enumerate(first_step_point_labels):
        ax.annotate(
            label, (np.deg2rad(first_step_points_t[i] + 1), first_step_points_r[i])
        )

    # Draw maneuver ring
    theta = np.arange(0, 2 * np.pi, 0.01)
    radius = np.full(len(theta), problem.maneuver_dist)
    ax.plot(
        theta,
        radius,
        lw=1.25,
        c="tab:orange",
        linestyle="dashed",
        label="Maneuver Distance",
    )

    # Draw keep out distance circle
    radius_keepout = np.full(len(theta), problem.new_cpa_dist)
    ax.plot(
        theta,
        radius_keepout,
        lw=1.25,
        c="tab:red",
        linestyle="dotted",
        label="Keep Out Distance",
    )

    # Draw maneuver point
    ax.scatter(
        np.deg2rad(solution.maneuver_point[0]),
        solution.maneuver_point[1],
        s=150,
        marker="x",
        lw=1,
        c="k",
    )
    ax.annotate(
        " Mx", (np.deg2rad(solution.maneuver_point[0] + 3), solution.maneuver_point[1])
    )

    # Draw New course change point and new speed point
    final_points_t = [solution.rs_point[0], solution.rc_point[0]]
    final_points_r = [solution.rs_point[1], solution.rc_point[1]]
    final_point_labels = [" rs", " rc"]
    ax.scatter(
        np.deg2rad(final_points_t), final_points_r, s=75, marker="x", lw=1, c="k"
    )
    for i, label in enumerate(final_point_labels):
        ax.annotate(label, (np.deg2rad(final_points_t[i] + 1), final_points_r[i]))

    # Draw RML (Relative Motion Line)
    m, c = find_line_equation(r_point, m_point, cartesian=False)
    temp_x = -4
    temp_y = (m * temp_x) + c
    temp_theta, temp_r = cartesian_to_bearing(temp_x, temp_y)
    theta_vals = [np.deg2rad(r_point[0]), np.deg2rad(temp_theta)]
    radii = [r_point[1], temp_r]
    ax.plot(theta_vals, radii, c="mediumblue", lw=1, label="RML")

    # CPA line from origin to RML
    thetas = [0, np.deg2rad(solution.cpa_bearing)]
    radii = [0, solution.cpa_range]
    ax.plot(thetas, radii, c="tab:red", lw=1, label="CPA")

    # Adding NRML (New Relative Motion Line)
    nrml_equ = find_nrml_equation(
        r_point, m_point, solution.maneuver_point, problem.new_cpa_dist
    )
    arml_equ = find_arml_equation(m_point, nrml_equ)

    temp_x = -5
    temp_y = (nrml_equ[0] * temp_x) + nrml_equ[1]
    temp_theta, temp_r = cartesian_to_bearing(temp_x, temp_y)
    thetas = [np.deg2rad(solution.maneuver_point[0]), np.deg2rad(temp_theta)]
    radii = [solution.maneuver_point[1], temp_r]
    ax.plot(thetas, radii, c="darkgreen", lw=1, label="NRML")

    # Adding ARML (Actual Relative Motion Line) - Extended to edge
    plot_radius = 12
    a_arml = 1 + (arml_equ[0] ** 2)
    b_arml = 2 * arml_equ[0] * arml_equ[1]
    c_arml = (arml_equ[1] ** 2) - (plot_radius**2)

    x_arml_1 = (-b_arml + np.sqrt((b_arml**2) - (4 * a_arml * c_arml))) / (2 * a_arml)
    x_arml_2 = (-b_arml - np.sqrt((b_arml**2) - (4 * a_arml * c_arml))) / (2 * a_arml)

    y_arml_1 = (arml_equ[0] * x_arml_1) + arml_equ[1]
    y_arml_2 = (arml_equ[0] * x_arml_2) + arml_equ[1]

    # Determine which direction to extend based on rs_point direction
    m_x, m_y = bearing_to_cartesian(m_point[0], m_point[1])
    rs_x, rs_y = bearing_to_cartesian(solution.rs_point[0], solution.rs_point[1])

    direction_x = rs_x - m_x
    direction_y = rs_y - m_y

    vec1_x = x_arml_1 - m_x
    vec1_y = y_arml_1 - m_y
    vec2_x = x_arml_2 - m_x
    vec2_y = y_arml_2 - m_y

    dot1 = vec1_x * direction_x + vec1_y * direction_y
    dot2 = vec2_x * direction_x + vec2_y * direction_y

    if dot1 > dot2:
        temp_theta, temp_r = cartesian_to_bearing(x_arml_1, y_arml_1)
    else:
        temp_theta, temp_r = cartesian_to_bearing(x_arml_2, y_arml_2)

    thetas = [np.deg2rad(m_point[0]), np.deg2rad(temp_theta)]
    radii = [m_point[1], temp_r]
    ax.plot(thetas, radii, c="darkviolet", lw=1, label="ARML")

    # Add Vertical Line
    e_x, _ = bearing_to_cartesian(solution.e_point[0], solution.e_point[1])
    temp_y = 5
    temp_theta, temp_r = cartesian_to_bearing(e_x, temp_y)
    thetas = [np.deg2rad(r_point[0]), np.deg2rad(temp_theta)]
    radii = [r_point[1], temp_r]
    ax.plot(thetas, radii, c="k", lw=1)

    # Adding rs vector line (with arrow) - Speed Change
    ax.annotate(
        "",
        xy=(np.deg2rad(solution.rs_point[0]), solution.rs_point[1]),
        xytext=(np.deg2rad(solution.e_point[0]), solution.e_point[1]),
        arrowprops=dict(arrowstyle="->", lw=2, color="red"),
    )

    # Adding rc vector line (with arrow) - Course Change
    ax.annotate(
        "",
        xy=(np.deg2rad(solution.rc_point[0]), solution.rc_point[1]),
        xytext=(np.deg2rad(solution.e_point[0]), solution.e_point[1]),
        arrowprops=dict(arrowstyle="->", lw=2, color="green"),
    )

    # === TRADITIONAL RADAR PLOTTING SHEET STYLING ===

    # Setting up theta (bearing) - North at top, clockwise
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)

    # Bearing lines every 10 degrees with labels every 30 degrees
    ax.set_thetagrids(
        np.arange(0, 360, 30),  # Label every 30°
        labels=[f"{i:03d}°" for i in range(0, 360, 30)],  # Format: 000°, 030°, etc.
    )

    # Show all tick marks every 10 degrees (like a traditional sheet)
    ax.set_xticks(np.deg2rad(np.arange(0, 360, 10)))

    # Enable radial grid lines (bearing lines radiating from center)
    ax.grid(
        True,
        which="major",
        axis="both",
        linestyle="-",
        linewidth=0.5,
        color="gray",
        alpha=0.4,
    )

    # Setting Up Radius (Range) Ticks
    ax.set_rticks([2, 4, 6, 8, 10, 12])
    ax.set_rlabel_position(355.5)
    ax.set_rmax(12)

    # Style the radial (range) grid lines to look more traditional
    ax.yaxis.grid(True, color="gray", linestyle="-", linewidth=0.5, alpha=0.4)

    # Adding Legend with better positioning
    ax.legend(loc="upper right", bbox_to_anchor=(1.15, 1.1), framealpha=0.9)

    # Adding title
    ax.set_title(
        "Collision Avoidance Radar Plot", pad=20, fontsize=14, fontweight="bold"
    )

    plt.tight_layout()

    if show:
        plt.show()

    return fig
