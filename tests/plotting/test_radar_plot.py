"""
Tests the radar plotting functions.

Author: Omar Younis
Date: 04/11/2025    [dd/mm/yyyy]
"""

from datetime import datetime

import numpy as np

import matplotlib

matplotlib.use("Agg")  # Non-interactive backend for testing
from matplotlib.figure import Figure
from matplotlib.projections import PolarAxes


from radar_plotter.models import RadarProblem, RadarPoint, RadarSolution
from radar_plotter.plotting.radar_plot import plot_radar_solution


def test_plot_radar_solution_returns_figure():
    """Test that plot_radar_solution returns a Figure object."""
    problem = RadarProblem(
        our_course=0.0,
        our_speed=10.0,
        maneuver_dist=5.0,
        new_cpa_dist=2.5,
        r_point=RadarPoint(45.0, 11.5, "14:00"),
        m_point=RadarPoint(43.0, 9.0, "14:06"),
    )

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
        rc_point=(46.0, 6.0),
    )

    fig = plot_radar_solution(problem, solution, show=False)

    assert isinstance(fig, Figure)
    assert len(fig.axes) > 0  # Should have at least one axis


def test_plot_radar_solution_no_errors():
    """Test that plotting produces valid figure with expected elements."""
    problem = RadarProblem(
        our_course=0.0,
        our_speed=10.0,
        maneuver_dist=5.0,
        new_cpa_dist=2.5,
        r_point=RadarPoint(45.0, 11.5, "14:00"),
        m_point=RadarPoint(43.0, 9.0, "14:06"),
    )

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
        rc_point=(46.0, 6.0),
    )

    # Should not raise any exceptions
    fig = plot_radar_solution(problem, solution, show=False)

    # Verify figure is valid
    assert isinstance(fig, Figure)
    assert len(fig.axes) > 0, "Figure should have at least one axis"

    # Get the polar axis
    ax = fig.axes[0]

    # Verify it's a polar plot
    assert isinstance(ax, PolarAxes), "Plot should use polar projection"

    # Verify plot has content (lines, points, etc.)
    assert len(ax.lines) > 0, "Plot should have lines (RML, CPA, NRML, ARML)"
    assert len(ax.collections) > 0, (
        "Plot should have scatter points (r, m, e, mx, rs, rc)"
    )

    # Verify plot has a title
    assert ax.get_title() != "", "Plot should have a title"

    # Verify plot has a legend
    legend = ax.get_legend()
    assert legend is not None, "Plot should have a legend"
    assert len(legend.get_texts()) > 0, "Legend should have entries"

    # Verify theta (bearing) is set correctly (North at top, clockwise)
    # Now that we know it's a PolarAxes, we can safely call these methods
    assert ax.get_theta_direction() == -1, "Bearings should increase clockwise"
    assert np.isclose(ax.get_theta_offset(), np.pi / 2, atol=0.01), (
        "North (0°) should be at top"
    )
