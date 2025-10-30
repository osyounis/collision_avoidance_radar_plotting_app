"""
A Steamlit web app for the Collision Avoidance Radar Plotting App.

This provides a user-friendly interface.

Author: Omar Younis
Date: 28/10/2025    [dd/mm/yyyy]
"""

import streamlit as st

from src.radar_plotter.models import RadarPoint, RadarProblem
from src.radar_plotter.solver import solver_radar_problem
from src.radar_plotter.plotting.radar_plot import plot_radar_solution


def main():
    """
    Main code which runs the app.
    """
    st.set_page_config(
        page_title="Radar Plotting App",
        page_icon="🎯",
        layout="wide"
    )

    st.title("⚓️ Collision Avoidance Radar Plotting App")
    st.markdown("Calculate collusion avoidance maneuvers using radar plotting techniques")

    # Disclaimer
    st.warning("⚠️ **Disclaimer**: This is an educational tool ONLY and should NOT be used for real collision avoidance situations. This is for training purposes ONLY.")

    # Sidebar for inputs
    st.sidebar.header("📋 Input Parameters")

    # Own ship information
    st.sidebar.subheader("Your Vessel")
    our_course = st.sidebar.number_input(
        "Course (°)",
        min_value=0.0,
        max_value=359.0,
        value=0.0,
        step=1.0
        )
    our_speed = st.sidebar.number_input(
        "Speed (kts)",
        min_value=0.0,
        max_value=50.0,
        value=10.0,
        step=0.1
    )

    # Maneuver parameters
    st.sidebar.subheader("Maneuver Parameters")
    maneuver_dist = st.sidebar.number_input(
        "Maneuver Distance (NM)",
        min_value=0.1,
        max_value=20.0,
        value=5.0,
        step=0.1
    )
    new_cpa_dist = st.sidebar.number_input(
        "Keep Out Distance (NM)",
        min_value=0.1,
        max_value=20.0,
        value=2.5,
        step=0.1
    )

    # Target Vessel (First position [Point R])
    st.sidebar.subheader("Target Vessel - First Appearance")
    r_bearing = st.sidebar.number_input(
        "Bearing (°)",
        min_value=0.0,
        max_value=359.0,
        value=45.0,
        step=1.0,
        key="r_bearing"
    )
    r_ranage = st.sidebar.number_input(
        "Range (NM)",
        min_value=0.1,
        max_value=50.0,
        value=11.5,
        step=0.1,
        key="r_range"
    )
    r_time = st.sidebar.text_input(
        "Time (HH:MM) [24hr]",
        value="14:00",
        key="r_time"
    )

    # Target Vessel (Second position [Point M])
    st.sidebar.subheader("Target Vessel - Second Appearance")
    m_bearing = st.sidebar.number_input(
        "Bearing (°)",
        min_value=0.0,
        max_value=359.0,
        value=43.0,
        step=1.0,
        key="m_bearing"
    )
    m_ranage = st.sidebar.number_input(
        "Range (NM)",
        min_value=0.1,
        max_value=50.0,
        value=9.0,
        step=0.1,
        key="m_range"
    )
    m_time = st.sidebar.text_input(
        "Time (HH:MM) [24hr]",
        value="14:06",
        key="m_time"
    )

    # Calculate Button
    if st.sidebar.button("🎯 Calculate Solution", type="primary"):
        try:
            # Create problem
            problem = RadarProblem(
                our_course = our_course,
                our_speed = our_speed,
                maneuver_dist = maneuver_dist,
                new_cpa_dist = new_cpa_dist,
                r_point = RadarPoint(r_bearing, r_ranage, r_time),
                m_point = RadarPoint(m_bearing, m_ranage, m_time)
            )

            # Solve
            solution = solver_radar_problem(problem)

            # Display results
            st.header("📊 Results")

            col1, col2, col3 = st.columns(3)

            with col1:
                st.metric("CPA Distance", f"{solution.cpa_range:.1f} NM")
                st.metric("CPA Bearing", f"{solution.cpa_bearing:06.2f}°")
                st.metric("Time to CPA", solution.cpa_time.strftime("%H:%M"))

            with col2:
                st.metric("SRM (Speed Relative Movement)", f"{solution.srm:.1f} kts")
                st.metric("DRM (Direction Relative Movement)", f"{solution.drm:06.2f}°")
                st.metric("STM (Speed True Movement)", f"{solution.stm:.1f} kts")

            with col3:
                st.metric("DTM (Direction True Movement)", f"{solution.dtm:06.2f}°")
                st.metric("New Course (N/C)", f"{solution.new_course:06.2f}°")
                st.metric("New Speed (N/S)", f"{solution.new_speed:.1f} kts")

            # Plot
            st.header("📈 Radar Plot")
            fig = plot_radar_solution(problem, solution, show=False)
            st.pyplot(fig)

            # Success message
            st.success("✅ Solution calculated successfully!")

        except ValueError as e:
            st.error(f"❌ Invalid input: {str(e)}")
        except Exception as e:
            st.error(f"❌ Error calculating solution: {str(e)}")
            st.exception(e)

    # Instructions
    with st.expander("ℹ️ How to Use"):
        st.markdown("""
        1. **Enter your vessel's information**: Course and speed
        2. **Set maneuver parameters**:
           - Maneuver Distance: How far ahead to plan the maneuver
           - Keep Out Distance: Desired closest point of approach after maneuver
        3. **Enter target vessel's First Appearance**: First radar observation (bearing, range, time)
        4. **Enter target vessel's Second Appearance**: Second radar observation (bearing, range, time)
        5. **Click Calculate** to see the solution and radar plot

        The app will calculate the required course and speed changes to achieve the desired CPA.

        **Note**: Times must be in HH:MM format (e.g., 14:30)
        """)


if __name__ == "__main__":
    main()
