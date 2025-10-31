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

    # Initialize session state for active tab
    if 'active_tab' not in st.session_state:
        st.session_state.active_tab = "Calculator"

    st.title("⚓️ Collision Avoidance Radar Plotting App")
    st.markdown("Calculate collision avoidance maneuvers using radar plotting techniques")

    # Disclaimer
    st.warning("⚠️ **Disclaimer**: This is an educational tool ONLY and should NOT be used for real collision avoidance situations. This is for training purposes ONLY.")

    # Custom CSS for tab-like buttons (Coast Guard Auxiliary colors)
    st.markdown("""
        <style>
        /* Style for our custom tab buttons */
        div.stButton > button {
            height: 50px;
            border-radius: 8px;
            font-size: 16px;
            font-weight: 600;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }
        </style>
    """, unsafe_allow_html=True)

    # Create custom tab buttons using columns
    col1, col2 = st.columns(2)

    with col1:
        if st.button("📊 Calculator", use_container_width=True,
                     type="primary" if st.session_state.active_tab == "Calculator" else "secondary",
                     key="calc_tab_btn"):
            st.session_state.active_tab = "Calculator"
            st.rerun()

    with col2:
        if st.button("ℹ️ Instructions", use_container_width=True,
                     type="primary" if st.session_state.active_tab == "Instructions" else "secondary",
                     key="instr_tab_btn"):
            st.session_state.active_tab = "Instructions"
            st.rerun()

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
            # Switch to Calculator tab
            st.session_state.active_tab = "Calculator"
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

            # Store solution in session state
            st.session_state.solution = solution
            st.session_state.problem = problem
            st.session_state.has_solution = True

        except ValueError as e:
            st.session_state.error_message = f"❌ Invalid input: {str(e)}"
            st.session_state.has_solution = False
        except Exception as e:
            st.session_state.error_message = f"❌ Error calculating solution: {str(e)}"
            st.session_state.error_exception = e
            st.session_state.has_solution = False

        # Rerun to update the tab button styling
        st.rerun()

    # Display content based on active tab
    if st.session_state.active_tab == "Calculator":
        # Calculator Tab Content
        if 'has_solution' in st.session_state and st.session_state.has_solution:
            st.header("📊 Results")

            col1, col2, col3 = st.columns(3)

            with col1:
                st.metric("CPA Distance", f"{st.session_state.solution.cpa_range:.1f} NM")
                st.metric("CPA Bearing", f"{st.session_state.solution.cpa_bearing:06.2f}°")
                st.metric("Time to CPA", st.session_state.solution.cpa_time.strftime("%H:%M"))

            with col2:
                st.metric("SRM (Speed Relative Movement)", f"{st.session_state.solution.srm:.1f} kts")
                st.metric("DRM (Direction Relative Movement)", f"{st.session_state.solution.drm:06.2f}°")
                st.metric("STM (Speed True Movement)", f"{st.session_state.solution.stm:.1f} kts")

            with col3:
                st.metric("DTM (Direction True Movement)", f"{st.session_state.solution.dtm:06.2f}°")
                st.metric("New Course (N/C)", f"{st.session_state.solution.new_course:06.2f}°")
                st.metric("New Speed (N/S)", f"{st.session_state.solution.new_speed:.1f} kts")

            # Plot
            st.header("📈 Radar Plot")
            fig = plot_radar_solution(st.session_state.problem, st.session_state.solution, show=False)
            st.pyplot(fig)

            # Success message
            st.success("✅ Solution calculated successfully!")
        elif 'error_message' in st.session_state:
            st.error(st.session_state.error_message)
            if 'error_exception' in st.session_state:
                st.exception(st.session_state.error_exception)
        else:
            st.info("👈 Enter your vessel information and target observations in the sidebar, then click 'Calculate Solution' to see results.")

    else:  # Instructions tab
        st.header("How to Use This App")

        st.markdown("""
        ### 📋 Step-by-Step Guide

        #### 1. Enter Your Vessel's Information (Sidebar)
        - **Course (°)**: Your current heading in degrees (0-359)
        - **Speed (kts)**: Your current speed in knots

        #### 2. Set Maneuver Parameters
        - **Maneuver Distance (NM)**: How far ahead you want to plan the maneuver
        - **Keep Out Distance (NM)**: Your desired minimum safe distance (CPA) after maneuvering

        #### 3. Enter Target Vessel - First Appearance (Point R)
        Record the first time you observe the target vessel on radar:
        - **Bearing (°)**: Direction to the target (0-359)
        - **Range (NM)**: Distance to the target in nautical miles
        - **Time (HH:MM)**: Time of observation in 24-hour format (e.g., 14:00)

        #### 4. Enter Target Vessel - Second Appearance (Point M)
        Record the second observation (usually 6 minutes later):
        - **Bearing (°)**: New direction to the target
        - **Range (NM)**: New distance to the target
        - **Time (HH:MM)**: Time of this observation

        #### 5. Calculate Solution
        Click the **"🎯 Calculate Solution"** button in the sidebar.

        The app will display:
        - **CPA information**: Distance, bearing, and time of closest approach
        - **Relative Motion**: SRM (speed) and DRM (direction) of target relative to you
        - **True Motion**: STM (speed) and DTM (direction) of target's actual movement
        - **Recommended Maneuver**: New course and speed to maintain safe distance
        - **Radar Plot**: Visual representation of the situation

        ---

        ### 📚 Understanding the Results

        **CPA (Closest Point of Approach)**
        - The minimum distance the target will pass if no action is taken

        **SRM & DRM**
        - Speed and Direction of Relative Movement
        - How the target appears to move from your perspective

        **STM & DTM**
        - Speed and Direction of True Movement
        - The target's actual course and speed

        **N/C & N/S**
        - New Course and New Speed
        - Recommended changes to achieve your desired keep-out distance

        ---

        ### ⚠️ Important Notes

        - Times must be in **HH:MM** format (e.g., 14:30)
        - All bearings are in **degrees** (0-359, where 0° is North)
        - All distances are in **nautical miles**
        - All speeds are in **knots**
        - This is an **educational tool ONLY** - not for real navigation!

        ---

        ### 💡 Tips for Accurate Results

        1. Take observations at least 6 minutes apart for better accuracy
        2. Ensure your vessel maintains steady course and speed between observations
        3. Use precise bearing and range measurements from your radar
        4. Double-check all input values before calculating
        """)


if __name__ == "__main__":
    main()
