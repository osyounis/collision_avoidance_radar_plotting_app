# ⚓ Collision Avoidance Radar Plotting App

A Python application for calculating collision avoidance radar plots to help maritime navigators train and practice this essential skill.

[![Tests](https://github.com/osyounis/collision_avoidance_radar_plotting_app/actions/workflows/tests.yml/badge.svg)](https://github.com/osyounis/collision_avoidance_radar_plotting_app/actions/workflows/tests.yml)
[![codecov](https://codecov.io/gh/osyounis/collision_avoidance_radar_plotting_app/graph/badge.svg)](https://codecov.io/gh/osyounis/collision_avoidance_radar_plotting_app)
![Python](https://img.shields.io/badge/python-3.12-blue?logo=python&logoColor=white)
![License](https://img.shields.io/github/license/osyounis/collision_avoidance_radar_plotting_app)

---

## 📋 Features

- **Calculate CPA** (Closest Point of Approach)
- **Determine maneuvers** for collision avoidance
- **Visualize radar plots** with interactive graphics and vector arrows
- **Web interface** for non-technical users (Streamlit)
- **Python API** for advanced users
- **Educational tool** for Coast Guard Auxiliary and maritime training

---

## 🚀 Quick Start

### Installation

```bash
# Clone the repository
git clone https://github.com/osyounis/collision_avoidance_radar_plotting_app.git
cd collision_avoidance_radar_plotting_app

# Install with uv (recommended)
uv sync

# Or with pip
pip install -e .
```

### Running the Web App

```bash
# With uv
uv run streamlit run app.py

# Or directly
streamlit run app.py
```

Then open your browser to `http://localhost:8501`

---

## 📖 Usage

### Web Interface (Recommended for most users)

1. Run the Streamlit app (see above)
2. Enter your vessel's course and speed
3. Set maneuver parameters (Maneuver Distance and Keep Out Distance)
4. Enter contact vessel observations (Point R and Point M)
5. Click "Calculate Solution"

### Python API

```python
from radar_plotter.models import RadarProblem, RadarPoint
from radar_plotter.solver import solver_radar_problem

# Define the problem
problem = RadarProblem(
    our_course=0.0,
    our_speed=10.0,
    maneuver_dist=5.0,
    new_cpa_dist=2.5,
    r_point=RadarPoint(45.0, 11.5, "14:00"),
    m_point=RadarPoint(43.0, 9.0, "14:06")
)

# Solve
solution = solver_radar_problem(problem)

print(f"CPA: {solution.cpa_range:.1f} NM at {solution.cpa_time:%H:%M}")
print(f"New Course: {solution.new_course:.0f}°")
print(f"New Speed: {solution.new_speed:.0f} kts")
```

### Command Line Examples

```bash
# Run a basic calculation
uv run python examples/basic_calculation.py

# Display an animated radar plot
uv run python examples/animated_plot.py
```

---

## 🧪 Development

### Running Tests

```bash
uv run pytest                                    # Run all tests
uv run pytest -v                                 # Verbose output
uv run pytest --cov=radar_plotter                # With coverage
uv run pytest --cov=radar_plotter --cov-report=html   # Coverage + HTML report
```

### Linting and Type Checking

```bash
uv run ruff check .    # Linting
uv run ruff format .   # Auto-format
uv run mypy src/       # Type checking
```

### Project Structure

```
src/radar_plotter/
├── core/                    # Core calculation modules
│   ├── coordinates.py       # Polar ↔ Cartesian conversion
│   ├── cpa.py              # Closest Point of Approach
│   ├── maneuvers.py        # Collision avoidance maneuvers
│   ├── relative_motion.py  # SRM/DRM calculations
│   └── true_motion.py      # STM/DTM calculations
├── plotting/               # Visualization
│   ├── animation.py        # Animated radar plots
│   └── radar_plot.py       # Static radar plots
├── models.py               # Data models (RadarProblem, RadarSolution)
├── solver.py               # Main solver orchestration
└── scenarios.py            # Pre-defined test scenarios
```

---

## 📚 Theory

Collision Avoidance Radar Plotting is a maritime navigation technique used to:
- Track vessel movements on radar
- Calculate closest point of approach (CPA)
- Determine required maneuvers to avoid collisions
- Comply with COLREGs (International Regulations for Preventing Collisions at Sea)

### Key Concepts:

- **CPA**: Closest Point of Approach - minimum distance to target
- **SRM**: Speed of Relative Movement
- **DRM**: Direction of Relative Movement
- **STM**: Speed of True Movement
- **DTM**: Direction of True Movement
- **N/C**: New Course required to avoid collision
- **N/S**: New Speed required to avoid collision

---

## 🤝 Contributing

Contributions are welcome! Please:
1. Fork the repository
2. Create a feature branch from `dev`
3. Make your changes
4. Run tests and linting: `uv run pytest && uv run ruff check .`
5. Submit a pull request to `dev`

---

## 🪪 License

This project is licensed under **GNU General Public License v3.0 or later (GPL-3.0-or-later).**

See [LICENSE](LICENSE) for full text.

---

## ⚠️ Disclaimer

**This is an educational tool ONLY.** It should NOT be used for real collision avoidance situations. Always follow proper maritime navigation procedures and COLREGs.

---

## ✍️ Author

**Omar Younis**

---

## 🙏 Acknowledgments

- U.S. Coast Guard Auxiliary for radar plotting training materials
- Northeast Maritime Institute for [instructional videos](https://www.youtube.com/@nmi-edu)
- Developed with modern Python best practices using `uv` and Streamlit
