# 2D Motorboat Simulation

This is a 2D top-down motorboat simulation with realistic physics for a 16-foot motorboat with a 3KW motor.

## Features

*   **Realistic Physics:** Simulates a 16-foot motorboat with a 3KW motor.
*   **Simulated Wind:** Wind is simulated using Perlin noise for a more natural effect.
*   **Interactive Controls:**
    *   Throttle and rudder controls on the UI.
    *   A button to toggle between different modes:
        *   **Manual Mode:** Full control over the boat.
        *   **Anchor Mode:** Autopilot to maintain position.
        *   **Wind Vane Mode:** Autopilot to keep the bow of the boat pointing into the wind.
*   **Advanced Wind Estimation:** In Wind Vane mode, a Kalman filter is used to estimate the wind direction for precise control.

## How to Run

The simulation is written entirely in HTML and JavaScript. Simply open the `index.html` file in a web browser to run it.
