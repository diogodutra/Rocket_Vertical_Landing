"""
@file test_altitude.py
@brief Regression tests for the Altitude controller and vertical descent logic.
"""

import pytest
from src.helper.sim import RocketSimulator

@pytest.fixture
def simulator():
    """
    Provides a fresh instance of the RocketSimulator for each test run.
    """
    return RocketSimulator()

@pytest.mark.parametrize("initial_z", [50.0, 150.0, 200.0])
def test_attitude_recovery_at_various_altitudes(simulator, initial_z):
    """
    Verifies the Altitude Control channel across the operational flight ceiling.
    
    This test isolates the vertical descent logic to ensure the controller 
    properly manages throttle and thrust-to-weight ratios to achieve a soft 
    landing (vz ≈ -0.5 m/s) from varying drop heights.

    Constraints:
        - Position control disabled (`disable_pos=True`).
        - Attitude control disabled (`disable_att=True`) to verify 
          purely vertical descent performance without gimbal noise.

    Args:
        initial_z_m (float): Initial vertical altitude in Meters [m].
    """
    history = simulator.run(z=initial_z, disable_pos=True, disable_att=True)
    success, report = simulator.check_landing_criteria(history)    
    assert success, f"Landing failed! Failure Report: {report}"