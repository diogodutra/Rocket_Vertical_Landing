"""
@file test_attitude_controller.py
@brief Automated regression tests for GNC Altitude controller.
"""

import numpy as np
import pytest
from src.helper.sim import RocketSimulator

@pytest.fixture
def simulator():
    """
    Provides a clean instance of the RocketSimulator for each test case.
    """
    return RocketSimulator()

@pytest.mark.parametrize("initial_theta", [0.1, 0.2, 0.3, 0.4, 0.5,])
def test_attitude_stabilization_on_landing(simulator, initial_theta):
    """
    Verifies the Attitude Control Loop robustness.
    
    This test ensures the rocket can recover to a vertical orientation 
    (theta ≈ 0 rad) and achieve a soft touchdown (vz ≈ -0.5 m/s) even when 
    starting with an initial tilt. 

    Constraints:
        - Position control is disabled (`disable_pos=True`) to isolate 
          attitude stability from lateral drift correction.
    
    Args:
        initial_theta_deg (float): Initial pitch offset in Degrees [deg].
    """
    history = simulator.run(theta=np.deg2rad(initial_theta), disable_pos=True)
    success, report = simulator.check_landing_criteria(history)    
    assert success, f"Landing failed! Failure Report: {report}"