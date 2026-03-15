"""
@file test_all_controllers.py
@brief Full-system integration tests for GNC convergence and mission success.
"""

import pytest
import numpy as np
from src.helper.sim import RocketSimulator

@pytest.fixture
def simulator():
    return RocketSimulator()

@pytest.mark.parametrize("x0, z0, theta0, vx0, vz0, vtheta0", [
    (0.0, 150.0, 0.0, 0.0, -10.0, 0.0),             # Nominal scenario
    (0.75, 150.0, 0.0, 0.0, -10.0, 0.0),            # Lateral offset
    (0.0, 150.0, np.deg2rad(5), 0.0, -10.0, 0.0),   # Initial tilt
    (0.0, 150.0, 0.0, 0.4, -10.0, 0.0),             # Lateral velocity
])
def test_full_system_convergence(simulator, x0, z0, theta0, vx0, vz0, vtheta0):
    """
    Validates total system convergence across multiple degrees of freedom (DoF).
    
    This test verifies that the GNC system can simultaneously:
    1. Nullify lateral position and velocity errors (X-channel).
    2. Maintain upright orientation (Theta-channel).
    3. Execute a soft touchdown (Z-channel).

    All inputs and internal checks utilize International System of Units (SI).

    Args:
        x0 (float): Initial lateral position [m].
        z0 (float): Initial vertical altitude [m].
        theta0 (float): Initial pitch angle [rad].
        vx0 (float): Initial lateral velocity [m/s].
        vz0 (float): Initial vertical velocity [m/s].
        vtheta0 (float): Initial angular velocity [rad/s].
    """
    history = simulator.run(
        x=x0,
        z=z0,
        theta=theta0,
        vx=vx0,
        vz=vz0,
        vtheta=vtheta0
    )
    success, report = simulator.check_landing_criteria(history)
    
    assert success, f"Landing failed! Failure Report: {report}"