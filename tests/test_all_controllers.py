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
    Validates that the GNC system brings the vehicle to a safe landing
    using the RocketSimulator orchestration.
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