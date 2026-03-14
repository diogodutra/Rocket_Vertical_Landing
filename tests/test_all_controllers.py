import pytest
import numpy as np
from src.model.sim import RocketSimulator

@pytest.fixture
def simulator():
    return RocketSimulator()

@pytest.mark.parametrize("x0, z0, theta0, vx0, vz0, vtheta0", [
    (5.0, 150.0, 0.0, 0.0, -10.0, 0.0),             # Lateral offset
    (0.0, 150.0, np.deg2rad(5), 0.0, -10.0, 0.0),   # Initial tilt
    (0.0, 150.0, 0.0, 1.0, -10.0, 0.0),             # Lateral velocity
    (5.0, 150.0, np.deg2rad(-3), -0.5, -11.0, 0.0), # Combined dispersion
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