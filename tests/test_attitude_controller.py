import numpy as np
import pytest
from src.helper.sim import RocketSimulator

@pytest.fixture
def simulator():
    return RocketSimulator()

@pytest.mark.parametrize("initial_theta", [0.1, 0.2, 0.3, 0.4, 0.5,])
def test_attitude_stabilization_on_landing(simulator, initial_theta):
    """
    Verifies that the rocket recovers to a vertical orientation 
    and lands softly regardless of initial tip angle, ignoring 
    lateral translation errors.
    """    
    history = simulator.run(theta=np.deg2rad(initial_theta), disable_pos=True)
    success, report = simulator.check_landing_criteria(history)    
    assert success, f"Landing failed! Failure Report: {report}"