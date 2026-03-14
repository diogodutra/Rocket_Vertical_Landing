import pytest
from src.model.sim import RocketSimulator

@pytest.fixture
def simulator():
    return RocketSimulator()

@pytest.mark.parametrize("initial_z", [50.0, 150.0, 200.0])
def test_attitude_recovery_at_various_altitudes(simulator, initial_z):
    """
    Verifies vertical recovery and soft landing across a matrix of 
    starting altitudes. Note: We use a modified run approach to 
    'mute' lateral errors if desired.
    """    
    history = simulator.run(z=initial_z)
    success, report = simulator.check_landing_criteria(history)    
    assert success, f"Landing failed! Failure Report: {report}"