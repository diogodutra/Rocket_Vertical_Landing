import pytest
import numpy as np
from main import Rocket, FlightController, HeunIntegrator

@pytest.fixture
def sim_setup():
    rocket = Rocket()
    controller = FlightController(rocket)
    integrator = HeunIntegrator(rocket)
    # Target vertical velocity of -0.5 m/s
    target_state = np.array([0.0, 0.0, 0.0, 0.0, -0.5, 0.0])
    return rocket, controller, integrator, target_state

def run_stabilized_landing_sim(rocket, controller, integrator, target_state, max_time=40.0):
    """
    Runs simulation while ignoring lateral position error 
    to focus strictly on attitude recovery and vertical descent.
    """
    dt = 0.02
    t = 0.0
    while rocket.state[1] > 0 and t < max_time:
        # Mute the position controller: tell it the target x is wherever it currently is.
        # This prevents the outer loop from commanding tilts to 'travel' to x=0.
        target_state[0] = rocket.state[0] 
        
        thrust, delta, _ = controller.update(rocket.state, target_state, dt)
        
        rocket.set_thrust_magnitude(thrust)
        rocket.set_thrust_angle(delta)
        
        integrator.propagate(dt)
        t += dt
    return t, rocket.state

# Test a range of initial tilts: 0 rad, 0.1 rad (~5.7°), and -0.2 rad (~-11.4°)
@pytest.mark.parametrize("initial_theta", [0.0, 0.1, -0.2])
def test_attitude_stabilization_on_landing(sim_setup, initial_theta):
    """
    Verifies that the rocket recovers to a vertical orientation 
    and lands softly regardless of initial tip angle.
    """
    rocket, controller, integrator, target_state = sim_setup
    
    # Initial state: 150m up, falling at 10m/s, with varying initial tilt
    rocket.set_initial_state(x=0.0, z=150.0, theta=initial_theta, vz=-10.0)
    
    total_time, final_state = run_stabilized_landing_sim(rocket, controller, integrator, target_state)
    
    final_vz = final_state[4]
    final_theta_deg = np.rad2deg(final_state[2])

    # Assertions for a successful "Stable Recovery"
    assert total_time < 40.0, f"Simulation timed out for theta={initial_theta}"
    
    # 1. Soft Landing Check
    assert final_vz > -1.0, f"Hard landing with theta={initial_theta}: {final_vz:.2f} m/s"
    
    # 2. Verticality Check (Tolerance of 1.0 degree)
    assert abs(final_theta_deg) < 1.0, (
        f"Failed to verticalize! Started at {np.rad2deg(initial_theta):.1f}°, "
        f"landed at {final_theta_deg:.2f}°"
    )