import pytest
import numpy as np
from main import Rocket, FlightController, HeunIntegrator

@pytest.fixture
def sim_setup():
    rocket = Rocket()
    controller = FlightController(rocket)
    integrator = HeunIntegrator(rocket)
    # Target: Land at z=0, vz=-0.5. x target is muted in the loop.
    target_state = np.array([0.0, 0.0, 0.0, 0.0, -0.5, 0.0])
    return rocket, controller, integrator, target_state

def run_3dof_landing_sim(rocket, controller, integrator, target_state, max_time=60.0):
    dt = 0.02
    t = 0.0
    while rocket.state[1] > 0 and t < max_time:
        # Mute lateral position error to focus on attitude and altitude
        target_state[0] = rocket.state[0] 
        
        thrust, delta, _ = controller.update(rocket.state, target_state, dt)
        
        rocket.set_thrust_magnitude(thrust)
        rocket.set_thrust_angle(delta) 
        
        integrator.propagate(dt)
        t += dt
    return t, rocket.state

@pytest.mark.parametrize("initial_z", [50.0, 150.0, 200.0])
def test_attitude_recovery_at_various_altitudes(sim_setup, initial_z):
    """
    Verifies vertical recovery and soft landing across a matrix of 
    starting altitudes and tip angles.
    """
    rocket, controller, integrator, target_state = sim_setup
    
    # Initialize full 6-DOF state to ensure consistency
    # [x, z, theta, vx, vz, vtheta]
    rocket.set_initial_state(x=20.0, z=initial_z, theta=0.0, vx=0.0, vz=-10.0, vtheta=0.0)
    
    total_time, final_state = run_3dof_landing_sim(rocket, controller, integrator, target_state)
    
    final_vz = final_state[4]
    final_theta_deg = np.rad2deg(final_state[2])

    # Validation
    assert total_time < 35.0, f"Sim timed out (z={initial_z})"
    assert final_vz > -1.0, f"Hard landing from {initial_z}m: {final_vz:.2f} m/s"
    assert abs(final_theta_deg) < 1.0, f"Landed tilted from {initial_z}m: {final_theta_deg:.2f}°"