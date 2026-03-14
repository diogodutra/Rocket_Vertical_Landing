import pytest
import numpy as np
from main import Rocket, FlightController, HeunIntegrator

@pytest.fixture
def sim_setup():
    rocket = Rocket()
    controller = FlightController(rocket)
    integrator = HeunIntegrator(rocket)
    # Target: Origin (0,0) at soft landing speed (-0.5 m/s)
    # state: [x, z, theta, vx, vz, vtheta]
    target_state = np.array([0.0, 0.0, 0.0, 0.0, -0.5, 0.0])
    return rocket, controller, integrator, target_state

def run_full_3dof_sim(rocket, controller, integrator, target_state, max_time=45.0):
    dt = 0.02
    t = 0.0
    while rocket.state[1] > 0 and t < max_time:
        # Full 3DoF Control
        thrust, delta, _ = controller.update(rocket.state, target_state, dt)
        
        rocket.set_thrust_magnitude(thrust)
        rocket.set_thrust_angle(delta) 
        
        integrator.propagate(dt)
        t += dt
    return t, rocket.state

@pytest.mark.parametrize("x0, z0, theta0, vx0, vz0, vtheta0", [
    (5.0, 150.0, 0.0, 0.0, -10.0, 0.0),   # Lateral offset only
    (0.0, 150.0, np.deg2rad(5), 0.0, -10.0, 0.0),   # Tilt offset only
    # (0.0, 150.0, 0.0, 1.0, -10, 0.0),    # Lateral speed only
])
def test_full_system_convergence(sim_setup,
        x0, z0, theta0, vx0, vz0, vtheta0):
    """
    Validates that all controllers work in unison to bring the vehicle 
    to a soft landing at the target lateral coordinate (x=0).
    """
    rocket, controller, integrator, target_state = sim_setup
    
    # Initialize state with offsets
    rocket.set_initial_state(
        x=x0, 
        z=z0, 
        theta=theta0,
        vx=vx0,
        vz=vz0,
        vtheta=vtheta0
    )
    
    total_time, final_state = run_full_3dof_sim(rocket, controller, integrator, target_state)
    
    x_final, z_final, theta_final = final_state[0], final_state[1], final_state[2]
    vx_final, vz_final, vtheta_final = final_state[3], final_state[4], final_state[5]

    # Thresholds for a successful mission
    assert total_time < 35.0, f"Timeout (no landing) in 35 seconds."
    assert abs(x_final) < 3.0, f"Missed target position x: 3<|{x_final:.2f}| m"
    assert abs(vx_final) < 0.5, f"Sliding fast vx: 0.5<|{vz_final:.2f}| m/s"
    assert abs(vz_final) < 0.5, f"Hard landing vz: 0.5<|{vz_final:.2f}| m/s"
    theta_deg = np.deg2rad(theta_final)
    assert abs(theta_deg) < 2, f"Too tilted theta: 2<|{theta_deg}| deg"
    vtheta_deg = np.deg2rad(vtheta_final)
    assert abs(vtheta_deg) < 1, f"Rotating fast vtheta: 1<|{vtheta_deg:.2f}| deg/s"
    
    # 3. Structural Integrity (Vertical within 2 degrees)
    assert abs(np.rad2deg(theta_final)) < 2.0, f"Landed with unsafe tilt: {np.rad2deg(theta_final):.2f}°"