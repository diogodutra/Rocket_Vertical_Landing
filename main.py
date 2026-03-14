import numpy as np
from src.model.rocket import Rocket
from src.model.integrator import HeunIntegrator
from src.control.controller import FlightController
from src.helper.plots import plot_results

def run_simulation():
    # Setup Environment and Vehicle
    rocket = Rocket()
    integrator = HeunIntegrator(rocket)
    controller = FlightController(rocket)
    
    # Starting at 150m with a 10m/s descent rate
    rocket.set_initial_state(x=5.0, z=150.0, vz=-10.0, theta=0.0)
    
    # Simulation Parameters
    dt = 0.02  # 50Hz control loop
    t_max = 35.0
    t_span = np.arange(0, t_max, dt)
    
    # Target: Stationary at ground level (x=0, z=0)
    # We set vz_ref to a slightly negative value (-0.5) for a soft touchdown
    target_state = np.array([0.0, 0.0, 0.0, 0.0, -0.5, 0.0])
    
    # Data logging
    history = []

    # Main Loop
    for t in t_span:
        curr_state = rocket.state
        
        # Stop if we hit the ground
        if curr_state[1] <= 0:
            print(f"Touchdown at T={t:.2f}s")
            break
            
        # FSW: Compute control laws
        thrust, delta, theta_cmd = controller.update(curr_state, target_state, dt)
        
        # Actuation: Apply to physics model
        rocket.set_thrust_magnitude(thrust)
        rocket.set_thrust_angle(delta)
        
        # Physics: Propagate state
        integrator.propagate(dt)
        
        # Log: Save for plotting
        history.append(np.concatenate(([t], curr_state, [thrust, delta, theta_cmd])))

    # Results Visualization
    print(f"Final state={curr_state}")
    if (abs(curr_state[0]) > 3): print(f"Bad x: 3 < {curr_state[0]}")
    if (abs(curr_state[2]) > np.deg2rad(2)): print(f"Bad theta: 2 < {np.rad2deg(curr_state[2])}")
    if (abs(curr_state[3]) > 0.5): print(f"Bad v_x: 0.5 < {curr_state[3]}")
    if (abs(curr_state[5]) > np.deg2rad(1)): print(f"Bad v_theta: 1 < {np.rad2deg(curr_state[5])}")
    history = np.array(history)
    plot_results(history)


if __name__ == "__main__":
    run_simulation()