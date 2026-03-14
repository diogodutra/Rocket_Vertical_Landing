import numpy as np
from src.model.rocket import Rocket
from src.model.integrator import HeunIntegrator
from src.control.controller import FlightController

class RocketSimulator:
    def __init__(self, *, dt=0.02, t_max=35.0,
                 target_state=[0.0, 0.0, 0.0, 0.0, -0.5, 0.0]):
        self.dt = dt
        self.t_max = t_max
        self.target_state = np.array(target_state)
        
    def run(self, *, x=0.0, z=150.0, vx=0.0, vz=-10.0, theta=0.0, vtheta=0.0):
        """
        Executes a single simulation run and returns the trajectory history.
        """
        # Re-initialize components for a clean run
        rocket = Rocket()
        integrator = HeunIntegrator(rocket)
        controller = FlightController(rocket)
        
        rocket.set_initial_state(
            x=x, z=z, theta=theta, vx=vx, vz=vz, vtheta=vtheta)
        
        history = []
        t_span = np.arange(0, self.t_max, self.dt)

        for t in t_span:
            curr_state = rocket.state
            
            # Ground contact detection
            if curr_state[1] <= 0:
                break
                
            # GNC Update
            thrust, delta, theta_cmd = controller.update(curr_state, self.target_state, self.dt)
            
            # Apply Actuation
            rocket.set_thrust_magnitude(thrust)
            rocket.set_thrust_angle(delta)
            
            # Physics Step
            integrator.propagate(self.dt)
            
            # Log data: [t, x, z, theta, vx, vz, vtheta, thrust, delta, theta_cmd]
            history.append(np.concatenate(([t], curr_state, [thrust, delta, theta_cmd])))

        return np.array(history)

    @staticmethod
    def check_landing_criteria(history):
        """
        Evaluates the final state of a simulation against safety requirements.
        Returns (bool, dict) indicating overall success and specific failures.
        """
        if len(history) == 0:
            return False, {"error": "Empty history"}
            
        final_row = history[-1]
        t_f = final_row[0]
        # state indices shifted by 1 because of t at index 0
        x_f, theta_f, vx_f, vz_f, vtheta_f = final_row[1], final_row[3], final_row[4], final_row[5], final_row[6]

        results = {
            "timeout": t_f < 35.0,
            "precision_x": abs(x_f) < 3.0,
            "soft_landing_vz": abs(vz_f) < 0.5,
            "lateral_drift_vx": abs(vx_f) < 0.5,
            "upright_theta": abs(np.rad2deg(theta_f)) < 2.0,
            "angular_stability": abs(np.rad2deg(vtheta_f)) < 1.0
        }
        
        success = all(results.values())
        return success, results

# Example Usage:
if __name__ == "__main__":
    from src.helper.plots import plot_results
    
    sim = RocketSimulator()
    history = sim.run(initial_x=5.0, initial_theta=np.deg2rad(2))
    
    success, report = sim.check_landing_criteria(history)
    print(f"Landing Success: {success}")
    if not success:
        print(f"Failure Report: {report}")
        
    plot_results(history)