"""
@file run_flight.py
@brief Entry point for executing GNC simulations and mission success evaluation.
"""

from src.helper.sim import RocketSimulator
from src.helper.plots import plot_flight

def run_simulation(**kwargs):
    """
    Configures and executes a specific flight scenario to validate the GNC logic.
    
    This function initializes the simulator with a target touchdown velocity
    of -0.5 m/s (SI Units) and evaluates the resulting trajectory against 
    safety constraints (tilt, drift, and impact velocity).
    """
    sim = RocketSimulator(dt=0.02, t_max=35.0,
            target_state=[0.0, 0.0, 0.0, 0.0, -0.5, 0.0])

    history = sim.run(**kwargs)
        
    success, report = sim.check_landing_criteria(history) 

    return history, success, report

if __name__ == "__main__":
    history, success, report = run_simulation()
       
    print(f"Landing Success: {success}")
    if not success:
        print(f"Failure Report: {report}")

    plot_flight(history)