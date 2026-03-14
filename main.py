from src.model.sim import RocketSimulator
from src.helper.plots import plot_flight
import numpy as np

def run_simulation():
    # Setup Environment and Vehicle
    sim = RocketSimulator(dt=0.02, t_max=35.0,
                 target_state=[0.0, 0.0, 0.0, 0.0, -0.5, 0.0])
    
    # Starting at 150m with a 10m/s descent rate
    # history = sim.run(x=5.0, z=150.0, vz=-10.0, theta=0.0)
    history = sim.run(x=3.0, z=150.0, vz=-10.0, theta=np.deg2rad(1.0))
    
    success, report = sim.check_landing_criteria(history)    
    print(f"Landing Success: {success}")
    if not success:
        print(f"Failure Report: {report}")

    plot_flight(history)

if __name__ == "__main__":
    run_simulation()