from src.helper.sim import RocketSimulator
from src.helper.plots import plot_flight

def run_simulation():
    sim = RocketSimulator(dt=0.02, t_max=35.0,
            target_state=[0.0, 0.0, 0.0, 0.0, -0.5, 0.0])
        
    history = sim.run()
    
    success, report = sim.check_landing_criteria(history)    
    print(f"Landing Success: {success}")
    if not success:
        print(f"Failure Report: {report}")

    plot_flight(history)

if __name__ == "__main__":
    run_simulation()