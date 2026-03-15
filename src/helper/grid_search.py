import numpy as np
from src.helper.sim import RocketSimulator

def generate_oat_grid(x_lim=(-30, 30), z_lim=(0, 200), vx_lim=(-9, 9), 
                      vz_lim=(-100, -1), theta_lim=(-20, 20), num_points=11):
    """
    Generates a dictionary of OAT (One-At-a-Time) test values.
    """
    return {
        'x [m]': np.linspace(*x_lim, num_points),
        'z [m]': np.linspace(*z_lim, num_points),
        'vx [m/s]': np.linspace(*vx_lim, num_points),
        'vz [m/s]': np.linspace(*vz_lim, num_points),
        'theta [deg]': np.linspace(*theta_lim, num_points)
    }

def perform_grid_search(grids, nominal_state=None, t_max=35.0):
    """
    Executes simulations across a pre-generated grid object.
    """
    sim = RocketSimulator(t_max=t_max)
    
    if nominal_state is None:
        nominal_state = {
            'x': 0.0, 'z': 150.0, 'theta': 0.0, 
            'vx': 0.0, 'vz': -10.0, 'vtheta': 0.0
        }

    results = {}

    for var_name, values in grids.items():
        var_results = []
        actuation_envelope = []
        
        for val in values:
            p = nominal_state.copy()
            
            # Map input label to simulator keyword
            if 'x [m]' in var_name: p['x'] = val
            elif 'z [m]' in var_name: p['z'] = val
            elif 'vx' in var_name: p['vx'] = val
            elif 'vz' in var_name: p['vz'] = val
            elif 'theta' in var_name: p['theta'] = np.deg2rad(val)
            
            history = sim.run(**p)
            passed, _ = sim.check_landing_criteria(history)
            
            # Extract Actuation data (Requirement Analysis)
            if history.shape[0] > 0:
                thrusts = history[:, 7]
                deltas = np.rad2deg(history[:, 8])
                env = (np.min(thrusts), np.max(thrusts), np.min(deltas), np.max(deltas))
            else:
                env = (0, 0, 0, 0)
                
            var_results.append(passed)
            actuation_envelope.append(env)
            
        results[var_name] = (values, var_results, np.array(actuation_envelope))
    
    return results