import numpy as np
import matplotlib.pyplot as plt
from src.model.sim import RocketSimulator

def perform_grid_search():
    sim = RocketSimulator(t_max=35.0)
    
    nom = {'x': 0.0, 'z': 150.0, 'theta': 0.0, 'vx': 0.0, 'vz': -10.0, 'vtheta': 0.0}
    
    grids = {
        'x [m]': np.linspace(-25, 25, 11),
        'z [m]': np.linspace(0, 200, 11),
        'vx [m/s]': np.linspace(-5, 5, 11),
        'vz [m/s]': np.linspace(-100, -1, 11),
        'theta [deg]': np.linspace(-8, 8, 11)
    }

    results = {}

    for var_name, values in grids.items():
        var_results = []
        for val in values:
            p = nom.copy()            
            if 'x [m]' in var_name: p['x'] = val
            elif 'z [m]' in var_name: p['z'] = val
            elif 'vx' in var_name: p['vx'] = val
            elif 'vz' in var_name: p['vz'] = val
            elif 'theta' in var_name: p['theta'] = np.deg2rad(val)
            
            history = sim.run(**p)
            
            passed, _ = sim.check_landing_criteria(history)
            var_results.append(passed)
            
        results[var_name] = (values, var_results)
    
    return results

def plot_grid_results(results):
    fig, axs = plt.subplots(1, 5, figsize=(22, 5))
    
    for i, (var_name, (values, passes)) in enumerate(results.items()):
        # Green for pass, Red for fail
        colors = ['#2ecc71' if p else '#e74c3c' for p in passes]
        y_pos = np.ones_like(values)
        
        axs[i].scatter(values, y_pos, c=colors, s=300, marker='s', edgecolors='black', linewidths=0.5)
        axs[i].set_title(var_name, fontweight='bold')
        axs[i].set_ylim(0.8, 1.2)
        axs[i].set_yticks([])
        axs[i].grid(True, axis='x', linestyle='--', alpha=0.6)
        
    plt.suptitle("Controller Sensitivity Grid Search (Refactored)", fontsize=16, y=1.05)
    plt.tight_layout()
    plt.savefig('grid_search_summary.png')
    print("Grid search complete. Plot saved to grid_search_summary.png")
    plt.show()

if __name__ == "__main__":
    data = perform_grid_search()
    plot_grid_results(data)