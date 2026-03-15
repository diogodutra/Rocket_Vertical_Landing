"""
@file run_grid.py
@brief Visualization of grid search results for controller robustness verification.
"""

import matplotlib.pyplot as plt
import numpy as np
from src.helper.grid_search import perform_grid_search, generate_oat_grid

def plot_grid_results(results):
    """
    Generates a categorical scatter plot representing the stability margins
    of the GNC system across varying plant and controller parameters.

    Args:
        results (dict): A dictionary where keys are parameter names and values
            are tuples containing:
            - values (np.ndarray): The specific values tested for that parameter.
            - passes (list of bool): Success status for each test case.
            - envelope (tuple): The identified stable range [min, max].
    """
    fig, axs = plt.subplots(1, 5, figsize=(14, 5))
    
    # Iterate through parameters (e.g., Mass [kg], Inertia [kg·m²], Gains, etc.)
    for i, (var_name, (values, passes, envelope)) in enumerate(results.items()):
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
    plt.savefig('docs/grid_search_summary.png')
    plt.show()

if __name__ == "__main__":
    oat_grid = generate_oat_grid()
    data = perform_grid_search(oat_grid)
    plot_grid_results(data)