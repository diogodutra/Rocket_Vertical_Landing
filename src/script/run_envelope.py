"""
@file run_envelope.py
@brief Analysis of actuators demand (Thrust and TVC) across the flight envelope.
"""

import numpy as np
import matplotlib.pyplot as plt
from src.helper.grid_search import perform_grid_search, generate_oat_grid

def plot_grid_results(results):
    """
    Generates a professional telemetry-style analysis of actuation requirements.
    
    This visualization maps the relationship between plant uncertainties 
    (e.g., mass, inertia) and the physical limits of the hardware:
    1.  Top Row: Binary success/failure status (V&V check).
    2.  Bottom Row: Dynamic range of thrust [kN] and TVC [deg] required for stability.

    Args:
        results (dict): Dictionary of parameter study results containing values,
            pass/fail status, and the actuation envelope (min/max demands).
    """
    
    # Maintain the 1:3 ratio for a professional "telem" look
    fig, axs = plt.subplots(2, 5, figsize=(12, 6), 
                            gridspec_kw={'height_ratios': [1, 3]},
                            sharex='col')
    
    for i, (var_name, (values, passes, envelope)) in enumerate(results.items()):
        # Success Status ---
        colors = ['#2ecc71' if p else '#e74c3c' for p in passes]
        axs[0, i].scatter(values, np.ones_like(values), c=colors, s=150, 
                          marker='s', edgecolors='black', linewidth=0.5)
        axs[0, i].set_title(var_name, fontweight='bold', fontsize=10)
        axs[0, i].set_ylim(0.8, 1.2)
        axs[0, i].set_yticks([])
        axs[0, i].spines['top'].set_visible(False)
        axs[0, i].spines['left'].set_visible(False)
        axs[0, i].spines['right'].set_visible(False)
        
        # Actuation Requirements ---
        # Unpack envelope: (min_t, max_t, min_delta, max_delta)
        min_thrusts_kN = envelope[:, 0] / 1000.0
        max_thrusts_kN = envelope[:, 1] / 1000.0
        min_tvc = envelope[:, 2]
        max_tvc = envelope[:, 3]
        
        ax_thrust = axs[1, i]
        ax_tvc = ax_thrust.twinx()
        
        # Plot Thrust Band (Min/Max)
        ax_thrust.fill_between(values, min_thrusts_kN, max_thrusts_kN, color='b', alpha=0.1)
        ax_thrust.plot(values, max_thrusts_kN, 'b-o', markersize=3, alpha=0.8, label='Max Thrust')
        ax_thrust.plot(values, min_thrusts_kN, 'b--', alpha=0.4, label='Min Thrust')
        
        # Plot TVC Band (Min/Max Deflection)
        ax_tvc.fill_between(values, min_tvc, max_tvc, color='r', alpha=0.1)
        ax_tvc.plot(values, max_tvc, 'r-x', markersize=3, alpha=0.8)
        ax_tvc.plot(values, min_tvc, 'r--', alpha=0.4)
        
        # Formatting
        if i == 0: ax_thrust.set_ylabel('Thrust (kN)', color='b', fontweight='bold')
        if i == 4: ax_tvc.set_ylabel('TVC (deg)', color='r', fontweight='bold')
        
        ax_thrust.grid(True, alpha=0.3)
        
        # Zero line for TVC to show symmetry
        ax_tvc.axhline(0, color='black', linewidth=0.5, alpha=0.5)

    plt.suptitle("GNC Actuation Envelope Analysis (Full Min/Max Range)", fontsize=14, y=0.98)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig('docs/actuation_requirements.png', dpi=300)
    plt.show()

if __name__ == "__main__":    
    oat_grid = generate_oat_grid()

    data = perform_grid_search(oat_grid)
    plot_grid_results(data)