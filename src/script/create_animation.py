"""
Module for generating GNC landing visualizations.

This module processes simulation history to produce a dual-pane animation
correlating physical rocket dynamics with real-time controller commands.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.transforms as mtransforms
import matplotlib.colors as mcolors
from matplotlib.animation import FFMpegWriter, FuncAnimation
from matplotlib.patches import Polygon

def get_flame_color(thrust_kn):
    """Calculates the flame color based on thrust intensity.

    Interpolates between Orange, Red, and Purple based on defined kN thresholds
    to provide a visual 'thermal' state of the engine.

    Args:
        thrust_kn: Current engine thrust in kilonewtons.

    Returns:
        A list of three floats representing the RGB color.
    """
    if thrust_kn <= 49:
        return 'orange'
    if thrust_kn >= 61:
        return 'purple'
    
    # New thresholds: Orange (49), Red (55), Purple (61)
    points = [49, 55, 61]
    colors = ['orange', 'red', 'purple']
    rgb_colors = [mcolors.to_rgb(c) for c in colors]
    
    if thrust_kn < 55:
        # Interpolate Orange to Red
        frac = (thrust_kn - 49) / (55 - 49)
        new_rgb = [(1 - frac) * rgb_colors[0][i] + frac * rgb_colors[1][i] for i in range(3)]
    else:
        # Interpolate Red to Purple
        frac = (thrust_kn - 55) / (61 - 55)
        new_rgb = [(1 - frac) * rgb_colors[1][i] + frac * rgb_colors[2][i] for i in range(3)]
        
    return new_rgb

def animate_flight(history, filename="docs/landing_animation.mp4"):
    """Generates a dual-pane MP4 dashboard of the flight simulation.

    Args:
        history: A 2D numpy array where columns represent:
            [0]: Time, [1]: X, [2]: Z, [3]: Theta, [7]: Thrust, [8]: Delta.
        filename: String path for the output video file.
    """
    t = history[:, 0]
    x_data, z_data, theta_data = history[:, 1], history[:, 2], history[:, 3]
    thrust_data, delta_data = history[:, 7], history[:, 8]
    
    fig, (ax_sim, ax_cmd) = plt.subplots(1, 2, figsize=(14, 7))
    
    # Flight Canvas (Left)
    ax_sim.set_xlim(-20, 20) 
    ax_sim.set_ylim(-15, 160)
    ax_sim.set_aspect('equal')
    ax_sim.set_autoscale_on(False)
    ax_sim.grid(True, linestyle='--', alpha=0.3)
    ax_sim.axhline(0, color='black', lw=2, zorder=1)
    ax_sim.plot(0, 0, 'gx', markersize=12, markeredgewidth=3, zorder=2)

    # Command Plot (Right)
    ax_cmd.set_xlim(0, 35)
    ax_cmd.set_ylim(-11, 11) 
    ax_cmd.set_xlabel("Time [s]")
    ax_cmd.set_ylabel("Gimbal Angle δ [deg]", color='tab:red')
    ax_cmd.grid(True, alpha=0.3)
    
    ax_thrust = ax_cmd.twinx()
    ax_thrust.set_ylim(48, 75) 
    ax_thrust.set_ylabel("Thrust [kN]", color='tab:blue')

    # Rocket Dimensions
    w, h = 2.0, 10.0
    half_h, half_w = h/2, w/2
    rocket_coords = np.array([
        [0, half_h], [half_w, half_h - 2], [half_w, -half_h + 1.5],
        [half_w + 1.2, -half_h], [half_w, -half_h], [-half_w, -half_h],
        [-half_w - 1.2, -half_h], [-half_w, -half_h + 1.5], [-half_w, half_h - 2],
    ])

    rocket_patch = Polygon(rocket_coords, facecolor='lightgray', edgecolor='black', zorder=4)
    flame_patch = Polygon([[0,0], [0,0], [0,0]], edgecolor='none', alpha=0.8, zorder=3)
    
    ax_sim.add_patch(rocket_patch)
    ax_sim.add_patch(flame_patch)
    
    trace, = ax_sim.plot([], [], 'b--', alpha=0.3, lw=1)
    line_delta, = ax_cmd.plot([], [], color='tab:red', lw=1.5)
    line_thrust, = ax_thrust.plot([], [], color='tab:blue', lw=1.5)
    
    telemetry = ax_sim.text(0, 0, '', verticalalignment='bottom', horizontalalignment='center',
                            family='monospace', fontsize=9, fontweight='bold',
                            bbox=dict(facecolor='white', alpha=0.7, edgecolor='none', pad=2))

    def update(frame):
        """Update function for Matplotlib FuncAnimation."""
        cx, cz = x_data[frame], z_data[frame] + half_h
        theta_deg = np.rad2deg(theta_data[frame])
        thrust_kn = thrust_data[frame] / 1000.0
        delta_deg = np.rad2deg(delta_data[frame])
        
        # 1. Update Rocket Simulation Plot
        rocket_patch.set_xy(rocket_coords + [cx, cz])
        t_base = ax_sim.transData
        t_rot = mtransforms.Affine2D().rotate_deg_around(cx, cz, theta_deg)
        rocket_patch.set_transform(t_rot + t_base)
        
        # 2. Flame Visuals with new thresholds
        f_len = (thrust_kn / 60.0) * 14.0 * np.random.uniform(0.85, 1.15)
        if thrust_kn > 0.1:
            flame_patch.set_facecolor(get_flame_color(thrust_kn))
            flame_patch.set_xy(np.array([[-0.6, 0], [0.6, 0], [0, -f_len]]) + [cx, cz - half_h])
            flame_patch.set_visible(True)
        else:
            flame_patch.set_visible(False)
        
        t_flame_rot = mtransforms.Affine2D().rotate_deg_around(cx, cz - half_h, theta_deg + delta_deg)
        flame_patch.set_transform(t_flame_rot + t_base)
        
        # 3. Update Visuals
        trace.set_data(x_data[:frame], z_data[:frame] + half_h)
        telemetry.set_position((cx, cz + h * 0.8))
        telemetry.set_text(f'{thrust_kn:>4.1f}kN | δ={delta_deg:>+4.1f}°')
        
        line_delta.set_data(t[:frame], np.rad2deg(delta_data[:frame]))
        line_thrust.set_data(t[:frame], thrust_data[:frame] / 1000.0)
        
        return rocket_patch, flame_patch, trace, telemetry, line_delta, line_thrust

    ani = FuncAnimation(fig, update, frames=range(0, len(t), 2), interval=40, blit=False)

    print(f"Encoding Dashboard to {filename}...")
    writer = FFMpegWriter(fps=30, metadata=dict(artist='Diogo Dutra'), bitrate=3500)
    ani.save(filename, writer=writer)
    plt.close()
    print("Done.")

if __name__ == "__main__":
    from src.script.run_flight import run_simulation
    
    history, success, report = run_simulation(x=4)
    animate_flight(history)