import matplotlib.pyplot as plt
import numpy as np

def plot_results(data):
    # Column mapping
    t = data[:, 0]
    x, z, theta = data[:, 1], data[:, 2], data[:, 3]
    vx, vz = data[:, 4], data[:, 5]
    thrust, delta = data[:, 7], data[:, 8]
    theta_cmd = data[:, 9] 

    fig, axs = plt.subplots(4, 1, figsize=(12, 6), sharex=True)
    
    # 1. Trajectory
    axs[0].plot(t, z * 0.1, label='Altitude (z)')
    axs[0].plot(t, x, label='Lateral (x)')
    axs[0].set_ylabel('Position [m]')
    axs[0].legend(loc='upper center', ncol=2)
    axs[0].grid(True)
    
    # 2. Velocities
    axs[1].plot(t, vz, label='Vertical (vz)')
    axs[1].plot(t, vx, label='Lateral (vx)')
    axs[1].set_ylabel('Velocity [m/s]')
    axs[1].legend(loc='upper center', ncol=2)
    axs[1].grid(True)

    # 3. Attitude (Theta) vs Command
    axs[2].plot(t, np.rad2deg(theta), label='Actual (theta)', color='purple', linewidth=2)
    axs[2].plot(t, np.rad2deg(theta_cmd), label='Command (ref)', color='orange', linestyle='--')
    axs[2].axhline(y=0, color='black', linestyle='-', alpha=0.3)
    axs[2].set_ylabel('Attitude [deg]')
    axs[2].legend(loc='upper center', ncol=2)
    axs[2].grid(True)

    # 4. Control Effort
    axs[3].plot(t, thrust/1000, label='Thrust [kN]', color='r')
    axs[3].set_ylabel('Thrust [kN]')
    
    ax3_twin = axs[3].twinx()
    ax3_twin.plot(t, np.rad2deg(delta), label='Gimbal [deg]', color='g', alpha=0.7)
    ax3_twin.set_ylabel('Gimbal [deg]')
    
    axs[3].set_xlabel('Time [s]')
    axs[3].grid(True)
    
    # Combined legend for twin axis
    lines, labels = axs[3].get_legend_handles_labels()
    lines2, labels2 = ax3_twin.get_legend_handles_labels()
    axs[3].legend(lines + lines2, labels + labels2, loc='upper center', ncol=2)
    
    plt.tight_layout()
    plt.show()