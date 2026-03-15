from .pid import PID
from .compute_gains import compute_theoretical_gains
from src.model.rocket import Rocket

def clamp(n, smallest, largest):
    """Utility to constrain a value within a defined range."""
    return max(smallest, min(n, largest))

class FlightController:
    """
    Manages 3-DoF rocket flight dynamics using nested PID control loops.
    
    This controller handles altitude, lateral position, and attitude (tilt),
    incorporating a virtual gate for terminal descent stability.
    """
    def __init__(self, rocket: Rocket):
        """
        Initializes the flight controller with gains derived from rocket parameters.

        Args:
            rocket (Rocket): Instance providing mass (m), inertia (J), 
                length (l), and gravity (g).
        """
        self.m = rocket.m
        self.g = rocket.g
        
        # Compute gains using default frequencies and damping ratios
        # You can also pass these as arguments to __init__ for easier tuning
        gains = compute_theoretical_gains(rocket)
        
        # Altitude Loop (Independent)
        # Target: Vertical position z
        alt = gains['altitude']
        self.alt_pid = PID(kp=alt['kp'], ki=alt['ki'], kd=alt['kd'], 
                           output_limits=(0, 100000)) # Newton limits
        
        # Position Loop (Outer)
        # Target: x -> commands theta_cmd
        pos = gains['position']
        self.pos_pid = PID(kp=pos['kp'], ki=pos['ki'], kd=pos['kd'],
                           output_limits=(-0.01, 0.01))
        
        # Attitude Loop (Inner)
        # Target: theta -> commands gimbal delta
        att = gains['attitude']
        self.att_pid = PID(kp=att['kp'], ki=att['ki'], kd=att['kd'], 
                           output_limits=(-0.17, 0.17)) # ~10 deg gimbal limit

    def update(self, state, target_state, dt,
               disable_att=False, disable_pos=False, disable_alt=False):
        """
        Performs a single control step to calculate actuator commands.

        Args:
            state (list): Current state [x, z, theta, vx, vz, vtheta].
            target_state (list): Target state [x_ref, z_ref, theta_ref, vx_ref, vz_ref, vtheta_ref].
            dt (float): Time step since last update [s].
            disable_att (bool): If True, bypasses attitude control.
            disable_pos (bool): If True, bypasses lateral position control.
            disable_alt (bool): If True, bypasses altitude control.

        Returns:
            tuple: (thrust_cmd, delta_cmd, theta_cmd) representing 
                thrust magnitude [N], TVC angle [rad], and targeted tilt [rad].
        """
        x, z, theta, vx, vz, vtheta = state
        x_ref, z_ref, _, _, vz_ref, _ = self.virtual_gate(state, target_state)

        # Horizontal / Rotational Axis (Nested Loop Closure)
        # Outer Loop: Calculate required tilt (theta) to fix lateral position
        theta_cmd = 0
        if not disable_pos:
            x_error = (x_ref - x)
            theta_cmd = self.pos_pid.compute(x_error, dt)


        # Inner Loop: Calculate required gimbal (delta) to achieve that tilt
        delta_cmd = 0
        if not disable_att:
            att_error = theta_cmd - theta
            delta_cmd = self.att_pid.compute(att_error, dt)
            
        thrust_cmd = self.m * self.g # Gravity cancellation (feedforward)
        if not disable_alt:
            alt_error = z_ref - z
            thrust_cmd += self.alt_pid.compute(alt_error, dt)

        return thrust_cmd, delta_cmd, theta_cmd
    

    def virtual_gate(self, state, target_state):
        """
        Creates a virtual moving checkpoint to provide smoother touchdown.

        Args:
            state (list): Current vehicle state.
            target_state (list): Original reference state.

        Returns:
            list: Adjusted target state with offset Z reference.
        """
        adj_target_state = target_state.copy()
        z = state[1]
        gate_offset = 0.5
        gate_threshold = 2
        if z > gate_threshold:
            adj_target_state[1] += gate_offset
        else:
            # Linear ramp to zero to soften final contact
            adj_target_state[1] += gate_offset * z / gate_threshold

        return adj_target_state
    

if __name__ == "__main__":    
    rocket = Rocket()
    dt = 0.01
    target_state = [0.0, 0.0, 0.0, 0.0, -0.5, 0.0] # Target landing at -0.5 m/s

    # Define a list of scenarios to verify different controller behaviors
    # Format: (Scenario Name, [x, z, theta, vx, vz, vtheta])
    scenarios = [
        ("Nominal",                 [ 0.0, 150.0, 0.0 , 0.0, -10.0, 0.00]),
        ("High Altitude Descent",   [ 2.0, 100.0, 0.01, 1.0, -5.0 , 0.01]),
    ]

    print("=" * 60)
    print(f"{'SCENARIO':<25} | {'THRUST (N)':<12} | {'TVC (rad)':<10}")
    print("-" * 60)

    for name, state in scenarios:
        controller = FlightController(rocket)
        thrust, delta, theta_c = controller.update(state, target_state, dt)
        
        # Print a clean table for easy copy-pasting into C++ tests
        print(f"{name:<25} | {thrust:12.4f} | {delta:10.4f}")
        
        # Detailed view for individual verification if needed
        # print(f"  Inputs: {state}")
        # print(f"  Target Tilt: {theta_c:10.4f} rad")

    print("=" * 60)
    print("Use these values to verify C++ GNCController::update() consistency.")