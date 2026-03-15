from .pid import PID
from .compute_gains import compute_theoretical_gains
from src.model.rocket import Rocket

def clamp(n, smallest, largest): return max(smallest, min(n, largest))

class FlightController:
    def __init__(self, rocket: Rocket):
        """
        Args:
            rocket: An instance of the Rocket class providing m, J, l, g.
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
                           output_limits=(-0.02, 0.02)) # ~1 deg tilt limit
        
        # Attitude Loop (Inner)
        # Target: theta -> commands gimbal delta
        att = gains['attitude']
        self.att_pid = PID(kp=att['kp'], ki=att['ki'], kd=att['kd'], 
                           output_limits=(-0.17, 0.17)) # ~10 deg gimbal limit

    def update(self, state, target_state, dt,
               disable_att=False, disable_pos=False, disable_alt=False):
        """
        state: [x, z, theta, vx, vz, vtheta]
        target_state: [x_ref, z_ref, theta_ref, vx_ref, vz_ref, vtheta_ref]
        """
        x, z, theta, vx, vz, vtheta = state
        x_ref, z_ref, _, _, vz_ref, _ = target_state

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