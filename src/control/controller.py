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
                           output_limits=(-0.35, 0.35)) # ~20 deg tilt limit
        
        # Attitude Loop (Inner)
        # Target: theta -> commands gimbal delta
        att = gains['attitude']
        self.att_pid = PID(kp=att['kp'], ki=att['ki'], kd=att['kd'], 
                           output_limits=(-0.17, 0.17)) # ~10 deg gimbal limit

    def update(self, state, target_state, dt):
        """
        state: [x, z, theta, vx, vz, vtheta]
        target_state: [x_ref, z_ref, theta_ref, vx_ref, vz_ref, vtheta_ref]
        """
        x, z, theta, vx, vz, vtheta = state
        x_ref, z_ref, _, _, vz_ref, _ = target_state

        # --- Horizontal / Rotational Axis (Nested Loop Closure) ---
        # Outer Loop: Calculate required tilt (theta) to fix lateral position
        x_error = (x_ref - x)
        vx_error = (0 - vx) * 4 # Assuming we want 0 lateral speed at the target
        theta_cmd = self.pos_pid.compute(x_error + vx_error, dt)        
        theta_cmd /= 15.0

        # landing smoothing
        z_landing = 1.0
        if z < z_landing: theta_cmd *= (z - 0.5 * 0) / z_landing
        
        # Inner Loop: Calculate required gimbal (delta) to achieve that tilt
        att_error = theta_cmd - theta
        delta_cmd = - self.att_pid.compute(att_error, dt)

        # --- Vertical Axis ---
        alt_error = z_ref - z
        vel_error = (vz_ref - vz) * 0.1  # Difference between target speed and current speed
        # Feedforward: PID correction + Gravity cancellation (feedforward)
        thrust_cmd = self.alt_pid.compute(alt_error + vel_error, dt) + (self.m * self.g)

        return thrust_cmd, delta_cmd, theta_cmd
    

if __name__ == "__main__":    
    rocket = Rocket()
    dt = 0.01
    target_state = [0.0, 0.0, 0.0, 0.0, -0.5, 0.0] # Target landing at -0.5 m/s

    # Define a list of scenarios to verify different controller behaviors
    # Format: (Scenario Name, [x, z, theta, vx, vz, vtheta])
    scenarios = [
        ("Nominal",                 [ 0.0, 150.0, 0.0 , 0.0, -10.0, 0.00]),
        ("High Altitude Descent",   [ 3.0, 100.0, 0.01, 0.1, -9.0, 0.01]),
        ("Lateral Correction",      [-5.0,  50.0, 0.0 , 0.2, -5.0, 0.0]),
        ("Near Touchdown",          [ 0.5,   0.8, 0.01, 0.1, -1.0, 0.0]),
        ("Stationary/Hover",        [ 0.0,  20.0, 0.0 , 0.0, 0.0, 0.0]),
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