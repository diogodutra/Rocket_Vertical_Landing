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
        # if abs(x_error) < 10: x_error *= abs(x_error) / 10
        vx_error = (0 - vx) * 4 # Assuming we want 0 lateral speed at the target
        theta_cmd = self.pos_pid.compute(x_error + vx_error, dt)        
        theta_cmd /= 10.0

        # landing smoothing
        z_landing = 1.0
        if z < z_landing: theta_cmd *= (z + 0.5 * 0) / z_landing
        
        # Inner Loop: Calculate required gimbal (delta) to achieve that tilt
        att_error = theta_cmd - theta
        delta_cmd = - self.att_pid.compute(att_error, dt)

        # --- Vertical Axis ---
        alt_error = z_ref - z
        vel_error = (vz_ref - vz) * 0.1  # Difference between target speed and current speed
        # Feedforward: PID correction + Gravity cancellation (feedforward)
        thrust_cmd = self.alt_pid.compute(alt_error + vel_error, dt) + (self.m * self.g)

        return thrust_cmd, delta_cmd, theta_cmd