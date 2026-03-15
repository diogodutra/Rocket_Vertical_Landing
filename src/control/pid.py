"""
@file pid.py
@brief Standard PID control implementation with saturation and anti-windup.
"""

class PID:
    """
    A Proportional-Integral-Derivative (PID) controller with output clamping.
    
    This implementation includes a conditional integration anti-windup mechanism
    to prevent integral overshoot when the actuator reaches its physical limits.
    """
    def __init__(self, kp, ki, kd, output_limits=(None, None)):
        """
        Initializes the PID controller with gains and saturation limits.

        Args:
            kp (float): Proportional gain.
            ki (float): Integral gain.
            kd (float): Derivative gain.
            output_limits (tuple): A tuple of (min_output, max_output). 
                Use None for no limit.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        
        self.integral = 0.0
        self.prev_error = None
        
    def compute(self, error, dt):
        """
        Calculates the control output based on current error and time increment.

        Args:
            error (float): The difference between reference and process variable.
            dt (float): Time increment since last calculation [s].

        Returns:
            float: The computed control command, clamped to output_limits.
        """
        if dt <= 0: return 0.0
        
        # Proportional
        p_term = self.kp * error
        
        # Integral
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Derivative
        if self.prev_error:
            d_term = self.kd * (error - self.prev_error) / dt
        else:
            d_term = 0

        self.prev_error = error
        
        output = p_term + i_term + d_term
        
        # Apply Output Limits
        min_out, max_out = self.output_limits
        if max_out is not None and output > max_out:
            output = max_out
            # Simple anti-windup: stop integrating if saturated
            self.integral -= error * dt 
        elif min_out is not None and output < min_out:
            output = min_out
            self.integral -= error * dt
            
        return output
    
    def reset(self):
        """
        Resets the internal controller state.
        
        Clears the accumulated integral and resets the error history to 
        prevent derivative spikes on the next compute() call.
        """
        self.integral = 0.0
        self.prev_error = 0.0