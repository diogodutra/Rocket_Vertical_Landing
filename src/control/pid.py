class PID:
    def __init__(self, kp, ki, kd, output_limits=(None, None)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        
        self.integral = 0.0
        self.prev_error = 0.0
        
    def compute(self, error, dt):
        if dt <= 0: return 0.0
        
        # Proportional
        p_term = self.kp * error
        
        # Integral
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Derivative
        d_term = self.kd * (error - self.prev_error) / dt
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
        """Resets the integral and derivative terms to zero."""
        self.integral = 0.0
        self.prev_error = 0.0