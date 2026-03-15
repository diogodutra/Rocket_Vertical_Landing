"""
@file integrator.py
@brief Numerical integration methods for state propagation.
"""

class EulerIntegrator:
    """
    Performs first-order Euler integration for time propagation.
    
    Mathematical form:
    s(t + dt) = s(t) + f(s, u) * dt
    """
    def __init__(self, model):
        """
        Args:
            model: An instance of RocketVehicle or any object 
                   implementing get_derivatives(state, T, delta).
        """
        self.model = model

    def propagate(self, dt):
        """
        Updates the model's state using the Euler method.

        Args:
            dt (float): Integration time step [s].
        """
        derivatives = self.model.get_derivatives()
        
        new_state = self.model.state + derivatives * dt
        
        self.model.set_state(new_state)

class HeunIntegrator:
    """
    Performs second-order Heun's method (Improved Euler) for time propagation.
    
    This is a Predictor-Corrector method that reduces local truncation error
    from O(dt^2) to O(dt^3).
    """
    def __init__(self, model):
        """
        Args:
            model: An instance of Rocket implementing get_derivatives() 
                   and set_state().
        """
        self.model = model

    def propagate(self, dt):
        """
        Updates the model's state using Heun's method.

        Algorithm:
        1. Predict: Calculate intermediate state using current slope.
        2. Correct: Average current slope and predicted slope to update state.

        Args:
            dt (float): Integration time step [s].
        """
        # Save initial state to revert after prediction
        initial_state = self.model.state.copy()

        # Predictor (Standard Euler step)
        k1 = self.model.get_derivatives()
        state_predictor = initial_state + k1 * dt
        
        # Corrector
        self.model.set_state(state_predictor)
        k2 = self.model.get_derivatives()
        
        # Average the slopes and apply to the original state
        new_state = initial_state + (0.5 * (k1 + k2) * dt)
        
        # Update model to the final corrected state
        self.model.set_state(new_state)