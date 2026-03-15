"""
@file rocket.py
@brief Physics plant model for a 3-DoF vertical landing vehicle.
"""

import numpy as np

class Rocket:
    """
    Represents the physical parameters and dynamics of the rocket vehicle.
    
    The model assumes a 2D plane (X-Z) where:
    - Z is the vertical altitude.
    - X is the lateral displacement.
    - Theta is the pitch angle (0 rad = perfectly vertical).
    - Delta is the TVC gimbal angle relative to the vehicle centerline.
    """
    def __init__(self,
                 mass = 5000.0,
                 rotational_inertia = 7.5e5,
                 distance_tvc_cm = 3.0,
                 gravity = 9.81,
                 ):
        """
        Initializes the rocket plant with constant physical properties.

        Args:
            mass (float): Vehicle mass (m) [kg].
            rotational_inertia (float): Moment of inertia (J) [kg·m²].
            distance_tvc_cm (float): Distance from Center of Mass to TVC actuator (l) [m].
            gravity (float): Acceleration due to gravity (g) [m/s²].
        """

        # Constants from Section 2.1
        self.m = mass
        self.J = rotational_inertia
        self.l = distance_tvc_cm
        self.g = gravity
        
        self.state = np.zeros(6)
        self.T = 0.0
        self.delta = 0.0

    def set_initial_state(self, *,
            x = 0.0,
            z = 150.0,
            theta = 0.0,
            vx = 0.0,
            vz = -10.0,
            vtheta = 0.0):
        """Sets the initial flight conditions in SI units [m, m/s, rad, rad/s]."""
        self.set_state(np.array([x, z, theta, vx, vz, vtheta], dtype=float))
        
    def set_state(self, new_state):
        """Sets the current state vector."""
        self.state = np.array(new_state, dtype=float)

    def set_thrust_magnitude(self, force):
        """Sets the engine thrust magnitude [N]."""
        self.T = force

    def set_thrust_angle(self, delta):
        """Sets the TVC gimbal angle [rad]."""
        self.delta = delta
    
    def get_derivatives(self):
        """
        Calculates the state derivatives (velocities and accelerations) 
        using nonlinear equations of motion.
            
        Returns:
            np.ndarray: [vx (m/s), vz (m/s), vtheta (rad/s), 
                         ax (m/s²), az (m/s²), alpha (rad/s²)]
        """
        theta = self.state[2]
        vx, vz, vtheta = self.state[3:]

        # Nonlinear Equations of Motion
        ax = (self.T * np.sin(theta + self.delta)) / self.m
        az = (self.T * np.cos(theta + self.delta)) / self.m - self.g
        alpha = (-self.T * np.sin(self.delta) * self.l) / self.J

        return np.array([vx, vz, vtheta, ax, az, alpha])