import numpy as np

class Rocket:
    def __init__(self,
                 mass = 5000.0,
                 rotational_inertia = 7.5e5,
                 distance_tvc_cm = 3.0,
                 gravity = 9.81,
                 ):
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
        self.set_state(np.array([x, z, theta, vx, vz, vtheta], dtype=float))
        
    def set_state(self, new_state):
        self.state = np.array(new_state, dtype=float)

    def set_thrust_magnitude(self, force):
        self.T = force

    def set_thrust_angle(self, delta):
        self.delta = delta
    
    def get_derivatives(self):
        """
        Calculates the state derivatives (velocities and accelerations).
            
        Returns:
            np.array: [vx, vz, vtheta, ax, az, alpha]
        """
        theta = self.state[2]
        vx, vz, vtheta = self.state[3:]

        # Nonlinear Equations of Motion
        ax = (self.T * np.sin(theta + self.delta)) / self.m
        az = (self.T * np.cos(theta + self.delta)) / self.m - self.g
        alpha = (-self.T * np.sin(self.delta) * self.l) / self.J

        return np.array([vx, vz, vtheta, ax, az, alpha])