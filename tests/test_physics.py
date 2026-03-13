import pytest
import numpy as np
import numpy.testing as npt
from src.model.rocket import Rocket
from src.model.integrator import HeunIntegrator as Integrator

def test_gravity_freefall():
    """Verify that with zero thrust, the rocket falls at 1g."""
    rocket = Rocket()
    expected_state = rocket.state
    
    rocket.set_thrust_magnitude(0.0)
    rocket.set_thrust_angle(0.0)
    rocket.set_initial_state(z=0.0, vz=0.0)
    
    integrator = Integrator(rocket)
    dt = 1.0
    integrator.propagate(dt)

    expected_state[1] = - 0.5 * rocket.g * dt * dt # expected z
    expected_state[4] = - rocket.g * dt # expected vz
    npt.assert_allclose(rocket.state, expected_state, atol=1e-19, rtol=0)

def test_static_hover():
    """Verify that Thrust = mg results in zero vertical acceleration."""
    rocket = Rocket()
    expected_state = rocket.state

    hover_thrust = rocket.m * rocket.g    
    rocket.set_thrust_magnitude(hover_thrust)
    rocket.set_thrust_angle(0.0)
    
    integrator = Integrator(rocket)
    dt = 0.1
    integrator.propagate(dt)
    
    expected_state[4] = 0.0 # expected vz
    
    npt.assert_allclose(rocket.state, expected_state, atol=1e-19, rtol=0)

def test_rotational_torque():
    """Verify that a positive delta creates a negative angular acceleration."""
    rocket = Rocket()
    
    # Apply thrust and a 2-degrees gimbal angle
    thrust = 50000.0
    delta = np.deg2rad(2.0)
    rocket.set_thrust_magnitude(thrust)
    rocket.set_thrust_angle(delta)
    
    integrator = Integrator(rocket)
    dt = 0.1
    integrator.propagate(dt)
    
    # Expected angular acceleration alpha = (-T * sin(delta) * l) / J
    expected_alpha = (-thrust * np.sin(delta) * rocket.l) / rocket.J
    # New angular rate vtheta = vtheta_old + alpha * dt
    expected_vtheta = expected_alpha * dt
    
    assert pytest.approx(rocket.state[5], abs=1e-19) == expected_vtheta

def test_lateral_acceleration():
    """
    Verify that gimbaling the engine produces the correct lateral 
    force and acceleration in the X direction.
    """
    rocket = Rocket()

    # High thrust with a 5-degree gimbal angle to the right (positive delta)
    thrust = 100000.0
    delta = np.deg2rad(5.0)
    dt = 0.1
    
    rocket.set_thrust_magnitude(thrust)
    rocket.set_thrust_angle(delta)
    rocket.set_initial_state(x=0.0, vx=0.0, theta=0.0)
    
    integrator = Integrator(rocket)
    integrator.propagate(dt)
    
    expected_ax = (thrust * np.sin(delta)) / rocket.m # since theta=0, ax = (T * sin(delta)) / m
    expected_vx = expected_ax * dt
    assert pytest.approx(rocket.state[3], rel=1e-19) == expected_vx

    expected_x = 0.5 * expected_ax * dt * dt
    assert pytest.approx(rocket.state[0], rel=1e-19) == expected_x