This document presents answers to the assignment questions.

# Session 2 - Problem formulation and modelling

## Question 1
Formulate the equations of motion for this system and implement their time-based
propagation.

### Coordinate Systems

The inertial frame is fixed at Earth surface, which is assumed to be non-accelerated and flat, with global Z axis pointing upwards and global X axis pointing to the right. 

The body frame has origin at the rocket Center of Mass (CM), with local Z' pointed upwards along the rocket frame axis and X' pointed to the rocket's right side. The rocket states are all referenced in the global frame.

### Equations
To derive the accelerations, we analyze the vector sum of forces in the inertial frame and the moments about the Center of Mass, considering a rigid body with constant linear and rotational inertias.

We employ Newton's 2nd law for each axis, considering that the only external forces are gravity and thrust (no aerodynamics) without misalignment between TVC and rocket local frame:

$$\ddot{z} = \frac{T}{m} \cos(\theta + \delta) - g$$
$$\ddot{x} = \frac{T}{m} \sin(\theta + \delta)$$

We can also employ Euler equation:
$$\ddot{\theta} = \frac{-T \cdot l}{J} \sin(\delta)$$

From the equations above, and using Euler integration, we derive the non-linear equations of motion and their time-based propagation in matrix form:
$$\frac{d}{dt} \begin{bmatrix} x \\ z \\ \theta \\ \dot{x} \\ \dot{z} \\ \dot{\theta} \end{bmatrix} = \begin{bmatrix} \dot{x} \\ \dot{z} \\ \dot{\theta} \\ \frac{T}{m} \sin(\theta + \delta) \\ \frac{T}{m} \cos(\theta + \delta) - g \\ -\frac{T \cdot l}{J} \sin(\delta) \end{bmatrix}$$

## Question 2
To verify the fidelity of the 3-DOF rocket model and the integration scheme, the simulation was subjected to three distinct test cases. These tests ensure that:
- the translational and rotational dynamics correctly propagate rocket's states;
- the translational dynamics are uncoupled;
- the rotational dynamics is correctly coupled with translational; and
- that the integrator does not introduces considerable numerical error.

Such tests are available for run:
```
pytest tests/test_physics.py
```