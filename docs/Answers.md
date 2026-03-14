This document presents answers to the assignment questions.

# Session 2 - Problem formulation and modelling

## Question 1 - Formulate the equations of motion for this system and implement their time-based propagation.

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

### State propagation
From the equations above, and using Euler integration, we derive the non-linear equations of motion and their time-based propagation in matrix form:
$$\frac{d}{dt} \begin{bmatrix} x \\ z \\ \theta \\ \dot{x} \\ \dot{z} \\ \dot{\theta} \end{bmatrix} = \begin{bmatrix} \dot{x} \\ \dot{z} \\ \dot{\theta} \\ \frac{T}{m} \sin(\theta + \delta) \\ \frac{T}{m} \cos(\theta + \delta) - g \\ -\frac{T \cdot l}{J} \sin(\delta) \end{bmatrix}$$

## Question 2 - Demonstrate your modelling works as expected on an example case.

To verify the fidelity of the 3-DOF rocket model and the integration scheme, the simulation is subjected to four distinct unit test cases. These tests ensure that:
- the translational and rotational dynamics correctly propagate the rocket's states;
- the translational dynamics are uncoupled;
- the rotational dynamics is correctly coupled with translational; and
- that the integrator does not introduces considerable numerical error.

Such tests are available for run:
```
pytest tests/test_physics.py
```

# Session 3 - Algorithm Implementation

## Question 3 - Implement an algorithm of your choice that can steer the rocket towards the given landing point and that meets the requirements. Explanations on the choice of controller architecture and gains are expected.

### Controller Architecture
Options:
- PID: simple SISO, but coupled dynamics requires nested loops;
- LQR: complete MIMO, but more complex and also requires linearization;
- MPC: handles non-linearities (ie: TVC gimbal saturation), but too complex and computationally expensive.

We choose PID for simplicity. More specifically, we implement 3 loops:
- Attitude controller (inner faster loop) that manages the TVC gimbal $\delta$ to maintain the commanded pitch $\theta_{cmd}$, tuned for higher bandwidth.

- Position controller (outer slower loop) that looks at the lateral error $x_e$ and commands a tilt angle $\theta_{cmd}$ to steer the rocket, tuned for lower bandwidth.

- Altitude controller (independent loop): A PID controller that manages Thrust $T$ to track a descent profile, ensuring we hit $z=0$ with $v_z \approx 0$.

This architecture is sensitive to model plant mismatch. Therefore, we should not expect good performance in extreme scenarios (ie: high attitude angle $\theta$, obstacle avoidance).

Also, there is a risk of resonance and instability due to: true coupling of attitude and altitude, which are assumed to be decoupled in this architecture; and sensors/actuators limitations (ie: saturation, delay due to latency or inertia).

### Gains Design
As a preliminary gain selection:
- Attitude PD controller must have high bandwidth, so we prioritize high $K_p$ for $\theta$ and a strong $K_d$ to dampen oscillations. $K_i$ should be null because we assume no torque disturbances (ie: aerodinamics, CM offsets).

- Position PD controller must be significantly slower than the inner loop (usually by a factor of 5–10) to prevent the two loops from "fighting" each other and causing instability. Also, $K_i$ should be null to avoid coupling with attitude controller, and because there is no lateral speed of the target landing position.

- Altitude PID controller can have less bandwidth since its dynamic is slower. $K_i$ is null to avoid windup since the landing target is not moving. It will comprise of PID control plus feedforward for gravity compensation.

This way, Altitude controller handles the gravity, while the nested Position/Attitude controllers handle the position similar to the "Inverted Pendulum" problem.

Calculating the theoretical gains for the attitude controller, we first assume that torque is linear for small $\delta$. Then, appliying this assumption on 2nd order closed-loop equation:

### Theoretical Gain Derivation
Initial gains were derived by linearizing the plant dynamics and applying pole placement for a desired natural frequency $\omega_n$ and damping ratio $\zeta$.

#### Attitude Controller (Inner Loop)
The linearized rotational dynamics are governed by $J\ddot{\theta} = -(T \cdot l) \cdot \delta$. Using a PD law $\delta = K_{p,\theta}\theta + K_{d,\theta}\dot{\theta}$:
* **$K_{p,\theta} = \frac{J \cdot \omega_n^2}{T \cdot l}$**
* **$K_{d,\theta} = \frac{2 \zeta \omega_n \cdot J}{T \cdot l}$**
* *Target:* $\omega_n = 2.0$ rad/s, $\zeta = 0.7$ (Fast response, reduced overshoot).

#### Altitude Controller (Vertical Loop)
The vertical dynamics $m\ddot{z} = T - mg$ are controlled via $T = mg + K_{p,z}e_z + K_{d,z}\dot{e}_z + K_{i,z}\int e_z$:
* **$K_{p,z} = m \cdot \omega_n^2$**
* **$K_{d,z} = 2 \zeta \omega_n \cdot m$**
* *Target:* $\omega_n = 1.0$ rad/s, $\zeta = 1.0$ (Critical damping to prevent "bouncing").

#### Position Controller (Outer Loop)
Translates lateral error into a tilt command. Since $a_x \approx g \cdot \theta$:
* **$K_{p,x} = \frac{\omega_n^2}{g}$**
* *Target:* $\omega_n = 0.5$ rad/s (Ensuring frequency separation from the inner loop, reduced overshoot).

Autopilot (rocket steering) implementation is found in `control` folder.

Notes:
* **Frequency Separation:** The inner loop is tuned to be faster than the outer loop to ensure stability and prevent resonance.
* **Integrator Anti-Windup:** Clamping logic is applied to the Altitude Integrator to prevent command saturation during high-thrust maneuvers.
* **Small-Angle Approximation:** The controller assumes $|\theta| < 20^\circ$ to maintain TVC linearity.