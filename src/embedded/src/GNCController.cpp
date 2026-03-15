/**
 * @file GNCController.cpp
 * @brief Implementation of the 3-DoF vertical landing control logic.
 */

#include "GNCController.hpp"
#include <algorithm>
#include <cmath>

namespace RocketGNC {

 /**
 * @details 
 * The control law follows a cascaded architecture:
 * 1. A Virtual Gate adjusts the target altitude to ensure smooth touchdown.
 * 2. The Altitude PID manages thrust magnitude with gravity feedforward.
 * 3. The Position PID (Outer Loop) translates lateral error into a commanded tilt angle.
 * 4. The Attitude PID (Inner Loop) translates tilt error into a TVC gimbal angle.
 */
ControlCommands GNCController::update(const VehicleState& state, float dt) {
    ControlCommands cmd;

    // Safety Constraint: validate that the incoming dt matches our design frequency
    float dt_protected = dt;
    if (std::abs(dt - FIXED_DT) > DT_TOLERANCE) {
        dt_protected = FIXED_DT + DT_TOLERANCE;
    }

    // Virtual Gate
    float adj_target_z = TARGET_Z;

    if (state.z > GATE_THRESHOLD) {
        adj_target_z += GATE_OFFSET;
    } else {
        // Ramp the offset to zero as we touch down
        adj_target_z += GATE_OFFSET * (state.z / GATE_THRESHOLD);
    }

    // Vertical Axis
    float alt_error = TARGET_Z - state.z;
    cmd.thrust_N = alt_pid.compute(alt_error, dt_protected) + HOVER_THRUST;
    
    // Outer Loop: Calculate required tilt   
    float x_error = TARGET_X - state.x;    
    float theta_cmd = pos_pid.compute(x_error, dt_protected);
    
    // Inner Loop: Calculate required gimbal (delta)
    float att_error = theta_cmd - state.theta;
    cmd.tvc_angle_rad = att_pid.compute(att_error, dt_protected);
    
    // Apply safety limits (clamping) to protect actuators
    apply_safety_limits(cmd);

    return cmd;
}

/**
 * @brief Hard-limits actuator commands to protect hardware integrity.
 * @details Clamps thrust based on engine throttleability and TVC based on 
 * the mechanical gimbal angle limits.
 */
void GNCController::apply_safety_limits(ControlCommands& commands) {
    // Clamp Thrust to engine physical operating range
    commands.thrust_N = std::clamp(commands.thrust_N, MIN_THRUST, MAX_THRUST);
    
    // Clamp TVC to gimbal mechanical stops
    float max_tvc_rad = MAX_TVC_DEG * DEG_TO_RAD;
    commands.tvc_angle_rad = std::clamp(commands.tvc_angle_rad, -max_tvc_rad, max_tvc_rad);
}

/**
 * @brief Reinitializes the control state.
 * @details Clears integrator accumulators and resets the 'first_run' flag 
 * to ensure no derivative spikes occur upon controller re-activation.
 */
void GNCController::reset_internal_state() {
    alt_pid.reset();
    pos_pid.reset();
    att_pid.reset();
    first_run = true;
}

} // namespace RocketGNC