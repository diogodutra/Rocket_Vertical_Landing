#include "GNCController.hpp"
#include <algorithm>
#include <cmath>

namespace RocketGNC {

ControlCommands GNCController::update(const VehicleState& state, float dt) {
    ControlCommands cmd;

    // Safety Constraint: validate that the incoming dt matches our design frequency
    float dt_protected = dt;
    if (std::abs(dt - FIXED_DT) > DT_TOLERANCE) {
        dt_protected = FIXED_DT + DT_TOLERANCE;
    }

    // Vertical Axis
    float alt_error = TARGET_Z - state.z;
    float vel_error = (TARGET_VZ - state.vz) * 0.1f;
    cmd.thrust_N = alt_pid.compute(alt_error + vel_error, dt_protected) + HOVER_THRUST;
       
    // Horizontal / Rotational Axis
    float x_error = TARGET_X - state.x;
    float vx_error = (TARGET_VX - state.vx) * 4.0f;
    
    // Outer Loop: Calculate required tilt
    float theta_cmd = pos_pid.compute(x_error + vx_error, dt_protected) / 15.0f;
    
    // Landing Smoothing
    const float z_landing = 1.0f;
    if (state.z < z_landing) {
        theta_cmd *= (state.z) / z_landing;
    }
    
    // Inner Loop: Calculate required gimbal (delta)
    float att_error = theta_cmd - state.theta;
    cmd.tvc_angle_rad = -att_pid.compute(att_error, dt_protected);
    
    // Apply safety limits (clamping)
    apply_safety_limits(cmd);

    return cmd;
}

void GNCController::apply_safety_limits(ControlCommands& commands) {
    // Clamp Thrust to engine physical operating range
    commands.thrust_N = std::clamp(commands.thrust_N, MIN_THRUST, MAX_THRUST);
    
    // Clamp TVC to gimbal mechanical stops
    float max_tvc_rad = MAX_TVC_DEG * (PI / 180.0f);
    commands.tvc_angle_rad = std::clamp(commands.tvc_angle_rad, -max_tvc_rad, max_tvc_rad);
}

void GNCController::reset_internal_state() {
    alt_pid.reset();
    pos_pid.reset();
    att_pid.reset();
    first_run = true;
}

} // namespace RocketGNC