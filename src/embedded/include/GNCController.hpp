#ifndef ROCKET_GNC_CONTROLLER_HPP
#define ROCKET_GNC_CONTROLLER_HPP

#include <cstdint>
#include "PID.hpp"

namespace RocketGNC {

static constexpr float PI = 3.14159265358979323846f;
static constexpr float DEG_TO_RAD = PI / 180.0f;

// Define the expected flight computer tick rate
static constexpr float EXPECTED_HZ = 100.0f;
static constexpr float FIXED_DT = 1.0f / EXPECTED_HZ;
static constexpr float DT_TOLERANCE = 0.001f; // 1ms tolerance

struct VehicleState {
    float x; float z; float theta;
    float vx; float vz; float vtheta;
};

struct ControlCommands {
    float thrust_N;
    float tvc_angle_rad;
};

class GNCController {
public:
    // Physical Constraints
    static constexpr float MAX_THRUST = 68000.0f;
    static constexpr float MIN_THRUST = 48000.0f;
    static constexpr float MAX_TVC_DEG = 10.0f;
    static constexpr float HOVER_THRUST = 49050.0f;

    // TARGET STATES
    static constexpr float TARGET_X = 0.0f;
    static constexpr float TARGET_Z = 0.0f;
    static constexpr float TARGET_T = 0.0f;
    static constexpr float TARGET_VX = 0.0f;
    static constexpr float TARGET_VZ = -0.5f;
    static constexpr float TARGET_VT = 0.0f;

    /** * TUNED CONTROL GAINS 
     * Hardcoding these as constexpr ensures they reside in read-only memory.
     * Replace these values with the output from your Python gain script.
     */
    static constexpr float KP_Z = 5000.0f;
    static constexpr float KD_Z = 10000.0f; 
    
    static constexpr float KP_T = 45.8716f;
    static constexpr float KD_T = 30.5810f;
    
    static constexpr float KP_X = 0.3876f;
    static constexpr float KD_X = 0.1193f;

    static constexpr float MAX_TCMD_Z = 100000.0f; // thrust_cmd max saturation on altitude controller (rad)
    static constexpr float MIN_TCMD_Z = 0.0f; // thrust_cmd max saturation on altitude controller (rad)
    static constexpr float MAX_TCMD_X = 0.35f; // theta_cmd saturation on position controller (rad)
    static constexpr float MAX_TCMD_T = 0.17f; // theta_cmd saturation on attitude controller (rad)

    GNCController() :
        alt_pid(KP_Z, 0.0f, KD_Z,  MIN_TCMD_Z, MAX_TCMD_Z),
        pos_pid(KP_X, 0.0f, KD_X, -MAX_TCMD_X, MAX_TCMD_X),
        att_pid(KP_T, 0.0f, KD_T, -MAX_TCMD_T, MAX_TCMD_T) {}

    ControlCommands update(const VehicleState& state, float dt);
    void reset_internal_state();
    
private:
    float prev_error_theta = 0.0f;
    bool first_run = true; // To prevent derivative spikes
    void apply_safety_limits(ControlCommands& commands);
    PID alt_pid;
    PID pos_pid;
    PID att_pid;
};

} // namespace RocketGNC

#endif