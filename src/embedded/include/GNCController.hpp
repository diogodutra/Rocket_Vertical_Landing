/**
 * @file GNCController.hpp
 * @brief 3-DoF Rocket Guidance and Control logic for vertical landing.
 * * Coordinate System: Body-Fixed (X: Lateral, Z: Vertical/Altitude).
 * Units: SI (Meters, Radians, Newtons, Seconds).
 */

#ifndef ROCKET_GNC_CONTROLLER_HPP
#define ROCKET_GNC_CONTROLLER_HPP

#include <cstdint>
#include "PID.hpp"

namespace RocketGNC {

static constexpr float PI = 3.14159265358979323846f;   ///< Pi mathematical constant
static constexpr float DEG_TO_RAD = PI / 180.0f;       ///< Conversion of angle from degrees to radians

// Define the expected flight computer tick rate
static constexpr float EXPECTED_HZ = 100.0f;           ///< Sampling frequency for GNC computation cycle [Hz]
static constexpr float FIXED_DT = 1.0f / EXPECTED_HZ;  ///< Time increment calculated from sampling frequency [s]
static constexpr float DT_TOLERANCE = 0.001f;          ///< Time increment tolerance [s]

/** @brief Vehicle orientation and kinematics in the 2D plane. */
struct VehicleState {
    float x;      ///< Lateral position [m]
    float z;      ///< Vertical altitude [m]
    float theta;  ///< Tilt angle from vertical [rad]
    float vx;     ///< Lateral velocity [m/s]
    float vz;     ///< Vertical velocity [m/s]
    float vtheta; ///< Angular velocity [rad/s]
};

/** @brief Actuator commands produced by the GNC loop. */
struct ControlCommands {
    float thrust_N;      ///< Commanded thrust magnitude [N]
    float tvc_angle_rad; ///< Thrust Vector Control (Gimbal) angle [rad]
};

/**
 * @class GNCController
 * @brief Implements a cascaded PID control architecture for soft landing.
 */
class GNCController {
public:
    // Physical Constraints
    static constexpr float MAX_THRUST = 68000.0f;   ///< Maximum thrust command [kN]
    static constexpr float MIN_THRUST = 48000.0f;   ///< Minimum thrust command [kN]
    static constexpr float MAX_TVC_DEG = 10.0f;     ///< Maximum TVC gimbal range [deg]
    static constexpr float HOVER_THRUST = 49050.0f; ///< Theoretical hovering thrust for feedfoward [kN]

    // TARGET STATES
    static constexpr float TARGET_X = 0.0f;   // Landing lateral position [m]
    static constexpr float TARGET_Z = 0.0f;   // Landing vertical position [m]
    static constexpr float TARGET_T = 0.0f;   // Landing attitude [rad]
    static constexpr float TARGET_VX = 0.0f;  // Landing lateral speed [m/s]
    static constexpr float TARGET_VZ = -0.5f; // Landing vertical speed [m/s]
    static constexpr float TARGET_VT = 0.0f;  // Landing angular speed [rad/s]

    // CONTROL GAINS
    static constexpr float KP_Z =  5000.0f;  ///< Proportional gain for Altitude controller
    static constexpr float KD_Z = 10000.0f;  ///< Derivative gain for Altitude controller 
    
    static constexpr float KP_T = -81.5494f; ///< Proportional gain for Attitude controller
    static constexpr float KD_T = -32.6198f; ///< Derivative gain for Attitude controller
    
    static constexpr float KP_X = 0.0499f;   ///< Proportional gain for Position controller
    static constexpr float KD_X = 0.1713f;   ///< Derivative gain for Position controller

    static constexpr float MAX_TCMD_Z = 75000.0f; ///< Maximum thrust command saturation on Altitude controller [kN]
    static constexpr float MIN_TCMD_Z = 0.0f;     ///< Minimum thrust command saturation on Altitude controller [kN]
    static constexpr float MAX_TCMD_X = 0.02f;    ///< Attitude command range saturation on Position controller [rad]
    static constexpr float MAX_TCMD_T = 0.17f;    ///< Attitude command range saturation on Position controller [rad]
    
    static constexpr float GATE_OFFSET = 0.5f;    ///< Elevation of TARGET_Z when not near landing position
    static constexpr float GATE_THRESHOLD = 2.0f; ///< Distance above TARGET_Z that applies 100% of GATE_OFFSET

    GNCController() :
        alt_pid(KP_Z, 0.0f, KD_Z,  MIN_TCMD_Z, MAX_TCMD_Z),
        pos_pid(KP_X, 0.0f, KD_X, -MAX_TCMD_X, MAX_TCMD_X),
        att_pid(KP_T, 0.0f, KD_T, -MAX_TCMD_T, MAX_TCMD_T) {}

    /**
     * @brief Computes the next control iteration.
     * * Handles the virtual gate logic for terminal descent and nested loop 
     * closure for position/attitude.
     * * @param state Current estimated vehicle state.
     * @param dt Time delta since last update [s].
     * @return ControlCommands Normalized actuator setpoints.
     */
    ControlCommands update(const VehicleState& state, float dt);

    /** @brief Resets PID integrators and derivative filters. */
    void reset_internal_state();
    
private:
    float prev_error_theta = 0.0f; ///< Previous attitude error used for manual derivative calculations [rad]
    bool first_run = true;         ///< Guard flag to prevent derivative spikes on the first control tick

    /** * @brief Clamps control outputs to physical hardware limits.
     * Ensures thrust and TVC angles do not exceed engine or gimbal specifications.
     * @param commands The raw command struct to be modified in-place.
     */
    void apply_safety_limits(ControlCommands& commands);
    PID alt_pid;                   ///< Independent PID controller for vertical channel (Altitude)
    PID pos_pid;                   ///< Outer-loop PID controller for lateral position (X-axis)
    PID att_pid;                   ///< Inner-loop PID controller for vehicle attitude (Theta)
};

} // namespace RocketGNC

#endif