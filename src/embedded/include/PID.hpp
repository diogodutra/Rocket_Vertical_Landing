/**
 * @file PID.hpp
 * @brief Generic Proportional-Integral-Derivative (PID) controller implementation.
 * * Features integrated saturation limits and clamping-based anti-windup logic 
 * to ensure stable control performance in physical actuator systems.
 */

#ifndef ROCKET_GNC_PID_HPP
#define ROCKET_GNC_PID_HPP

#include <optional>

namespace RocketGNC {

/**
 * @class PID
 * @brief Standard PID controller with optional output saturation.
 */
class PID {
public:
    /**
     * @brief Construct a new PID controller instance.
     * * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     * @param min_out Optional lower saturation limit for the output signal.
     * @param max_out Optional upper saturation limit for the output signal.
     */
    explicit PID(float kp, float ki, float kd, 
                 std::optional<float> min_out = std::nullopt, 
                 std::optional<float> max_out = std::nullopt)
        : kp(kp), ki(ki), kd(kd), min_limit(min_out), max_limit(max_out),
          integral(0.0f), prev_error(0.0f), last_output(0.0f), initialized(false) {}

    /**
     * @brief Computes the control output based on the current error.
     * * Includes protection against derivative kicks on startup and 
     * basic anti-windup by freezing the integrator during saturation.
     * * @param error The difference between the setpoint and the process variable.
     * @param dt The time delta since the last computation [s].
     * @return float The commanded output signal.
     */
    float compute(float error, float dt) noexcept {
        // Division by zero protection & clock glitch recovery
        if (dt <= 0.0f) return last_output;

        if (!initialized) {
            // Prevent a huge derivative spike (derivative kick).
            prev_error = error;
            initialized = true;
        }

        // Proportional
        float p_term = kp * error;

        // Integral
        integral += error * dt;
        float i_term = ki * integral;

        // Derivative (safe because dt > 0)
        float d_term = kd * (error - prev_error) / dt;
        prev_error = error;

        float output = p_term + i_term + d_term;

        // Apply Output Limits & Anti-Windup
        if (max_limit && output > *max_limit) {
            output = *max_limit;
            integral -= error * dt; 
        } else if (min_limit && output < *min_limit) {
            output = *min_limit;
            integral -= error * dt;
        }

        last_output = output;
        return output;
    }

    /**
     * @brief Resets the internal controller state.
     * Clears the integrator, error history, and re-triggers initialization logic.
     */
    void reset() noexcept {
        integral = 0.0f;
        prev_error = 0.0f;
        last_output = 0.0f;
        initialized = false;
    }

private:
    float kp; ///< Proportional Gain
    float ki; ///< Integral Gain
    float kd; ///< Derivative Gain

    std::optional<float> min_limit; ///< Lower output saturation limit
    std::optional<float> max_limit; ///< Upper output saturation limit

    float integral;      ///< Accumulated error integral
    float prev_error;    ///< Error from the previous time step
    float last_output;   ///< Last computed output (for dt <= 0 fallback)
    bool initialized;    ///< Flag to handle derivative kick on startup
};

} // namespace RocketGNC

#endif