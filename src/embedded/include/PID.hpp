#ifndef ROCKET_GNC_PID_HPP
#define ROCKET_GNC_PID_HPP

#include <optional>

namespace RocketGNC {

class PID {
public:
    explicit PID(float kp, float ki, float kd, 
                 std::optional<float> min_out = std::nullopt, 
                 std::optional<float> max_out = std::nullopt)
        : kp(kp), ki(ki), kd(kd), min_limit(min_out), max_limit(max_out),
          integral(0.0f), prev_error(0.0f), last_output(0.0f), initialized(false) {}

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

    void reset() noexcept {
        integral = 0.0f;
        prev_error = 0.0f;
        last_output = 0.0f;
        initialized = false;
    }

private:
    float kp, ki, kd;
    std::optional<float> min_limit;
    std::optional<float> max_limit;
    float integral;
    float prev_error;
    float last_output;
    bool initialized;
};

} // namespace RocketGNC

#endif