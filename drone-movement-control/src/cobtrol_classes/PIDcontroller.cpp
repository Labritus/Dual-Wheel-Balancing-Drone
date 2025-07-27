#include "PIDController.hpp"
#include <algorithm> // For std::clamp

PIDController::PIDController(float kp, float ki, float kd, float output_min, float output_max)
    : kp_(kp), ki_(ki), kd_(kd),
      setpoint_(0.0f), integral_(0.0f), prev_error_(0.0f),
      output_min_(output_min), output_max_(output_max),
      anti_windup_enabled_(false), integral_limit_(0.0f) {}

void PIDController::setParameters(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PIDController::setOutputLimits(float min, float max) {
    output_min_ = min;
    output_max_ = max;
}

void PIDController::setAntiWindup(bool enabled, float limit) {
    anti_windup_enabled_ = enabled;
    integral_limit_ = limit;
}

void PIDController::setSetpoint(float setpoint) {
    setpoint_ = setpoint;
}

void PIDController::reset() {
    integral_ = 0.0f;
    prev_error_ = 0.0f;
}

float PIDController::compute(float process_value, float dt) {
    // Calculate error
    const float error = setpoint_ - process_value;
    
    // Proportional term
    const float p_term = kp_ * error;

    float i_term = 0.0f;
    if (ki_ != 0.0f && dt > 0.0f) {
        integral_ += error * dt;

        if (anti_windup_enabled_) {
            if (integral_limit_ > 0.0f) {
                integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);
            } else {
                // Calculate maximum allowable integral based on output limits
                const float max_integral = (output_max_ - p_term) / ki_;
                const float min_integral = (output_min_ - p_term) / ki_;
                integral_ = std::clamp(integral_, min_integral, max_integral);
            }
        }
        i_term = ki_ * integral_;
    }
    

    const float d_term = (dt > 0.0f) ? kd_ * (prev_error_ - error) / dt : 0.0f;
    prev_error_ = error;
    

    float output = p_term + i_term + d_term;

    output = std::clamp(output, output_min_, output_max_);
    
    return output;
}

void PIDController::getParameters(float& kp, float& ki, float& kd) const {
    kp = kp_;
    ki = ki_;
    kd = kd_;
}