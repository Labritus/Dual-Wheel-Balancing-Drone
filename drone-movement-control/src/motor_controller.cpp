#include "../include/motor_controller.h"
#include "../include/config.h"
#include <iostream>
#include <cmath>

MotorController::MotorController() : initialized_(false) {
}

MotorController::~MotorController() {
    // Ensure the motors are stopped when the object is destroyed
    stop();
}

bool MotorController::initialize() {
    if (initialized_) {
        return true;
    }
    
    // Initialize the GPIO pins
    // Motor A control pins
    if (!gpio_.initialize(MOTOR_AIN1_PIN, PinMode::OUTPUT) ||
        !gpio_.initialize(MOTOR_AIN2_PIN, PinMode::OUTPUT) ||
        !gpio_.initialize(MOTOR_PWMA_PIN, PinMode::PWM)) {
        std::cerr << "Unable to initialize Motor A control pins" << std::endl;
        return false;
    }
    
    // Motor B control pins
    if (!gpio_.initialize(MOTOR_BIN1_PIN, PinMode::OUTPUT) ||
        !gpio_.initialize(MOTOR_BIN2_PIN, PinMode::OUTPUT) ||
        !gpio_.initialize(MOTOR_PWMB_PIN, PinMode::PWM)) {
        std::cerr << "Unable to initialize Motor B control pins" << std::endl;
        return false;
    }
    
    // Set the initial state to stop
    stop();
    
    initialized_ = true;
    return true;
}

bool MotorController::setPWM(int motor_left, int motor_right) {
    if (!initialized_) {
        return false;
    }
    
    // Limit the PWM range
    motor_left = limitPWM(motor_left, PWM_MAX_VALUE, PWM_MIN_VALUE);
    motor_right = limitPWM(motor_right, PWM_MAX_VALUE, PWM_MIN_VALUE);
    
    // Set Motor A direction and PWM
    if (motor_left < 0) {
        gpio_.digitalWrite(MOTOR_AIN1_PIN, PinState::HIGH);
        gpio_.digitalWrite(MOTOR_AIN2_PIN, PinState::LOW);
    } else {
        gpio_.digitalWrite(MOTOR_AIN1_PIN, PinState::LOW);
        gpio_.digitalWrite(MOTOR_AIN2_PIN, PinState::HIGH);
    }
    
    // Set Motor B direction and PWM
    if (motor_right < 0) {
        gpio_.digitalWrite(MOTOR_BIN1_PIN, PinState::HIGH);
        gpio_.digitalWrite(MOTOR_BIN2_PIN, PinState::LOW);
    } else {
        gpio_.digitalWrite(MOTOR_BIN1_PIN, PinState::LOW);
        gpio_.digitalWrite(MOTOR_BIN2_PIN, PinState::HIGH);
    }
    
    // Set PWM values (scaled to a 0-255 range)
    int pwm_left = std::abs(motor_left) * 255 / PWM_MAX_VALUE;
    int pwm_right = std::abs(motor_right) * 255 / PWM_MAX_VALUE;
    
    gpio_.setPWM(MOTOR_PWMA_PIN, pwm_left);
    gpio_.setPWM(MOTOR_PWMB_PIN, pwm_right);
    
    return true;
}

void MotorController::stop() {
    if (!initialized_) {
        return;
    }
    
    // Stop Motor A
    gpio_.digitalWrite(MOTOR_AIN1_PIN, PinState::LOW);
    gpio_.digitalWrite(MOTOR_AIN2_PIN, PinState::LOW);
    gpio_.setPWM(MOTOR_PWMA_PIN, 0);
    
    // Stop Motor B
    gpio_.digitalWrite(MOTOR_BIN1_PIN, PinState::LOW);
    gpio_.digitalWrite(MOTOR_BIN2_PIN, PinState::LOW);
    gpio_.setPWM(MOTOR_PWMB_PIN, 0);
}

bool MotorController::isInitialized() const {
    return initialized_;
}

int MotorController::limitPWM(int pwm, int max_value, int min_value) {
    if (pwm > max_value) {
        return max_value;
    } else if (pwm < min_value) {
        return min_value;
    }
    return pwm;
}
