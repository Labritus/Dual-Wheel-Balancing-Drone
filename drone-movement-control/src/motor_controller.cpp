#include "../include/motor_controller.h"
#include "../include/config.h"
#include <iostream>
#include <cmath>

MotorController::MotorController() : initialized_(false) {
}

MotorController::~MotorController() {
    // 确保在对象销毁时停止电机
    stop();
}

bool MotorController::initialize() {
    if (initialized_) {
        return true;
    }
    
    // 初始化GPIO引脚
    // 电机A控制引脚
    if (!gpio_.initialize(MOTOR_AIN1_PIN, PinMode::OUTPUT) ||
        !gpio_.initialize(MOTOR_AIN2_PIN, PinMode::OUTPUT) ||
        !gpio_.initialize(MOTOR_PWMA_PIN, PinMode::PWM)) {
        std::cerr << "无法初始化电机A控制引脚" << std::endl;
        return false;
    }
    
    // 电机B控制引脚
    if (!gpio_.initialize(MOTOR_BIN1_PIN, PinMode::OUTPUT) ||
        !gpio_.initialize(MOTOR_BIN2_PIN, PinMode::OUTPUT) ||
        !gpio_.initialize(MOTOR_PWMB_PIN, PinMode::PWM)) {
        std::cerr << "无法初始化电机B控制引脚" << std::endl;
        return false;
    }
    
    // 设置初始状态为停止
    stop();
    
    initialized_ = true;
    return true;
}

bool MotorController::setPWM(int motor_left, int motor_right) {
    if (!initialized_) {
        return false;
    }
    
    // 限制PWM范围
    motor_left = limitPWM(motor_left, PWM_MAX_VALUE, PWM_MIN_VALUE);
    motor_right = limitPWM(motor_right, PWM_MAX_VALUE, PWM_MIN_VALUE);
    
    // 设置电机A方向和PWM
    if (motor_left < 0) {
        gpio_.digitalWrite(MOTOR_AIN1_PIN, PinState::HIGH);
        gpio_.digitalWrite(MOTOR_AIN2_PIN, PinState::LOW);
    } else {
        gpio_.digitalWrite(MOTOR_AIN1_PIN, PinState::LOW);
        gpio_.digitalWrite(MOTOR_AIN2_PIN, PinState::HIGH);
    }
    
    // 设置电机B方向和PWM
    if (motor_right < 0) {
        gpio_.digitalWrite(MOTOR_BIN1_PIN, PinState::HIGH);
        gpio_.digitalWrite(MOTOR_BIN2_PIN, PinState::LOW);
    } else {
        gpio_.digitalWrite(MOTOR_BIN1_PIN, PinState::LOW);
        gpio_.digitalWrite(MOTOR_BIN2_PIN, PinState::HIGH);
    }
    
    // 设置PWM值 (缩放到0-255范围)
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
    
    // 停止电机A
    gpio_.digitalWrite(MOTOR_AIN1_PIN, PinState::LOW);
    gpio_.digitalWrite(MOTOR_AIN2_PIN, PinState::LOW);
    gpio_.setPWM(MOTOR_PWMA_PIN, 0);
    
    // 停止电机B
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