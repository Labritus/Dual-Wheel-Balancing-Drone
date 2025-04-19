#include "../include/drone_controller.h"
#include "../include/config.h"
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <iomanip>

DroneController::DroneController() :
    position_mode_(true),
    position_pid_(DEFAULT_POSITION_KP, DEFAULT_POSITION_KI, DEFAULT_POSITION_KD),
    velocity_pid_(DEFAULT_VELOCITY_KP, DEFAULT_VELOCITY_KI, 0.0f),
    target_position_(0),
    target_velocity_(0),
    current_position_(0),
    current_velocity_(0),
    battery_voltage_(0),
    motor_pwm_(0),
    motor_right_pwm_(0),
    i2c_device_("/dev/i2c-1"), // Default I2C device path, may need to be adjusted
    low_battery_triggered_(false),
    motors_active_(false)
{
    // Set PID limits
    position_pid_.setOutputLimit(-6900, 6900);
    velocity_pid_.setOutputLimit(-6900, 6900);
    position_pid_.setIntegralLimit(100000);
    velocity_pid_.setIntegralLimit(10000);
}

DroneController::~DroneController() {
    stopMotors();
    clearCallbacks();
}

bool DroneController::initializeHardware() {
    std::cout << "Initializing motor controller..." << std::endl;
    if (!motor_controller_.initialize()) {
        std::cerr << "Motor controller initialization failed" << std::endl;
        triggerErrorCallbacks("Motor controller initialization failed", 1001);
        return false;
    }
    
    std::cout << "Initializing MPU6050 sensor..." << std::endl;
    if (!mpu6050_.initialize(i2c_device_)) {
        std::cerr << "MPU6050 initialization failed" << std::endl;
        triggerErrorCallbacks("MPU6050 initialization failed", 1002);
        return false;
    }
    
    std::cout << "Initializing encoders..." << std::endl;
    if (!encoder_left_.initialize(ENCODER_A_PIN1, ENCODER_A_PIN2) ||
        !encoder_right_.initialize(ENCODER_B_PIN1, ENCODER_B_PIN2)) {
        std::cerr << "Encoder initialization failed" << std::endl;
        triggerErrorCallbacks("Encoder initialization failed", 1003);
        return false;
    }
    
    // Start encoder counting
    encoder_left_.start();
    encoder_right_.start();
    
    // Set initial position if in position mode
    if (position_mode_) {
        encoder_left_.resetCount();
        encoder_right_.resetCount();
        triggerStateChangeCallbacks("Position Mode", true);
    } else {
        triggerStateChangeCallbacks("Velocity Mode", true);
    }
    
    std::cout << "Hardware initialization completed" << std::endl;
    triggerStateChangeCallbacks("System Ready", true);
    return true;
}

void DroneController::update() {
    // Read sensor data
    attitude_ = mpu6050_.calculateAttitude();
    triggerAttitudeCallbacks();
    
    // Get motor encoder data
    if (position_mode_) {
        // Position mode: use encoder count
        current_position_ = encoder_left_.getCount();
        
        // Compute position PID
        motor_pwm_ = position_pid_.computePosition(target_position_, current_position_);
    } else {
        // Velocity mode: use encoder velocity
        current_velocity_ = encoder_left_.getVelocity();
        
        // Compute velocity PID
        motor_pwm_ = velocity_pid_.computeIncremental(target_velocity_, current_velocity_);
    }
    
    // Temporarily set right motor PWM to 0 (might implement differential steering later)
    motor_right_pwm_ = 0;
    
    // Check battery voltage
    if (checkBatteryVoltage()) {
        // Battery voltage is normal, set motor PWM
        motor_controller_.setPWM(motor_pwm_, motor_right_pwm_);
        triggerMotorOutputCallbacks(motor_pwm_, motor_right_pwm_);
        
        // Restart motors if previously stopped
        if (!motors_active_) {
            motors_active_ = true;
            triggerStateChangeCallbacks("Motors Running", true);
        }
    } else {
        // Battery voltage too low, stop motors
        if (motors_active_) {
            stopMotors();
            triggerStateChangeCallbacks("Motors Running", false);
            motors_active_ = false;
        }
    }
    
    // Handle keyboard input
    handleKeyboard();
}

void DroneController::displayStats() {
    std::cout << "===================== Status Info =====================" << std::endl;
    std::cout << "Mode: " << (position_mode_ ? "Position Mode" : "Velocity Mode") << std::endl;
    
    if (position_mode_) {
        std::cout << "Target Position: " << target_position_ << ", Current Position: " << current_position_ << std::endl;
    } else {
        std::cout << "Target Velocity: " << target_velocity_ << ", Current Velocity: " << current_velocity_ << std::endl;
    }
    
    std::cout << "Battery Voltage: " << battery_voltage_ << "mV" << std::endl;
    std::cout << "Attitude: Roll=" << std::fixed << std::setprecision(2) << attitude_.roll 
              << "°, Pitch=" << attitude_.pitch << "°" << std::endl;
    std::cout << "Motor PWM: Left=" << motor_pwm_ << ", Right=" << motor_right_pwm_ << std::endl;
    std::cout << "======================================================" << std::endl;
}

void DroneController::stopMotors() {
    motor_controller_.stop();
    motor_pwm_ = 0;
    motor_right_pwm_ = 0;
    triggerMotorOutputCallbacks(0, 0);
    motors_active_ = false;
}

void DroneController::setTargetPosition(int position) {
    target_position_ = position;
}

void DroneController::setTargetVelocity(int velocity) {
    target_velocity_ = velocity;
}

int DroneController::getBatteryVoltage() const {
    return battery_voltage_;
}

AttitudeData DroneController::getAttitude() const {
    return attitude_;
}

int DroneController::getCurrentPosition() const {
    return current_position_;
}

int DroneController::getCurrentVelocity() const {
    return current_velocity_;
}

int DroneController::getMotorLeftPWM() const {
    return motor_pwm_;
}

int DroneController::getMotorRightPWM() const {
    return motor_right_pwm_;
}

bool DroneController::isPositionMode() const {
    return position_mode_;
}

bool DroneController::checkBatteryVoltage() {
    // Actual battery voltage reading should be implemented here
    // For demonstration, assume battery voltage is constant
    battery_voltage_ = 12000; // 12V
    
    // Notify battery status change
    triggerBatteryCallbacks();
    
    // Check if voltage is below threshold
    bool is_voltage_normal = battery_voltage_ >= BATTERY_MIN_VOLTAGE;
    
    // If battery voltage is low, trigger warning
    if (!is_voltage_normal && !low_battery_triggered_) {
        triggerErrorCallbacks("Battery voltage too low", 2001);
        triggerStateChangeCallbacks("Low Battery Warning", true);
        low_battery_triggered_ = true;
    } else if (is_voltage_normal && low_battery_triggered_) {
        // Voltage recovered
        triggerStateChangeCallbacks("Low Battery Warning", false);
        low_battery_triggered_ = false;
    }
    
    return is_voltage_normal;
}

void DroneController::handleKeyboard() {
    // In a real application, functionality for controlling the drone via button or serial input can be added here
    // For simplicity, keyboard handling is not implemented
}

// Register callback functions
void DroneController::registerAttitudeCallback(AttitudeUpdateCallback callback) {
    attitude_callbacks_.push_back(callback);
}

void DroneController::registerBatteryCallback(BatteryUpdateCallback callback) {
    battery_callbacks_.push_back(callback);
}

void DroneController::registerMotorOutputCallback(MotorOutputCallback callback) {
    motor_output_callbacks_.push_back(callback);
}

void DroneController::registerErrorCallback(ErrorCallback callback) {
    error_callbacks_.push_back(callback);
}

void DroneController::registerStateChangeCallback(StateChangeCallback callback) {
    state_change_callbacks_.push_back(callback);
}

// Clear all callbacks
void DroneController::clearCallbacks() {
    attitude_callbacks_.clear();
    battery_callbacks_.clear();
    motor_output_callbacks_.clear();
    error_callbacks_.clear();
    state_change_callbacks_.clear();
}

// Trigger callback functions
void DroneController::triggerAttitudeCallbacks() {
    for (const auto& callback : attitude_callbacks_) {
        callback(attitude_.roll, attitude_.pitch, attitude_.yaw);
    }
}

void DroneController::triggerBatteryCallbacks() {
    for (const auto& callback : battery_callbacks_) {
        callback(battery_voltage_);
    }
}

void DroneController::triggerMotorOutputCallbacks(int left_pwm, int right_pwm) {
    for (const auto& callback : motor_output_callbacks_) {
        callback(left_pwm, right_pwm);
    }
}

void DroneController::triggerErrorCallbacks(const std::string& error_message, int error_code) {
    for (const auto& callback : error_callbacks_) {
        callback(error_message, error_code);
    }
}

void DroneController::triggerStateChangeCallbacks(const std::string& state_name, bool state_active) {
    for (const auto& callback : state_change_callbacks_) {
        callback(state_name, state_active);
    }
}

// Switch control modes
void DroneController::switchToPositionMode() {
    if (!position_mode_) {
        position_mode_ = true;
        encoder_left_.resetCount();
        encoder_right_.resetCount();
        triggerStateChangeCallbacks("Position Mode", true);
        triggerStateChangeCallbacks("Velocity Mode", false);
    }
}

void DroneController::switchToVelocityMode() {
    if (position_mode_) {
        position_mode_ = false;
        triggerStateChangeCallbacks("Position Mode", false);
        triggerStateChangeCallbacks("Velocity Mode", true);
    }
}
