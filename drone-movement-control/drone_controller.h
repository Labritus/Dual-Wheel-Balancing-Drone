#ifndef DRONE_CONTROLLER_H
#define DRONE_CONTROLLER_H

#include "motor_controller.h"
#include "mpu6050.h"
#include "encoder.h"
#include "pid_controller.h"
#include <string>
#include <cstdint>
#include <functional>
#include <vector>

// Define callback function types
using AttitudeUpdateCallback = std::function<void(float roll, float pitch, float yaw)>;
using BatteryUpdateCallback = std::function<void(int voltage)>;
using MotorOutputCallback = std::function<void(int left_pwm, int right_pwm)>;
using ErrorCallback = std::function<void(const std::string& error_message, int error_code)>;
using StateChangeCallback = std::function<void(const std::string& state_name, bool state_active)>;

class DroneController {
public:
    DroneController();
    ~DroneController();
    
    // Initialize hardware
    bool initializeHardware();
    
    // Update controller state
    void update();
    
    // Display current status
    void displayStats();
    
    // Stop all motors
    void stopMotors();
    
    // Set target position
    void setTargetPosition(int position);
    
    // Set target velocity
    void setTargetVelocity(int velocity);
    
    // Get battery voltage (mV)
    int getBatteryVoltage() const;
    
    // Get attitude data
    AttitudeData getAttitude() const;
    
    // Get current position
    int getCurrentPosition() const;
    
    // Get current velocity
    int getCurrentVelocity() const;
    
    // Get motor PWM values
    int getMotorLeftPWM() const;
    int getMotorRightPWM() const;
    
    // Get control mode
    bool isPositionMode() const;
    
    // Register callback functions
    void registerAttitudeCallback(AttitudeUpdateCallback callback);
    void registerBatteryCallback(BatteryUpdateCallback callback);
    void registerMotorOutputCallback(MotorOutputCallback callback);
    void registerErrorCallback(ErrorCallback callback);
    void registerStateChangeCallback(StateChangeCallback callback);
    
    // Clear all callbacks
    void clearCallbacks();
    
    // Switch control modes
    void switchToPositionMode();
    void switchToVelocityMode();
    
private:
    // Check if battery voltage is too low
    bool checkBatteryVoltage();
    
    // Handle keyboard input
    void handleKeyboard();
    
    // Trigger callback functions
    void triggerAttitudeCallbacks();
    void triggerBatteryCallbacks();
    void triggerMotorOutputCallbacks(int left_pwm, int right_pwm);
    void triggerErrorCallbacks(const std::string& error_message, int error_code);
    void triggerStateChangeCallbacks(const std::string& state_name, bool state_active);
    
    // Controller options
    bool position_mode_; // true = position mode, false = velocity mode
    
    // Hardware interfaces
    MotorController motor_controller_;
    MPU6050 mpu6050_;
    Encoder encoder_left_;
    Encoder encoder_right_;
    
    // PID controllers
    PIDController position_pid_;
    PIDController velocity_pid_;
    
    // Target values
    int target_position_;
    int target_velocity_;
    
    // Current state
    int current_position_;
    int current_velocity_;
    AttitudeData attitude_;
    int battery_voltage_;
    
    // Control output
    int motor_pwm_;
    int motor_right_pwm_; // Right motor PWM value
    
    // I2C device path
    std::string i2c_device_;
    
    // Callback storage
    std::vector<AttitudeUpdateCallback> attitude_callbacks_;
    std::vector<BatteryUpdateCallback> battery_callbacks_;
    std::vector<MotorOutputCallback> motor_output_callbacks_;
    std::vector<ErrorCallback> error_callbacks_;
    std::vector<StateChangeCallback> state_change_callbacks_;
    
    // State tracking
    bool low_battery_triggered_;
    bool motors_active_;
};

#endif // DRONE_CONTROLLER_H
