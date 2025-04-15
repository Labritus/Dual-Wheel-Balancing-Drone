#include <iostream>
#include <thread>
#include <signal.h>
#include <fstream>
#include <mutex>
#include <iomanip>
#include <chrono>

#include "../include/drone_controller.h"
#include "../include/drone_status_interface.h"

// Global variables
bool running = true;
std::ofstream log_file;
std::mutex log_mutex;
DroneController drone_controller;
DroneStatusInterface& status_interface = DroneStatusInterface::getInstance();

// Signal handler for Ctrl+C
void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received, shutting down the program..." << std::endl;
    running = false;
}

// Log messages to both the console and file
void logMessage(const std::string& message) {
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    
    std::stringstream timestamp;
    timestamp << std::put_time(std::localtime(&now_time_t), "%Y-%m-%d %H:%M:%S");
    timestamp << '.' << std::setfill('0') << std::setw(3) << now_ms.count();
    
    std::string log_entry = "[" + timestamp.str() + "] " + message;
    
    // Thread-safe logging
    std::lock_guard<std::mutex> lock(log_mutex);
    std::cout << log_entry << std::endl;
    
    if (log_file.is_open()) {
        log_file << log_entry << std::endl;
    }
}

// Attitude update callback
void onAttitudeUpdate(float roll, float pitch, float yaw) {
    static int counter = 0;
    static float last_roll = 0;
    static float last_pitch = 0;
    
    // Log only when attitude change exceeds threshold or every 50 updates
    if (counter++ % 50 == 0 || 
        std::abs(roll - last_roll) > 5.0f || 
        std::abs(pitch - last_pitch) > 5.0f) {
        
        std::stringstream ss;
        ss << "Attitude update: Roll=" << std::fixed << std::setprecision(2) << roll
           << "°, Pitch=" << pitch << "°, Yaw=" << yaw << "°";
        logMessage(ss.str());
        
        // If the attitude angles exceed warning thresholds, log a warning event
        if (std::abs(roll) > 30.0f || std::abs(pitch) > 30.0f) {
            status_interface.addEvent("Attitude Warning", "Attitude angles exceed safe range");
            logMessage("Warning: Attitude angles exceed safe range!");
        }
        
        last_roll = roll;
        last_pitch = pitch;
    }
    
    // Update the attitude data in the status interface
    DroneStatus current_status = status_interface.getStatus();
    current_status.roll = roll;
    current_status.pitch = pitch;
    current_status.yaw = yaw;
    status_interface.updateStatus(current_status);
}

// Battery update callback
void onBatteryUpdate(int voltage_mv) {
    static int last_voltage = 0;
    
    // Log only when the voltage change exceeds a threshold
    if (std::abs(voltage_mv - last_voltage) > 100) {
        std::stringstream ss;
        ss << "Battery voltage: " << voltage_mv << " mV";
        logMessage(ss.str());
        
        if (voltage_mv < 11000) {
            status_interface.addEvent("Battery Warning", "Battery voltage is below 11V");
            logMessage("Warning: Battery voltage low!");
        }
        
        last_voltage = voltage_mv;
    }
    
    // Update the battery data in the status interface
    DroneStatus current_status = status_interface.getStatus();
    current_status.battery_voltage = voltage_mv;
    status_interface.updateStatus(current_status);
}

// Motor output callback
void onMotorOutput(int left_pwm, int right_pwm) {
    static int last_left_pwm = 0;
    static int last_right_pwm = 0;
    
    // Log only when PWM values change significantly
    if (std::abs(left_pwm - last_left_pwm) > 500 || std::abs(right_pwm - last_right_pwm) > 500) {
        std::stringstream ss;
        ss << "Motor output: Left PWM=" << left_pwm << ", Right PWM=" << right_pwm;
        logMessage(ss.str());
        
        last_left_pwm = left_pwm;
        last_right_pwm = right_pwm;
    }
    
    // Update the motor output data in the status interface
    DroneStatus current_status = status_interface.getStatus();
    current_status.motor_left_pwm = left_pwm;
    current_status.motor_right_pwm = right_pwm;
    status_interface.updateStatus(current_status);
}

// Error callback
void onError(const std::string& error_message, int error_code) {
    std::stringstream ss;
    ss << "Error [" << error_code << "]: " << error_message;
    logMessage(ss.str());
    
    // Log the error event
    status_interface.addEvent("System Error", ss.str());
}

// State change callback
void onStateChange(const std::string& state_name, bool state_active) {
    std::stringstream ss;
    ss << "State change: " << state_name << " is now " << (state_active ? "active" : "inactive");
    logMessage(ss.str());
    
    // Log the state change event
    status_interface.addEvent("State Change", ss.str());
    
    // Update the state description in the status interface
    DroneStatus current_status = status_interface.getStatus();
    current_status.status_description = state_name + (state_active ? " Active" : " Inactive");
    status_interface.updateStatus(current_status);
}

// Status export callback function - used to display a status summary
void onStatusExport(const DroneStatus& status) {
    static int counter = 0;
    
    // Print a status summary every 50 updates
    if (counter++ % 50 == 0) {
        std::stringstream ss;
        ss << "Status summary: "
           << "Roll=" << std::fixed << std::setprecision(2) << status.roll
           << "°, Pitch=" << status.pitch
           << "°, Battery=" << status.battery_voltage
           << " mV, Position=" << status.position
           << ", Motor=" << status.motor_left_pwm << "/" << status.motor_right_pwm;
        logMessage(ss.str());
    }
}

int main() {
    // Register the signal handler for SIGINT
    signal(SIGINT, signalHandler);
    
    // Open the log file
    log_file.open("drone_log.txt", std::ios::out | std::ios::app);
    if (!log_file.is_open()) {
        std::cerr << "Unable to open log file" << std::endl;
        return 1;
    }
    
    logMessage("Drone control system started");
    
    // Register callback functions
    drone_controller.registerAttitudeCallback(onAttitudeUpdate);
    drone_controller.registerBatteryCallback(onBatteryUpdate);
    drone_controller.registerMotorOutputCallback(onMotorOutput);
    drone_controller.registerErrorCallback(onError);
    drone_controller.registerStateChangeCallback(onStateChange);
    
    // Register status export callback
    status_interface.registerExportCallback(onStatusExport);
    
    // Start CSV and JSON logging
    status_interface.startCSVRecording("drone_status.csv");
    status_interface.startJSONRecording("drone_events.json");
    
    // Initialize hardware
    if (!drone_controller.initializeHardware()) {
        logMessage("Hardware initialization failed, exiting program");
        log_file.close();
        return 1;
    }
    
    logMessage("System initialization complete, starting control loop");
    
    // Set the target position
    drone_controller.setTargetPosition(1000);
    
    // Main control loop
    int counter = 0;
    while (running) {
        // Update the controller
        drone_controller.update();
        
        // Display status every 100 cycles
        if (counter++ % 100 == 0) {
            // Update DroneStatus data
            DroneStatus current_status = status_interface.getStatus();
            current_status.position = drone_controller.getCurrentPosition();
            current_status.velocity = drone_controller.getCurrentVelocity();
            current_status.position_mode = drone_controller.isPositionMode();
            status_interface.updateStatus(current_status);
            
            // Display the current stats
            drone_controller.displayStats();
        }
        
        // Add some loop events to demonstrate event logging
        if (counter % 1000 == 0) {
            status_interface.addEvent("Loop Event", "Completed 1000 control loops");
            logMessage("Completed 1000 control loops");
        }
        
        // Brief sleep to reduce CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    // Stop the motors
    drone_controller.stopMotors();
    
    // Stop logging recordings
    status_interface.stopCSVRecording();
    status_interface.stopJSONRecording();
    
    logMessage("System shutdown normally");
    
    // Close the log file
    log_file.close();
    
    return 0;
}
