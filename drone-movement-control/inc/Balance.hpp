#ifndef __BALANCE_HPP
#define __BALANCE_HPP

#include "System.hpp"
#include "Control.hpp"
#include "Motor.hpp"
#include "LED.hpp"
#include "MPU6050.hpp"
#include "OLED.hpp"
#include "Key.hpp"
#include "Delay.hpp"
// Removed STM32-specific includes: Encoder, ADC, Ultrasonic
#include <atomic>
#include <chrono>
#include <functional>

// Modern callback function type definition
using BalanceCallback = std::function<void(float angle, float gyro, int left_speed, int right_speed)>;

class Balance {
public:
    // Initialize the self-balancing car (non-blocking)
    static bool init();
    
    // Process one iteration (replaces blocking run() loop)
    static void processOnce();
    
    // Set a callback function
    static void setCallback(BalanceCallback callback);
    
    // Get current system state
    static bool isInitialized() { return initialized_.load(); }
    static bool isRunning() { return running_.load(); }
    
    // Control system state
    static void start() { running_.store(true); }
    static void stop() { running_.store(false); }
    
    // Get current sensor readings (thread-safe)
    static float getCurrentAngle() { return current_angle_.load(); }
    static float getCurrentGyro() { return current_gyro_.load(); }
    static int getLeftSpeed() { return left_speed_.load(); }
    static int getRightSpeed() { return right_speed_.load(); }
    
    // Legacy run function - DEPRECATED
    [[deprecated("Use processOnce() in real-time thread instead")]]
    static void run();
    
    // Legacy callback handler - DEPRECATED
    static void handleCallback();
    
private:
    static BalanceCallback m_callback;
    static std::atomic<bool> initialized_;
    static std::atomic<bool> running_;
    static std::atomic<float> current_angle_;
    static std::atomic<float> current_gyro_;
    static std::atomic<int> left_speed_;
    static std::atomic<int> right_speed_;
    static uint64_t last_callback_time_;
    
    // Internal processing functions
    static void updateSensorData();
    static void executeControlLoop();
    static void invokeCallback();
};

#endif // __BALANCE_HPP