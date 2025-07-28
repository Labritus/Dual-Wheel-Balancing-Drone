#include "../include/callback_test.h"
#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <atomic>
#include <future>
#include <vector>

// Real-time simulation control
static std::atomic<bool> simulation_running{true};
static std::atomic<int> test_phase{0};

// Fast, non-blocking sensor data callback function
void onSensorData(float x, float y, float z) {
    // Fast processing only - no blocking operations
    static uint64_t last_print_time = 0;
    auto current_time = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    
    // Rate-limited output (non-blocking)
    if (current_time - last_print_time > 100000) {  // 100ms rate limit
        std::cout << "Sensor callback: x=" << x << ", y=" << y << ", z=" << z;
        
        // Fast magnitude calculation
        float magnitude = std::abs(x) + std::abs(y) + std::abs(z);
        std::cout << " | Magnitude=" << magnitude << std::endl;
        
        last_print_time = current_time;
    }
}

// Fast alert event callback function
void onAlertEvent(const std::string& event_name, int event_data) {
    // Fast alert processing - no blocking
    std::cout << "Alert: " << event_name;
    
    if (event_data > 75) {
        std::cout << " [CRITICAL:" << event_data << "]";
    } else if (event_data > 50) {
        std::cout << " [MODERATE:" << event_data << "]";
    } else {
        std::cout << " [MINOR:" << event_data << "]";
    }
    std::cout << std::endl;
}

// Fast logging event callback function
void onLogEvent(const std::string& event_name, int event_data) {
    // Fast logging - immediate return
    std::cout << "Log: " << event_name << " (Code:" << event_data << ")" << std::endl;
}

// Non-blocking test orchestrator using async events
void runTestSequence(CallbackTest& callback_test) {
    // Schedule test events using non-blocking timers
    std::vector<std::future<void>> scheduled_events;
    
    // Schedule normal sensor data (immediate)
    scheduled_events.push_back(std::async(std::launch::async, [&]() {
        callback_test.simulateSensorData(1.5f, 2.3f, 0.8f);
    }));
    
    // Schedule abnormal sensor data (after 500ms)
    scheduled_events.push_back(std::async(std::launch::async, [&]() {
        // Use proper sleep instead of busy wait for test scenarios
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (simulation_running.load()) {
            callback_test.simulateSensorData(15.2f, -3.7f, 0.5f);
        }
    }));
    
    // Schedule system startup event (after 1000ms)
    scheduled_events.push_back(std::async(std::launch::async, [&]() {
        // Use proper sleep instead of busy wait for test scenarios
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (simulation_running.load()) {
            callback_test.simulateEvent("System Startup", 100);
        }
    }));
    
    // Schedule low battery alert (after 1500ms)
    scheduled_events.push_back(std::async(std::launch::async, [&]() {
        // Use proper sleep instead of busy wait for test scenarios
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        if (simulation_running.load()) {
            callback_test.simulateEvent("Low Battery", 30);
        }
    }));
    
    // Schedule sensor error (after 2000ms)
    scheduled_events.push_back(std::async(std::launch::async, [&]() {
        // Use proper sleep instead of busy wait for test scenarios
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        if (simulation_running.load()) {
            callback_test.simulateEvent("Sensor Error", 80);
        }
    }));
    
    // Schedule cleanup (after 2500ms)
    scheduled_events.push_back(std::async(std::launch::async, [&]() {
        // Use proper sleep instead of busy wait for test scenarios
        std::this_thread::sleep_for(std::chrono::milliseconds(2500));
        if (simulation_running.load()) {
            std::cout << "\nClearing all callbacks..." << std::endl;
            callback_test.clearCallbacks();
            
            // Test cleared callbacks
            callback_test.simulateSensorData(1.0f, 1.0f, 1.0f);
            callback_test.simulateEvent("Test Event", 50);
            
            std::cout << "\n===== Callback test completed =====\n" << std::endl;
            simulation_running.store(false);
        }
    }));
}

int main() {
    std::cout << "Real-time callback test (non-blocking implementation)" << std::endl;
    
    // Create callback test object
    CallbackTest callback_test;
    
    // Register fast callback functions
    callback_test.registerSensorCallback(onSensorData);
    callback_test.registerEventCallback(onAlertEvent);
    callback_test.registerEventCallback(onLogEvent);
    
    // Register fast lambda anomaly detector
    callback_test.registerSensorCallback([](float x, float y, float z) {
        // Fast anomaly detection - immediate return
        if (std::abs(x) > 10 || std::abs(y) > 10 || std::abs(z) > 10) {
            std::cout << "Anomaly: Abnormal values detected!" << std::endl;
        }
    });
    
    std::cout << "\n===== Starting real-time simulation =====\n" << std::endl;
    
    // Start non-blocking test sequence
    runTestSequence(callback_test);
    
    // Real-time main loop - non-blocking event processing
    auto start_time = std::chrono::steady_clock::now();
    while (simulation_running.load()) {
        // Check for system events (non-blocking)
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            current_time - start_time).count();
        
        // Non-blocking status updates every 1000ms
        static uint64_t last_status_time = 0;
        if (elapsed > last_status_time + 1000) {
            std::cout << "System running... (" << elapsed << "ms)" << std::endl;
            last_status_time = elapsed;
        }
        
        // Yield to other threads - no blocking sleep
        std::this_thread::yield();
        
        // Safety timeout (5 seconds)
        if (elapsed > 5000) {
            std::cout << "Safety timeout reached" << std::endl;
            simulation_running.store(false);
        }
    }
    
    return 0;
}
