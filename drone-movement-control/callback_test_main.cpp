#include "../include/callback_test.h"
#include <iostream>
#include <string>
#include <chrono>
#include <thread>

// Sensor data callback function
void onSensorData(float x, float y, float z) {
    std::cout << "Received sensor data callback: x=" << x << ", y=" << y << ", z=" << z << std::endl;
    
    // Compute the sum of absolute values as a simple analysis
    float magnitude = std::abs(x) + std::abs(y) + std::abs(z);
    std::cout << "    Data analysis: Magnitude=" << magnitude << std::endl;
}

// Alert level event callback function
void onAlertEvent(const std::string& event_name, int event_data) {
    std::cout << "Received alert event callback: " << event_name << std::endl;
    
    if (event_data > 75) {
        std::cout << "    Critical alert! Level: " << event_data << std::endl;
    } else if (event_data > 50) {
        std::cout << "    Moderate alert, Level: " << event_data << std::endl;
    } else {
        std::cout << "    Minor alert, Level: " << event_data << std::endl;
    }
}

// Logging event callback function
void onLogEvent(const std::string& event_name, int event_data) {
    std::cout << "Received log event callback: " << event_name << " (Code: " << event_data << ")" << std::endl;
}

int main() {
    // Create callback test object
    CallbackTest callback_test;
    
    // Register callback functions
    callback_test.registerSensorCallback(onSensorData);
    callback_test.registerEventCallback(onAlertEvent);
    callback_test.registerEventCallback(onLogEvent);
    
    // Register an additional callback using a lambda expression
    callback_test.registerSensorCallback([](float x, float y, float z) {
        // Create a simple anomaly detector
        if (std::abs(x) > 10 || std::abs(y) > 10 || std::abs(z) > 10) {
            std::cout << "Anomaly detector: Abnormal value detected!" << std::endl;
        }
    });
    
    // Simulate some sensor data and events
    std::cout << "\n===== Start simulating data =====\n" << std::endl;
    
    // Simulate normal sensor data
    callback_test.simulateSensorData(1.5f, 2.3f, 0.8f);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Simulate abnormal sensor data
    callback_test.simulateSensorData(15.2f, -3.7f, 0.5f);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Simulate system event
    callback_test.simulateEvent("System Startup", 100);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Simulate alert event
    callback_test.simulateEvent("Low Battery", 30);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    callback_test.simulateEvent("Sensor Error", 80);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Clear callbacks and confirm
    std::cout << "\nClearing all callbacks..." << std::endl;
    callback_test.clearCallbacks();
    
    // Try triggering callbacks (should have no output)
    callback_test.simulateSensorData(1.0f, 1.0f, 1.0f);
    callback_test.simulateEvent("Test Event", 50);
    
    std::cout << "\n===== Callback test completed =====\n" << std::endl;
    
    return 0;
}
