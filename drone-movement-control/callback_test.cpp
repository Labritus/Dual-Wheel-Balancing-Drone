#include "../include/callback_test.h"
#include <iostream>

CallbackTest::CallbackTest() {
    std::cout << "Callback test initialized" << std::endl;
}

CallbackTest::~CallbackTest() {
    clearCallbacks();
}

void CallbackTest::registerSensorCallback(SensorDataCallback callback) {
    sensor_callbacks_.push_back(callback);
}

void CallbackTest::registerEventCallback(EventCallback callback) {
    event_callbacks_.push_back(callback);
}

void CallbackTest::simulateSensorData(float x, float y, float z) {
    std::cout << "Generated sensor data: (" << x << ", " << y << ", " << z << ")" << std::endl;
    
    // Notify all registered sensor callbacks
    for (const auto& callback : sensor_callbacks_) {
        callback(x, y, z);
    }
}

void CallbackTest::simulateEvent(const std::string& event_name, int event_data) {
    std::cout << "Generated event: " << event_name << " Data: " << event_data << std::endl;
    
    // Notify all registered event callbacks
    for (const auto& callback : event_callbacks_) {
        callback(event_name, event_data);
    }
}

void CallbackTest::clearCallbacks() {
    sensor_callbacks_.clear();
    event_callbacks_.clear();
}
