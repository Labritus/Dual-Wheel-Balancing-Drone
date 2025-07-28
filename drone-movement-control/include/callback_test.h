#ifndef CALLBACK_TEST_H
#define CALLBACK_TEST_H

#include <functional>
#include <string>
#include <vector>

// Define callback function types
using SensorDataCallback = std::function<void(float x, float y, float z)>;
using EventCallback = std::function<void(const std::string& event_name, int event_data)>;

class CallbackTest {
public:
    CallbackTest();
    ~CallbackTest();
    
    // Register sensor data callback
    void registerSensorCallback(SensorDataCallback callback);
    
    // Register event callback
    void registerEventCallback(EventCallback callback);
    
    // Simulate generating sensor data and events
    void simulateSensorData(float x, float y, float z);
    void simulateEvent(const std::string& event_name, int event_data);
    
    // Remove all callbacks
    void clearCallbacks();
    
private:
    // Store callback functions
    std::vector<SensorDataCallback> sensor_callbacks_;
    std::vector<EventCallback> event_callbacks_;
};

#endif // CALLBACK_TEST_H