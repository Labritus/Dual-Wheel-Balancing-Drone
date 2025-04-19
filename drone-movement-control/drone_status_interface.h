#ifndef DRONE_STATUS_INTERFACE_H
#define DRONE_STATUS_INTERFACE_H

#include <functional>
#include <string>
#include <vector>
#include <mutex>
#include <queue>
#include <fstream>
#include "mpu6050.h"

// Define the drone status data structure
struct DroneStatus {
    float roll;           // Roll angle
    float pitch;          // Pitch angle
    float yaw;            // Yaw angle
    int battery_voltage;  // Battery voltage (mV)
    int motor_left;       // Left motor PWM
    int motor_right;      // Right motor PWM
    int position;         // Current position
    int velocity;         // Current velocity
    bool position_mode;   // Mode: true = position mode, false = velocity mode
    std::string status;   // Status description
    std::string status_description; // Detailed status description
    int motor_left_pwm;   // Left motor PWM
    int motor_right_pwm;  // Right motor PWM
};

// Define status export callback type
using StatusExportCallback = std::function<void(const DroneStatus& status)>;

class DroneStatusInterface {
public:
    static DroneStatusInterface& getInstance() {
        static DroneStatusInterface instance;
        return instance;
    }
    
    // Get current status
    DroneStatus getStatus() const;
    
    // Register status export callback
    void registerExportCallback(StatusExportCallback callback);
    
    // Update status and trigger callbacks
    void updateStatus(const DroneStatus& status);
    
    // Add event to event queue
    void addEvent(const std::string& event_name, const std::string& event_data);
    
    // Start/stop CSV recording
    bool startCSVRecording(const std::string& filename);
    void stopCSVRecording();
    
    // Start/stop JSON recording
    bool startJSONRecording(const std::string& filename);
    void stopJSONRecording();
    
private:
    DroneStatusInterface();
    ~DroneStatusInterface();
    
    // Disable copy and assignment
    DroneStatusInterface(const DroneStatusInterface&) = delete;
    DroneStatusInterface& operator=(const DroneStatusInterface&) = delete;
    
    // Format status as CSV
    std::string formatStatusCSV(const DroneStatus& status);
    
    // Format status as JSON
    std::string formatStatusJSON(const DroneStatus& status);
    
    // Export callbacks
    std::vector<StatusExportCallback> export_callbacks_;
    
    // Thread safety
    std::mutex status_mutex_;
    std::mutex event_mutex_;
    
    // Current status
    DroneStatus current_status_;
    
    // Event queue
    struct Event {
        std::string name;
        std::string data;
        std::chrono::system_clock::time_point timestamp;
    };
    std::queue<Event> event_queue_;
    
    // CSV file recording
    std::ofstream csv_file_;
    bool csv_header_written_;
    
    // JSON file recording
    std::ofstream json_file_;
    bool json_first_entry_;
};

#endif // DRONE_STATUS_INTERFACE_H
