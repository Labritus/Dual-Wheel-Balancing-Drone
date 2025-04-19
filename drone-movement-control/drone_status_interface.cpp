#include "../include/drone_status_interface.h"
#include <iostream>
#include <chrono>
#include <iomanip>
#include <sstream>

DroneStatusInterface::DroneStatusInterface() : 
    csv_header_written_(false),
    json_first_entry_(true)
{
    // Initialize current status
    current_status_.roll = 0.0f;
    current_status_.pitch = 0.0f;
    current_status_.yaw = 0.0f;
    current_status_.battery_voltage = 0;
    current_status_.motor_left = 0;
    current_status_.motor_right = 0;
    current_status_.position = 0;
    current_status_.velocity = 0;
    current_status_.position_mode = true;
    current_status_.status = "Initialized";
    current_status_.status_description = "";
    current_status_.motor_left_pwm = 0;
    current_status_.motor_right_pwm = 0;
}

DroneStatusInterface::~DroneStatusInterface() {
    stopCSVRecording();
    stopJSONRecording();
}

DroneStatus DroneStatusInterface::getStatus() const {
    return current_status_;
}

void DroneStatusInterface::registerExportCallback(StatusExportCallback callback) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    export_callbacks_.push_back(callback);
}

void DroneStatusInterface::updateStatus(const DroneStatus& status) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    
    // Update current status
    current_status_ = status;
    
    // Invoke all registered callbacks
    for (const auto& callback : export_callbacks_) {
        callback(status);
    }
    
    // Record to CSV if file is open
    if (csv_file_.is_open()) {
        // Write header if not yet written
        if (!csv_header_written_) {
            csv_file_ << "Timestamp,Roll,Pitch,Yaw,BatteryVoltage,MotorLeft,MotorRight,Position,Velocity,PositionMode,Status\n";
            csv_header_written_ = true;
        }
        
        csv_file_ << formatStatusCSV(status) << std::endl;
    }
    
    // Record to JSON if file is open
    if (json_file_.is_open()) {
        if (json_first_entry_) {
            json_file_ << "[\n";
            json_first_entry_ = false;
        } else {
            json_file_ << ",\n";
        }
        
        json_file_ << formatStatusJSON(status);
    }
}

void DroneStatusInterface::addEvent(const std::string& event_name, const std::string& event_data) {
    std::lock_guard<std::mutex> lock(event_mutex_);
    
    Event event;
    event.name = event_name;
    event.data = event_data;
    event.timestamp = std::chrono::system_clock::now();
    
    event_queue_.push(event);
    
    // Record event to JSON if file is open
    if (json_file_.is_open()) {
        if (json_first_entry_) {
            json_file_ << "[\n";
            json_first_entry_ = false;
        } else {
            json_file_ << ",\n";
        }
        
        // Get current time
        auto now = std::chrono::system_clock::now();
        auto now_time_t = std::chrono::system_clock::to_time_t(now);
        auto now_tm = std::localtime(&now_time_t);
        
        std::stringstream ss;
        ss << std::put_time(now_tm, "%Y-%m-%d %H:%M:%S");
        
        json_file_ << "{\n"
                   << "  \"type\": \"event\",\n"
                   << "  \"timestamp\": \"" << ss.str() << "\",\n"
                   << "  \"name\": \"" << event_name << "\",\n"
                   << "  \"data\": \"" << event_data << "\"\n"
                   << "}";
    }
}

bool DroneStatusInterface::startCSVRecording(const std::string& filename) {
    stopCSVRecording(); // Close existing file
    
    csv_file_.open(filename);
    if (!csv_file_.is_open()) {
        std::cerr << "Failed to open CSV file: " << filename << std::endl;
        return false;
    }
    
    csv_header_written_ = false;
    return true;
}

void DroneStatusInterface::stopCSVRecording() {
    if (csv_file_.is_open()) {
        csv_file_.close();
    }
}

bool DroneStatusInterface::startJSONRecording(const std::string& filename) {
    stopJSONRecording(); // Close existing file
    
    json_file_.open(filename);
    if (!json_file_.is_open()) {
        std::cerr << "Failed to open JSON file: " << filename << std::endl;
        return false;
    }
    
    json_first_entry_ = true;
    return true;
}

void DroneStatusInterface::stopJSONRecording() {
    if (json_file_.is_open()) {
        if (!json_first_entry_) {
            json_file_ << "\n]";
        } else {
            json_file_ << "[]";
        }
        json_file_.close();
    }
}

std::string DroneStatusInterface::formatStatusCSV(const DroneStatus& status) {
    // Get current time
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    auto now_tm = std::localtime(&now_time_t);
    
    std::stringstream ss;
    ss << std::put_time(now_tm, "%Y-%m-%d %H:%M:%S") << ",";
    ss << status.roll << ",";
    ss << status.pitch << ",";
    ss << status.yaw << ",";
    ss << status.battery_voltage << ",";
    ss << status.motor_left << ",";
    ss << status.motor_right << ",";
    ss << status.position << ",";
    ss << status.velocity << ",";
    ss << (status.position_mode ? "Position" : "Velocity") << ",";
    ss << status.status;
    
    return ss.str();
}

std::string DroneStatusInterface::formatStatusJSON(const DroneStatus& status) {
    // Get current time
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    auto now_tm = std::localtime(&now_time_t);
    
    std::stringstream ss;
    ss << std::put_time(now_tm, "%Y-%m-%d %H:%M:%S");
    
    std::stringstream json;
    json << "{\n"
         << "  \"type\": \"status\",\n"
         << "  \"timestamp\": \"" << ss.str() << "\",\n"
         << "  \"data\": {\n"
         << "    \"roll\": " << status.roll << ",\n"
         << "    \"pitch\": " << status.pitch << ",\n"
         << "    \"yaw\": " << status.yaw << ",\n"
         << "    \"battery_voltage\": " << status.battery_voltage << ",\n"
         << "    \"motor_left\": " << status.motor_left << ",\n"
         << "    \"motor_right\": " << status.motor_right << ",\n"
         << "    \"motor_left_pwm\": " << status.motor_left_pwm << ",\n"
         << "    \"motor_right_pwm\": " << status.motor_right_pwm << ",\n"
         << "    \"position\": " << status.position << ",\n"
         << "    \"velocity\": " << status.velocity << ",\n"
         << "    \"position_mode\": " << (status.position_mode ? "true" : "false") << ",\n"
         << "    \"status\": \"" << status.status << "\",\n"
         << "    \"status_description\": \"" << status.status_description << "\"\n"
         << "  }\n"
         << "}";
    
    return json.str();
}
