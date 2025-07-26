#include "LatencyMonitor.hpp"
#include "USART.hpp"
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>

LatencyMonitor::LatencyMonitor(const std::string& name, uint64_t max_latency_us)
    : name_(name), max_allowed_latency_us_(max_latency_us) {
    recent_latencies_.reserve(MAX_RECENT_SAMPLES);
}

LatencyMonitor::~LatencyMonitor() = default;

void LatencyMonitor::startMeasurement() {
    if (!enabled_.load()) return;
    start_time_ = std::chrono::high_resolution_clock::now();
}

void LatencyMonitor::endMeasurement() {
    if (!enabled_.load()) return;
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto latency_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time_);
    uint64_t latency_us = latency_ns.count() / 1000;
    
    recordLatency(latency_us);
}

void LatencyMonitor::recordLatency(uint64_t latency_us) {
    // Update statistics atomically
    measurement_count_.fetch_add(1);
    total_latency_.fetch_add(latency_us);
    
    // Update min/max
    uint64_t current_max = max_latency_.load();
    while (latency_us > current_max && 
           !max_latency_.compare_exchange_weak(current_max, latency_us)) {
        // Retry if compare_exchange failed
    }
    
    uint64_t current_min = min_latency_.load();
    while (latency_us < current_min && 
           !min_latency_.compare_exchange_weak(current_min, latency_us)) {
        // Retry if compare_exchange failed
    }
    
    // Store recent latency for detailed analysis
    {
        std::lock_guard<std::mutex> lock(latencies_mutex_);
        if (recent_latencies_.size() >= MAX_RECENT_SAMPLES) {
            recent_latencies_.erase(recent_latencies_.begin());
        }
        recent_latencies_.push_back(latency_us);
    }
    
    // Check for violation
    if (latency_us > max_allowed_latency_us_) {
        violation_count_.fetch_add(1);
        
        if (violation_callback_) {
            violation_callback_(name_, latency_us);
        }
    }
}

uint64_t LatencyMonitor::getAvgLatency() const {
    uint64_t count = measurement_count_.load();
    if (count == 0) return 0;
    return total_latency_.load() / count;
}

void LatencyMonitor::setViolationCallback(LatencyCallback callback) {
    violation_callback_ = callback;
}

std::string LatencyMonitor::generateReport() const {
    std::ostringstream oss;
    
    uint64_t count = measurement_count_.load();
    if (count == 0) {
        oss << "[" << name_ << "] No measurements recorded\n";
        return oss.str();
    }
    
    uint64_t max_lat = max_latency_.load();
    uint64_t min_lat = min_latency_.load();
    uint64_t avg_lat = getAvgLatency();
    uint64_t violations = violation_count_.load();
    
    oss << "[" << name_ << "] Latency Report:\n";
    oss << "  Measurements: " << count << "\n";
    oss << "  Max Latency: " << max_lat << " µs\n";
    oss << "  Min Latency: " << min_lat << " µs\n";
    oss << "  Avg Latency: " << avg_lat << " µs\n";
    oss << "  Violations: " << violations << " (" 
        << std::fixed << std::setprecision(2) 
        << (100.0 * violations / count) << "%)\n";
    oss << "  Threshold: " << max_allowed_latency_us_ << " µs\n";
    
    // Calculate percentiles from recent samples
    {
        std::lock_guard<std::mutex> lock(latencies_mutex_);
        if (!recent_latencies_.empty()) {
            std::vector<uint64_t> sorted_latencies = recent_latencies_;
            std::sort(sorted_latencies.begin(), sorted_latencies.end());
            
            size_t size = sorted_latencies.size();
            oss << "  95th Percentile: " << sorted_latencies[size * 95 / 100] << " µs\n";
            oss << "  99th Percentile: " << sorted_latencies[size * 99 / 100] << " µs\n";
        }
    }
    
    return oss.str();
}

void LatencyMonitor::reset() {
    max_latency_.store(0);
    min_latency_.store(UINT64_MAX);
    total_latency_.store(0);
    measurement_count_.store(0);
    violation_count_.store(0);
    
    std::lock_guard<std::mutex> lock(latencies_mutex_);
    recent_latencies_.clear();
}

// Static member definitions for SystemLatencyMonitors
std::unique_ptr<LatencyMonitor> SystemLatencyMonitors::balance_control_monitor_;
std::unique_ptr<LatencyMonitor> SystemLatencyMonitors::sensor_read_monitor_;
std::unique_ptr<LatencyMonitor> SystemLatencyMonitors::motor_control_monitor_;
std::unique_ptr<LatencyMonitor> SystemLatencyMonitors::callback_monitor_;

LatencyMonitor& SystemLatencyMonitors::getBalanceControlMonitor() {
    if (!balance_control_monitor_) {
        balance_control_monitor_ = std::make_unique<LatencyMonitor>("Balance Control", 5000);  // 5ms max
    }
    return *balance_control_monitor_;
}

LatencyMonitor& SystemLatencyMonitors::getSensorReadMonitor() {
    if (!sensor_read_monitor_) {
        sensor_read_monitor_ = std::make_unique<LatencyMonitor>("Sensor Read", 1000);  // 1ms max
    }
    return *sensor_read_monitor_;
}

LatencyMonitor& SystemLatencyMonitors::getMotorControlMonitor() {
    if (!motor_control_monitor_) {
        motor_control_monitor_ = std::make_unique<LatencyMonitor>("Motor Control", 2000);  // 2ms max
    }
    return *motor_control_monitor_;
}

LatencyMonitor& SystemLatencyMonitors::getCallbackMonitor() {
    if (!callback_monitor_) {
        callback_monitor_ = std::make_unique<LatencyMonitor>("Callback Processing", 1000);  // 1ms max
    }
    return *callback_monitor_;
}

std::string SystemLatencyMonitors::generateSystemReport() {
    std::ostringstream oss;
    oss << "=== SYSTEM LATENCY REPORT ===\n";
    
    if (balance_control_monitor_) {
        oss << balance_control_monitor_->generateReport() << "\n";
    }
    
    if (sensor_read_monitor_) {
        oss << sensor_read_monitor_->generateReport() << "\n";
    }
    
    if (motor_control_monitor_) {
        oss << motor_control_monitor_->generateReport() << "\n";
    }
    
    if (callback_monitor_) {
        oss << callback_monitor_->generateReport() << "\n";
    }
    
    oss << "=============================\n";
    return oss.str();
}

void SystemLatencyMonitors::setupViolationCallbacks() {
    auto violation_handler = [](const std::string& component, uint64_t latency_us) {
        USART::printf("LATENCY VIOLATION: %s took %llu µs\r\n", 
                     component.c_str(), latency_us);
    };
    
    getBalanceControlMonitor().setViolationCallback(violation_handler);
    getSensorReadMonitor().setViolationCallback(violation_handler);
    getMotorControlMonitor().setViolationCallback(violation_handler);
    getCallbackMonitor().setViolationCallback(violation_handler);
}