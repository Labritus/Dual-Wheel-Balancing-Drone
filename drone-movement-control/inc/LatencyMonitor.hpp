#ifndef __LATENCY_MONITOR_HPP
#define __LATENCY_MONITOR_HPP

#include <chrono>
#include <vector>
#include <mutex>
#include <atomic>
#include <string>
#include <functional>
#include <memory>

class LatencyMonitor {
public:
    using LatencyCallback = std::function<void(const std::string&, uint64_t)>;
    
    // Constructor with latency threshold in microseconds
    LatencyMonitor(const std::string& name, uint64_t max_latency_us = 5000);
    
    // Destructor
    ~LatencyMonitor();
    
    // Start measurement
    void startMeasurement();
    
    // End measurement and record latency
    void endMeasurement();
    
    // Get statistics
    uint64_t getMaxLatency() const { return max_latency_.load(); }
    uint64_t getMinLatency() const { return min_latency_.load(); }
    uint64_t getAvgLatency() const;
    uint64_t getLatencyCount() const { return measurement_count_.load(); }
    uint64_t getMeasurementCount() const { return measurement_count_.load(); }
    uint64_t getViolationCount() const { return violation_count_.load(); }
    
    // Set callback for latency violations
    void setViolationCallback(LatencyCallback callback);
    
    // Generate detailed report
    std::string generateReport() const;
    
    // Reset statistics
    void reset();
    
    // Enable/disable monitoring
    void enable(bool enabled) { enabled_.store(enabled); }
    bool isEnabled() const { return enabled_.load(); }
    
private:
    std::string name_;
    uint64_t max_allowed_latency_us_;
    std::chrono::high_resolution_clock::time_point start_time_;
    
    // Statistics (atomic for thread safety)
    std::atomic<uint64_t> max_latency_{0};
    std::atomic<uint64_t> min_latency_{UINT64_MAX};
    std::atomic<uint64_t> total_latency_{0};
    std::atomic<uint64_t> measurement_count_{0};
    std::atomic<uint64_t> violation_count_{0};
    std::atomic<bool> enabled_{true};
    
    // Recent latencies for detailed analysis
    mutable std::mutex latencies_mutex_;
    std::vector<uint64_t> recent_latencies_;
    static constexpr size_t MAX_RECENT_SAMPLES = 1000;
    
    LatencyCallback violation_callback_;
    
    void recordLatency(uint64_t latency_us);
};

// RAII helper for automatic latency measurement
class ScopedLatencyMeasurement {
public:
    explicit ScopedLatencyMeasurement(LatencyMonitor& monitor) 
        : monitor_(monitor) {
        monitor_.startMeasurement();
    }
    
    ~ScopedLatencyMeasurement() {
        monitor_.endMeasurement();
    }
    
private:
    LatencyMonitor& monitor_;
};

// Macro for easy scoped measurement
#define MEASURE_LATENCY(monitor) ScopedLatencyMeasurement _scoped_measurement(monitor)

// Global latency monitors for critical system components
class SystemLatencyMonitors {
public:
    static LatencyMonitor& getBalanceControlMonitor();
    static LatencyMonitor& getSensorReadMonitor();
    static LatencyMonitor& getMotorControlMonitor();
    static LatencyMonitor& getCallbackMonitor();
    
    // Generate system-wide latency report
    static std::string generateSystemReport();
    
    // Setup violation callbacks for all monitors
    static void setupViolationCallbacks();
    
private:
    static std::unique_ptr<LatencyMonitor> balance_control_monitor_;
    static std::unique_ptr<LatencyMonitor> sensor_read_monitor_;
    static std::unique_ptr<LatencyMonitor> motor_control_monitor_;
    static std::unique_ptr<LatencyMonitor> callback_monitor_;
};

#endif // __LATENCY_MONITOR_HPP