#ifndef ENCODER_H
#define ENCODER_H

#include "gpio_interface.h"
#include <thread>
#include <atomic>
#include <chrono>

class Encoder {
public:
    Encoder();
    ~Encoder();
    
    // Initialize the encoder
    bool initialize(int pin_a, int pin_b);
    
    // Read the current count value
    int getCount() const;
    
    // Read the current velocity (counts per second)
    int getVelocity() const;
    
    // Reset the counter
    void resetCount();
    
    // Start counting
    void start();
    
    // Stop counting
    void stop();
    
    // Check if the encoder is running
    bool isRunning() const;
    
private:
    // Pin numbers
    int pin_a_;
    int pin_b_;
    
    // GPIO interface
    GPIOInterface gpio_;
    
    // Counter and velocity
    std::atomic<int> count_;
    std::atomic<int> velocity_;
    
    // Running status
    std::atomic<bool> running_;
    
    // Last states
    int last_state_a_;
    int last_state_b_;
    
    // Timestamp for velocity calculation
    std::chrono::steady_clock::time_point last_time_;
    
    // Monitoring thread
    std::thread monitor_thread_;
    
    // Monitoring function
    void monitorPins();
};

#endif // ENCODER_H
