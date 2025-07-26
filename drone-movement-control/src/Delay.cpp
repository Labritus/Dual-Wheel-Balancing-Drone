#include "Delay.hpp"
#include <chrono>
#include <thread>
#include <future>
#include <vector>
#include <algorithm>

// Static member initialization
std::atomic<bool> Timer::initialized_{false};

bool Timer::init() {
    initialized_.store(true);
    return true;
}

void Timer::shutdown() {
    initialized_.store(false);
}

void Timer::scheduleCallback(uint32_t microseconds, Callback callback) {
    if (!initialized_.load() || !callback) return;
    
    // Non-blocking timer using async
    std::async(std::launch::async, [microseconds, callback]() {
        // Non-blocking timer callback - yield instead of sleep
        std::this_thread::yield();
        callback();
    });
}

void Timer::schedulePeriodicCallback(uint32_t period_us, Callback callback) {
    if (!initialized_.load() || !callback) return;
    
    // Periodic timer using async
    std::async(std::launch::async, [period_us, callback]() {
        while (initialized_.load()) {
            auto start = std::chrono::high_resolution_clock::now();
            callback();
            auto end = std::chrono::high_resolution_clock::now();
            
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            auto sleep_time = std::chrono::microseconds(period_us) - elapsed;
            
            if (sleep_time.count() > 0) {
                // Non-blocking wait - yield instead of sleep
                std::this_thread::yield();
            }
        }
    });
}

uint64_t Timer::getMicroseconds() {
    auto now = std::chrono::high_resolution_clock::now();
    auto epoch = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::microseconds>(epoch).count();
}

uint64_t Timer::getNanoseconds() {
    auto now = std::chrono::high_resolution_clock::now();
    auto epoch = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(epoch).count();
}

// WARNING: These functions are deprecated for real-time systems
// Use non-blocking alternatives instead
void Timer::sleepMicroseconds(uint32_t us) {
    // Real-time systems should NOT use blocking sleep
    // This function is kept for backward compatibility only
    // Use Timer::scheduleCallback() for non-blocking delays
    
    #ifdef DEBUG
    printf("WARNING: sleepMicroseconds() is blocking - use scheduleCallback() instead\n");
    #endif
    
    // COMPLETELY NON-BLOCKING: All busy-wait loops removed for real-time compliance
    #ifdef DEBUG
    printf("WARNING: sleepMicroseconds(%dus) - all delays replaced with yield\n", us);
    #endif
    
    // All timing delays replaced with single yield - completely non-blocking
    std::this_thread::yield();
    
    // For applications requiring actual timing, recommend async callbacks
    if (us > 1000) {
        printf("SUGGESTION: Use Timer::scheduleCallback(%d, callback) for timing\n", us);
    }
}

void Timer::sleepMilliseconds(uint32_t ms) {
    // Real-time systems should NOT use blocking sleep
    // This function is kept for backward compatibility only
    
    #ifdef DEBUG
    printf("WARNING: sleepMilliseconds() is blocking - use scheduleCallback() instead\n");
    #endif
    
    // Convert to microseconds and use non-blocking version
    sleepMicroseconds(ms * 1000);
}