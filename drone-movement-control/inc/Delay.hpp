#ifndef __DELAY_HPP
#define __DELAY_HPP

#include <stdint.h>
#include <functional>
#include <atomic>

// Timer and delay functionality class
class Timer {
public:
    using Callback = std::function<void()>;
    
    // Initialize timer system
    static bool init();
    
    // Shutdown timer system
    static void shutdown();
    
    // Schedule a one-time callback
    static void scheduleCallback(uint32_t microseconds, Callback callback);
    
    // Schedule a periodic callback
    static void schedulePeriodicCallback(uint32_t period_us, Callback callback);
    
    // Get current time in microseconds
    static uint64_t getMicroseconds();
    
    // Get current time in nanoseconds
    static uint64_t getNanoseconds();
    
    // Sleep for specified microseconds
    static void sleepMicroseconds(uint32_t us);
    
    // Sleep for specified milliseconds
    static void sleepMilliseconds(uint32_t ms);
    
    // Check if initialized
    static bool isInitialized() { return initialized_.load(); }
    
private:
    static std::atomic<bool> initialized_;
};

// Convenience delay functions
namespace Delay {
    // Delay in microseconds
    inline void us(uint32_t microseconds) {
        Timer::sleepMicroseconds(microseconds);
    }
    
    // Delay in milliseconds
    inline void ms(uint32_t milliseconds) {
        Timer::sleepMilliseconds(milliseconds);
    }
}

#endif // __DELAY_HPP