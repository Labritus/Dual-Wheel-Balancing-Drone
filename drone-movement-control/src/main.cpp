#include "Balance.hpp"
#include "MemoryManager.hpp"
#include "Delay.hpp"
#include "I2CSlave.hpp"
#include "System.hpp"
#include "RealTimeThread.hpp"
#include "Control.hpp"
#include "Show.hpp"
#include <chrono>
#include <thread>
#include <atomic>
#include <signal.h>
#include <cstdio> // For printf

// Real-time system control
static std::atomic<bool> system_running{true};

// Signal handler for graceful shutdown
void signalHandler(int signal) {
    system_running.store(false);
}

// Fast, non-blocking callback for balance data
void balanceStatusCallback(float angle, float gyro, int left_speed, int right_speed)
{
    // Fast processing only - no blocking operations
    static uint64_t last_print_time = 0;
    auto current_time = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    
    // Rate-limited printing (non-blocking)
    if (current_time - last_print_time > 500000) {  // 500ms
        printf("Angle: %.2f, Gyro: %.2f, Left Speed: %d, Right Speed: %d\n", 
               angle, gyro, left_speed, right_speed);
        last_print_time = current_time;
    }
    
    // Fast I2C message check
    if (I2CSlave::hasNewMessage()) {
        if (I2CSlave::isPersonDetected()) {
            Control::setFlagFollow(true);
            Control::setFlagAvoid(false);
        }
        I2CSlave::clearMessageFlag();
    }
}

int main(void)
{ 
    // Setup signal handlers for graceful shutdown
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    // Enable memory leak detection
    MemoryManager::getInstance().enableTracking(true);
    
    // Initialize system components
    if (!System::init()) {
        printf("Failed to initialize system\n");
        return -1;
    }
    
    // Timer initialization not needed for Raspberry Pi
    
    // Initialize the balance car
    Balance::init();
    
    // Initialize I2C slave with address 0x08
    I2CSlave::init();
    
    // Set status callback function
    Balance::setCallback(balanceStatusCallback);
    
    // Create real-time threads instead of blocking loop
    RealTimeThread balanceThread([]() {
        Balance::processOnce();  // Non-blocking process function
    }, 90, 5);  // High priority, 5ms period
    
    RealTimeThread displayThread([]() {
        if(Control::getFlagShow() == 0) {
            Show::appShow();   
            Show::oledShow();
        } else {
            Show::dataScope();
        }
    }, 50, 50);  // Lower priority, 50ms period
    
    // Start real-time threads
    balanceThread.start();
    displayThread.start();
    
    printf("Real-time balance system started\n");
    
    // Real-time main thread - event-driven system management
    auto last_health_check = std::chrono::steady_clock::now();
    auto last_status_print = std::chrono::steady_clock::now();
    
    while (system_running.load()) {
        auto current_time = std::chrono::steady_clock::now();
        
        // Non-blocking health monitoring (every 1000ms)
        if (current_time - last_health_check >= std::chrono::milliseconds(1000)) {
            if (MemoryManager::getInstance().hasLeaks()) {
                printf("Warning: Memory leak detected\n");
            }
            last_health_check = current_time;
        }
        
        // Non-blocking status updates (every 5000ms)
        if (current_time - last_status_print >= std::chrono::milliseconds(5000)) {
            printf("System status: Real-time threads active\n");
            last_status_print = current_time;
        }
        
        // Fast system event processing
        if (I2CSlave::hasNewMessage()) {
            // Fast message processing
            if (I2CSlave::isPersonDetected()) {
                Control::setFlagFollow(true);
                Control::setFlagAvoid(false);
            }
            I2CSlave::clearMessageFlag();
        }
        
        // Yield to real-time threads - no blocking sleep
        std::this_thread::yield();
    }
    
    // Graceful shutdown
    printf("Shutting down system...\n");
    balanceThread.stop();
    displayThread.stop();
    
    // Timer shutdown not needed for Raspberry Pi
    System::shutdown();
    
    // Final memory leak check
    if (MemoryManager::getInstance().hasLeaks()) {
        printf("Memory leaks detected on shutdown\n");
        return -1;
    }
    
    return 0;
}
