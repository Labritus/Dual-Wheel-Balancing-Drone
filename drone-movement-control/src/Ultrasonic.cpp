#include "Ultrasonic.hpp"
#include "Control.hpp"
#include "Delay.hpp"
#include "GPIOHelper.hpp"
#include <chrono>
#include <thread>
#include <atomic>
#include <memory>

// GPIO pin definitions for Raspberry Pi
#define TRIG_PIN 23  // GPIO 23 for TRIGGER
#define ECHO_PIN 24  // GPIO 24 for ECHO

// Non-blocking ultrasonic measurement state machine
enum class UltrasonicState {
    IDLE,
    TRIGGER_SENT,
    WAITING_FOR_ECHO_START,
    MEASURING_ECHO_DURATION,
    MEASUREMENT_COMPLETE,
    TIMEOUT_ERROR
};

// Static variables for non-blocking measurement
static std::atomic<UltrasonicState> current_state{UltrasonicState::IDLE};
static std::chrono::high_resolution_clock::time_point trigger_time;
static std::chrono::high_resolution_clock::time_point echo_start_time;
static std::chrono::high_resolution_clock::time_point echo_end_time;
static std::chrono::high_resolution_clock::time_point timeout_start;
static constexpr int TIMEOUT_US = 30000; // 30ms timeout for real-time compliance
static constexpr int TRIGGER_PULSE_US = 10; // 10us trigger pulse

// Forward declaration for the state machine function
static void processUltrasonicStateMachine();

// Initialize ultrasonic sensor
void Ultrasonic::init()
{
    // Initialize GPIO for ultrasonic sensor on Raspberry Pi
    if (!GPIOHelper::isInitialized()) {
        GPIOHelper::init();
    }
    
    // Configure GPIO pins for ultrasonic sensor
    GPIOHelper::setMode(TRIG_PIN, GPIOMode::OUTPUT);  // TRIG pin as output
    GPIOHelper::setMode(ECHO_PIN, GPIOMode::INPUT);   // ECHO pin as input
    
    // Initial state: TRIG pin low
    GPIOHelper::setValue(TRIG_PIN, GPIOValue::LOW);
    
    // Start periodic non-blocking measurement timer (every 50ms)
    Timer::schedulePeriodicCallback(50000, []() {
        processUltrasonicStateMachine();
    });
}

// Non-blocking state machine processing function
static void processUltrasonicStateMachine()
{
    auto current_time = std::chrono::high_resolution_clock::now();
    
    switch (current_state.load()) {
        case UltrasonicState::IDLE:
            // Start new measurement cycle
            GPIOHelper::setValue(TRIG_PIN, GPIOValue::HIGH);
            trigger_time = current_time;
            current_state.store(UltrasonicState::TRIGGER_SENT);
            
            // Schedule trigger pulse end using timer callback
            Timer::scheduleCallback(TRIGGER_PULSE_US, []() {
                GPIOHelper::setValue(TRIG_PIN, GPIOValue::LOW);
                timeout_start = std::chrono::high_resolution_clock::now();
                current_state.store(UltrasonicState::WAITING_FOR_ECHO_START);
            });
            break;
            
        case UltrasonicState::TRIGGER_SENT:
            // Waiting for trigger pulse timer callback - do nothing
            break;
            
        case UltrasonicState::WAITING_FOR_ECHO_START:
            // Non-blocking check for echo start
            if (GPIOHelper::getValue(ECHO_PIN) == GPIOValue::HIGH) {
                echo_start_time = current_time;
                current_state.store(UltrasonicState::MEASURING_ECHO_DURATION);
            } else {
                // Check for timeout
                auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
                    current_time - timeout_start).count();
                if (elapsed > TIMEOUT_US) {
                    current_state.store(UltrasonicState::TIMEOUT_ERROR);
                }
            }
            break;
            
        case UltrasonicState::MEASURING_ECHO_DURATION:
            // Non-blocking check for echo end
            if (GPIOHelper::getValue(ECHO_PIN) == GPIOValue::LOW) {
                echo_end_time = current_time;
                current_state.store(UltrasonicState::MEASUREMENT_COMPLETE);
            } else {
                // Check for timeout
                auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
                    current_time - echo_start_time).count();
                if (elapsed > TIMEOUT_US) {
                    current_state.store(UltrasonicState::TIMEOUT_ERROR);
                }
            }
            break;
            
        case UltrasonicState::MEASUREMENT_COMPLETE:
            // Calculate distance and report result
            {
                auto pulse_duration = std::chrono::duration_cast<std::chrono::microseconds>(
                    echo_end_time - echo_start_time);
                
                // Distance = (time * 343 * 1000) / 2000000 = time * 0.1715
                float distance_mm = (pulse_duration.count() * 343.0f) / 2000.0f;
                
                // Validate reasonable distance range (2mm to 4000mm for HC-SR04)
                if (distance_mm >= 2.0f && distance_mm <= 4000.0f) {
                    Control::setDistance(distance_mm);
                } else {
                    Control::setDistance(0); // Invalid measurement
                }
                
                current_state.store(UltrasonicState::IDLE);
            }
            break;
            
        case UltrasonicState::TIMEOUT_ERROR:
            // Report timeout error and reset
            Control::setDistance(0); // Set invalid distance
            current_state.store(UltrasonicState::IDLE);
            break;
    }
}

// Non-blocking distance measurement trigger
void Ultrasonic::readDistance()
{
    // For compatibility - the actual measurement is now handled by 
    // the periodic state machine. This function just ensures the 
    // state machine is running.
    if (current_state.load() == UltrasonicState::IDLE) {
        // Trigger immediate measurement by calling state machine
        processUltrasonicStateMachine();
    }
    // If measurement is already in progress, do nothing (non-blocking)
}
