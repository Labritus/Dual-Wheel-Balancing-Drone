#include "Balance.hpp"
#include "Delay.hpp"
#include "Show.hpp"
#include <cstdio>
// Removed STM32-specific includes: USART, EXTI, USART3 

// Initialize static members
BalanceCallback Balance::m_callback = nullptr;
std::atomic<bool> Balance::initialized_{false};
std::atomic<bool> Balance::running_{false};
std::atomic<float> Balance::current_angle_{0.0f};
std::atomic<float> Balance::current_gyro_{0.0f};
std::atomic<int> Balance::left_speed_{0};
std::atomic<int> Balance::right_speed_{0};
uint64_t Balance::last_callback_time_ = 0;

// Initialize the balance car (non-blocking)
bool Balance::init()
{
    if (initialized_.load()) {
        return true;
    }
    
    // Initialize system components
    if (!System::init()) {
        return false;
    }
    
    // Raspberry Pi compatible initialization
    LED1::init();                        // Initialize LED (using GPIOHelper)
    Key::init();                         // Initialize buttons (using GPIOHelper)
    OLED::init();                        // Initialize OLED (using I2CHelper)
    Motor::init();                       // Initialize motor control (using Linux PWM)
    // TODO: Initialize encoder interfaces for Raspberry Pi
    // TODO: Initialize ADC for battery monitoring
    IOI2C::init();                       // Initialize I2C communication
    MPU6050::initialize();
    
    static Control control;
    // TODO: Register sensor callback when MPU6050 supports getInstance()
    // MPU6050::getInstance().registerSensorCallback(&control);             
    MPU6050::dmpInit();                  // Initialize DMP     
    // TODO: Initialize ultrasonic sensor for Raspberry Pi
    // TODO: Setup MPU6050 interrupt handling for Raspberry Pi
    
    initialized_.store(true);
    running_.store(true);
    
    printf("Balance system initialized successfully\n");
    return true;
}

// Process one iteration - real-time safe
void Balance::processOnce()
{
    if (!initialized_.load() || !running_.load()) {
        return;
    }
    
    // Fast sensor data update
    updateSensorData();
    
    // Execute control loop
    executeControlLoop();
    
    // Invoke callback if needed (rate-limited)
    invokeCallback();
}

void Balance::updateSensorData()
{
    // Update atomic sensor values
    // TODO: Get actual angle and gyro values from Control class
    current_angle_.store(Control::getAngleBalance());
    current_gyro_.store(Control::getGyroBalance());
    // TODO: Implement encoder reading for Raspberry Pi
    // left_speed_.store(readLeftEncoder());
    // right_speed_.store(readRightEncoder());
    left_speed_.store(0);  // Default values for now
    right_speed_.store(0);
}

void Balance::executeControlLoop()
{
    // Fast control processing - all blocking delays removed
    // Motor control and balance logic happens here
    if(Control::getFlagStop() == 1) {
        Motor::setPwm(Control::getMotorLeft(), Control::getMotorRight());
    }
}

void Balance::invokeCallback()
{
    if (!m_callback) return;
    
    uint64_t current_time = Timer::getMicroseconds();
    // Rate limit callbacks to prevent overwhelming
    if (current_time - last_callback_time_ > 10000) {  // 10ms minimum interval
        m_callback(current_angle_.load(), current_gyro_.load(), 
                  left_speed_.load(), right_speed_.load());
        last_callback_time_ = current_time;
    }
}

// Legacy run function - DEPRECATED
void Balance::run()
{
    printf("Warning: Using deprecated blocking run() method\r\n");
    
    while(running_.load())
    {     
        processOnce();
        
        if(Control::getFlagShow() == 0)  // Use MiniBalance APP and OLED display
        {
            Show::appShow();   
            Show::oledShow();            // Display on OLED
        }
        else                             // Use MiniBalance upper computer
        {
            Show::dataScope();           // Send data to MiniBalance upper computer
        }
        
        // Non-blocking yield to prevent CPU starvation of other threads
        std::this_thread::yield();
        
        // DEPRECATED WARNING: This function should not be used in real-time systems
        // Use event-driven callbacks instead of blocking loops
    }
}

// Set callback function
void Balance::setCallback(BalanceCallback callback)
{
    m_callback = callback;
}

// Legacy handleCallback - kept for compatibility
void Balance::handleCallback()
{
    invokeCallback();
}