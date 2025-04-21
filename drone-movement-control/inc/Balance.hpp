#ifndef __BALANCE_HPP
#define __BALANCE_HPP

#include "System.hpp"
#include "Control.hpp"
#include "Motor.hpp"
#include "Encoder.hpp"
#include "LED.hpp"
#include "MPU6050.hpp"
#include "OLED.hpp"
#include "Key.hpp"
#include "ADC.hpp"
#include "Ultrasonic.hpp"

// Callback function type definition
typedef void (*BalanceCallback)(float angle, float gyro, int left_speed, int right_speed);

class Balance {
public:
    // Initialize the self-balancing car
    static void init();
    
    // Run the self-balancing car
    static void run();
    
    // Set a callback function
    static void setCallback(BalanceCallback callback);
    
    // Invoke the callback if set
    static void handleCallback();
    
private:
    static BalanceCallback m_callback; // Callback function pointer
};

#endif // __BALANCE_HPP
