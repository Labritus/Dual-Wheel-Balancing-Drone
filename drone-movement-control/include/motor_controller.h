#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "gpio_interface.h"
#include <cstdint>

class MotorController {
public:
    MotorController();
    ~MotorController();
    
    // Initialize the motor controller
    bool initialize();
    
    // Set the motor PWM values
    bool setPWM(int motor_left, int motor_right);
    
    // Stop the motors
    void stop();
    
    // Check the controller status
    bool isInitialized() const;
    
private:
    GPIOInterface gpio_;
    
    // Limit the PWM value within the allowed range
    int limitPWM(int pwm, int max_value, int min_value);
    
    // Motor pin status flag
    bool initialized_;
};

#endif // MOTOR_CONTROLLER_H
