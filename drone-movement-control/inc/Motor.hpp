#ifndef __MOTOR_HPP
#define __MOTOR_HPP

#include <stdint.h>

class Motor {
public:
    // Initialize motor control pins
    static void init();
    
    // Initialize PWM system
    static void pwmInit(uint16_t arr, uint16_t psc);
    
    // Set PWM for left motor
    static void setLeftPwm(int pwm);
    
    // Set PWM for right motor  
    static void setRightPwm(int pwm);
    
    // Set PWM for both motors
    static void setPwm(int left_pwm, int right_pwm);
    
    // Motor direction control
    static void leftForward();
    static void leftBackward();
    static void rightForward();
    static void rightBackward();
    
    // Stop motors
    static void stop();
    
private:
    // Initialize hardware PWM channel
    static void initHardwarePWM(int channel, uint16_t period, uint16_t prescaler);
    
    // Set PWM value for specific channel
    static void setPWMValue(int channel, int value);
    
    // PWM file descriptors
    static int pwm_fd[2];
    static bool initialized_;
};

#endif // __MOTOR_HPP