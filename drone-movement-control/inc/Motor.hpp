#ifndef __MOTOR_HPP
#define __MOTOR_HPP

#include "System.hpp"

// Motor control pin definitions
#define PWMA   TIM1->CCR1  // Left motor PWM output channel
#define PWMB   TIM1->CCR4  // Right motor PWM output channel
#define AIN1   PBout(15)   // Left motor direction control 1
#define AIN2   PBout(14)   // Left motor direction control 2
#define BIN1   PBout(13)   // Right motor direction control 1
#define BIN2   PBout(12)   // Right motor direction control 2

class Motor {
public:
    // Initialize motor pins
    static void init();
    
    // Initialize PWM
    // arr: auto-reload value
    // psc: prescaler
    static void pwmInit(uint16_t arr, uint16_t psc);
    
    // Left motor forward
    static void leftForward();
    
    // Left motor backward
    static void leftBackward();
    
    // Right motor forward
    static void rightForward();
    
    // Right motor backward
    static void rightBackward();
    
    // Set PWM value for left motor
    static void setLeftPwm(int pwm);
    
    // Set PWM value for right motor
    static void setRightPwm(int pwm);
    
    // Set PWM values for both motors
    static void setPwm(int left_pwm, int right_pwm);
    
    // Stop both motors
    static void stop();
};

#endif // __MOTOR_HPP
