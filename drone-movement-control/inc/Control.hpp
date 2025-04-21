#ifndef __CONTROL_HPP
#define __CONTROL_HPP

#include "System.hpp"

#define PI 3.14159265              // PI constant
#define Control_Frequency  200.0   // Control frequency / sampling frequency
#define Diameter_67  67.0          // Wheel diameter: 67 mm
#define EncoderMultiples   4.0     // Encoder multiplier
#define Encoder_precision  13.0    // Encoder resolution: 13 lines
#define Reduction_Ratio  30.0      // Gear reduction ratio: 30
#define Perimeter  210.4867        // Wheel circumference in mm

#define Middle_angle 0
#define DIFFERENCE 100

class Control {
public:
    // External interrupt handler
    static int exti15_10_IRQHandler(void);
    
    // Balance control
    static int balance(float angle, float gyro);
    
    // Velocity control
    static int velocity(int encoder_left, int encoder_right);
    
    // Turn control
    static int turn(float gyro);
    
    // Set PWM for motors
    static void setPwm(int motor_left, int motor_right);
    
    // Key press handling
    static void key(void);
    
    // Limit PWM output
    static int pwmLimit(int input, int max, int min);
    
    // Turn off motors based on conditions
    static uint8_t turnOff(float angle, int voltage);
    
    // Get angle from sensor
    static void getAngle(uint8_t way);
    
    // Absolute value
    static int myAbs(int a);
    
    // Pick-up detection
    static int pickUp(float acceleration, float angle, int encoder_left, int encoder_right);
    
    // Put-down detection
    static int putDown(float angle, int encoder_left, int encoder_right);
    
    // Compute speed from encoder values
    static void getVelocityFromEncoder(int encoder_left, int encoder_right);
    
    // Mode selection logic
    static void choose(int encoder_left, int encoder_right);
};

// Interrupt handler function declaration
extern "C" {
    int EXTI15_10_IRQHandler(void);
}

#endif // __CONTROL_HPP
