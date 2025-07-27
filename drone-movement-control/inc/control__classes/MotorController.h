#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <cstdint>

class MotorController {
private:
    // Motor pin definitions
    uint8_t leftMotorPin1;
    uint8_t leftMotorPin2;
    uint8_t rightMotorPin1;
    uint8_t rightMotorPin2;
    uint8_t leftPwmPin;
    uint8_t rightPwmPin;
    
    // Current speeds
    int16_t leftSpeed;
    int16_t rightSpeed;
    
    // PWM frequency for motor control
    uint32_t pwm_frequency;
    
    // Initialize GPIO pins
    void initPins();

public:
    // Constructor
    MotorController(uint8_t leftPin1, uint8_t leftPin2, uint8_t rightPin1, uint8_t rightPin2,
                   uint8_t leftPwm, uint8_t rightPwm);
    
    // Destructor
    ~MotorController();
    
    // Initialize motor controller
    bool initialize();
    
    // Set left/right motor speeds (-255 ~ 255)
    void setSpeed(int16_t left, int16_t right);
    
    // Stop all motors
    void stop();
    
    // Get current speeds
    int16_t getLeftSpeed() const { return leftSpeed; }
    int16_t getRightSpeed() const { return rightSpeed; }
    
    // Set PWM frequency for both motors
    void setPWMFrequency(uint32_t frequency) { pwm_frequency = frequency; }
    uint32_t getPWMFrequency() const { return pwm_frequency; }
};

#endif // MOTOR_CONTROLLER_H