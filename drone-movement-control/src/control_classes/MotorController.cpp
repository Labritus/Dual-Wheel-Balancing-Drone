#include "MotorController.h"
#include "../System.hpp"
#include <algorithm>

// Constructor
MotorController::MotorController(uint8_t leftPin1, uint8_t leftPin2, uint8_t rightPin1, uint8_t rightPin2,
                               uint8_t leftPwm, uint8_t rightPwm)
    : leftMotorPin1(leftPin1), leftMotorPin2(leftPin2), rightMotorPin1(rightPin1), rightMotorPin2(rightPin2),
      leftPwmPin(leftPwm), rightPwmPin(rightPwm), leftSpeed(0), rightSpeed(0) {}

// Destructor
MotorController::~MotorController() {
    stop();
}

// Initialize Raspberry Pi GPIO pins
void MotorController::initPins() {
    // Configure motor control pins as outputs
    System::setGPIOMode(leftMotorPin1, GPIO_OUTPUT);
    System::setGPIOMode(leftMotorPin2, GPIO_OUTPUT);
    System::setGPIOMode(rightMotorPin1, GPIO_OUTPUT);
    System::setGPIOMode(rightMotorPin2, GPIO_OUTPUT);
    
    // PWM pins will be configured by System::setPWM when needed
    System::setGPIOMode(leftPwmPin, GPIO_OUTPUT);
    System::setGPIOMode(rightPwmPin, GPIO_OUTPUT);
    
    // Initialize all pins to low
    System::setGPIOValue(leftMotorPin1, GPIO_LOW);
    System::setGPIOValue(leftMotorPin2, GPIO_LOW);
    System::setGPIOValue(rightMotorPin1, GPIO_LOW);
    System::setGPIOValue(rightMotorPin2, GPIO_LOW);
    System::setGPIOValue(leftPwmPin, GPIO_LOW);
    System::setGPIOValue(rightPwmPin, GPIO_LOW);
}

// Initialize motor controller
bool MotorController::initialize() {
    if (!System::init()) {
        return false;
    }
    
    initPins();
    
    // PWM frequency for motor control (1kHz)
    pwm_frequency = 1000;
    
    return true;
}

// Set left and right motor speeds
void MotorController::setSpeed(int16_t left, int16_t right) {
    // Limit speed range
    leftSpeed = std::clamp(left, static_cast<int16_t>(-255), static_cast<int16_t>(255));
    rightSpeed = std::clamp(right, static_cast<int16_t>(-255), static_cast<int16_t>(255));
    
    // Control left motor direction and speed
    if (leftSpeed >= 0) {
        System::setGPIOValue(leftMotorPin1, GPIO_HIGH);
        System::setGPIOValue(leftMotorPin2, GPIO_LOW);
        // Set PWM duty cycle (0-100%)
        float duty_cycle = (leftSpeed / 255.0f) * 100.0f;
        System::setPWM(leftPwmPin, pwm_frequency, duty_cycle);
    } else {
        System::setGPIOValue(leftMotorPin1, GPIO_LOW);
        System::setGPIOValue(leftMotorPin2, GPIO_HIGH);
        // Set PWM duty cycle for reverse direction
        float duty_cycle = (-leftSpeed / 255.0f) * 100.0f;
        System::setPWM(leftPwmPin, pwm_frequency, duty_cycle);
    }
    
    // Control right motor direction and speed
    if (rightSpeed >= 0) {
        System::setGPIOValue(rightMotorPin1, GPIO_HIGH);
        System::setGPIOValue(rightMotorPin2, GPIO_LOW);
        // Set PWM duty cycle (0-100%)
        float duty_cycle = (rightSpeed / 255.0f) * 100.0f;
        System::setPWM(rightPwmPin, pwm_frequency, duty_cycle);
    } else {
        System::setGPIOValue(rightMotorPin1, GPIO_LOW);
        System::setGPIOValue(rightMotorPin2, GPIO_HIGH);
        // Set PWM duty cycle for reverse direction
        float duty_cycle = (-rightSpeed / 255.0f) * 100.0f;
        System::setPWM(rightPwmPin, pwm_frequency, duty_cycle);
    }
}

// Stop all motors
void MotorController::stop() {
    setSpeed(0, 0);
    
    // Ensure all pins are low
    System::setGPIOValue(leftMotorPin1, GPIO_LOW);
    System::setGPIOValue(leftMotorPin2, GPIO_LOW);
    System::setGPIOValue(rightMotorPin1, GPIO_LOW);
    System::setGPIOValue(rightMotorPin2, GPIO_LOW);
    System::setGPIOValue(leftPwmPin, GPIO_LOW);
    System::setGPIOValue(rightPwmPin, GPIO_LOW);
}