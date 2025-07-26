#include "Motor.hpp"
#include "GPIOHelper.hpp"
#include <unistd.h>
#include <fcntl.h>
#include <fstream>
#include <sstream>

/**
 * Initialize motor control pins
 */
void Motor::init()
{
    if (!GPIOHelper::isInitialized()) {
        GPIOHelper::init();
    }
    
    GPIOHelper::setMode(12, GPIOMode::OUTPUT);  // BIN2
    GPIOHelper::setMode(13, GPIOMode::OUTPUT);  // BIN1
    GPIOHelper::setMode(14, GPIOMode::OUTPUT);  // AIN2
    GPIOHelper::setMode(15, GPIOMode::OUTPUT);  // AIN1
}

/**
 * Initialize PWM
 * @param arr Auto-reload value
 * @param psc Prescaler value
 */
void Motor::pwmInit(uint16_t arr, uint16_t psc)
{         
    init();
    
    // Initialize hardware PWM using Linux PWM subsystem
    // PWM0 for left motor (PWMA), PWM1 for right motor (PWMB)
    initHardwarePWM(0, arr, psc);
    initHardwarePWM(1, arr, psc);
}

/**
 * Left motor forward
 */
void Motor::leftForward()
{
    GPIOHelper::setValue(15, GPIOValue::HIGH);  // AIN1
    GPIOHelper::setValue(14, GPIOValue::LOW);   // AIN2
}

/**
 * Left motor backward
 */
void Motor::leftBackward()
{
    GPIOHelper::setValue(15, GPIOValue::LOW);   // AIN1
    GPIOHelper::setValue(14, GPIOValue::HIGH);  // AIN2
}

/**
 * Right motor forward
 */
void Motor::rightForward()
{
    GPIOHelper::setValue(13, GPIOValue::HIGH);  // BIN1
    GPIOHelper::setValue(12, GPIOValue::LOW);   // BIN2
}

/**
 * Right motor backward
 */
void Motor::rightBackward()
{
    GPIOHelper::setValue(13, GPIOValue::LOW);   // BIN1
    GPIOHelper::setValue(12, GPIOValue::HIGH);  // BIN2
}

/**
 * Set PWM for left motor
 * @param pwm PWM value
 */
void Motor::setLeftPwm(int pwm)
{
    // Clamp PWM range
    if (pwm > 7200) pwm = 7200;
    else if (pwm < 0) pwm = 0;
    
    // Set PWM value
    setPWMValue(0, pwm);
}

/**
 * Set PWM for right motor
 * @param pwm PWM value
 */
void Motor::setRightPwm(int pwm)
{
    // Clamp PWM range
    if (pwm > 7200) pwm = 7200;
    else if (pwm < 0) pwm = 0;
    
    // Set PWM value
    setPWMValue(1, pwm);
}

/**
 * Set PWM for both motors
 * @param left_pwm PWM value for left motor
 * @param right_pwm PWM value for right motor
 */
void Motor::setPwm(int left_pwm, int right_pwm)
{
    // Set left motor direction
    if (left_pwm > 0) {
        leftForward();
    } else {
        leftBackward();
        left_pwm = -left_pwm; // Use absolute value
    }
    
    // Set right motor direction
    if (right_pwm > 0) {
        rightForward();
    } else {
        rightBackward();
        right_pwm = -right_pwm; // Use absolute value
    }
    
    // Set PWM values
    setLeftPwm(left_pwm);
    setRightPwm(right_pwm);
}

/**
 * Stop both motors
 */
void Motor::stop()
{
    GPIOHelper::setValue(15, GPIOValue::LOW);  // AIN1
    GPIOHelper::setValue(14, GPIOValue::LOW);  // AIN2
    GPIOHelper::setValue(13, GPIOValue::LOW);  // BIN1
    GPIOHelper::setValue(12, GPIOValue::LOW);  // BIN2
    setPWMValue(0, 0);
    setPWMValue(1, 0);
}

void Motor::initHardwarePWM(int channel, uint16_t period, uint16_t prescaler) {
    std::string pwm_path = "/sys/class/pwm/pwmchip0/pwm" + std::to_string(channel);
    
    // Export PWM channel if not already exported
    std::ofstream export_file("/sys/class/pwm/pwmchip0/export");
    if (export_file.is_open()) {
        export_file << channel;
        export_file.close();
    }
    
    // Set period (in nanoseconds)
    uint32_t period_ns = (period + 1) * (prescaler + 1) * 1000 / 72;  // Assuming 72MHz base clock
    std::ofstream period_file(pwm_path + "/period");
    if (period_file.is_open()) {
        period_file << period_ns;
        period_file.close();
    }
    
    // Enable PWM
    std::ofstream enable_file(pwm_path + "/enable");
    if (enable_file.is_open()) {
        enable_file << "1";
        enable_file.close();
    }
}

void Motor::setPWMValue(int channel, int value) {
    std::string pwm_path = "/sys/class/pwm/pwmchip0/pwm" + std::to_string(channel);
    
    // Read current period
    std::ifstream period_file(pwm_path + "/period");
    uint32_t period_ns = 0;
    if (period_file.is_open()) {
        period_file >> period_ns;
        period_file.close();
    }
    
    // Calculate duty cycle (value is 0-7200, scale to period)
    uint32_t duty_cycle_ns = (value * period_ns) / 7200;
    
    // Set duty cycle
    std::ofstream duty_file(pwm_path + "/duty_cycle");
    if (duty_file.is_open()) {
        duty_file << duty_cycle_ns;
        duty_file.close();
    }
}
