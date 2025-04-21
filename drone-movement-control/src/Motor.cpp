#include "Motor.hpp"

/**
 * Initialize motor control pins
 */
void Motor::init()
{
    RCC->APB2ENR |= 1 << 3;        // Enable PORTB clock   
    GPIOB->CRH &= 0X0000FFFF;      // Set PORTB12 13 14 15
    GPIOB->CRH |= 0X22220000;      // PORTB12 13 14 15 push-pull output
}

/**
 * Initialize PWM
 * @param arr Auto-reload value
 * @param psc Prescaler value
 */
void Motor::pwmInit(uint16_t arr, uint16_t psc)
{         
    init();                         // Initialize motor control IO
    
    RCC->APB2ENR |= 1 << 11;       // Enable TIM1 clock    
    RCC->APB2ENR |= 1 << 2;        // Enable PORTA clock     
    GPIOA->CRH &= 0XFFFF0FF0;      // Configure PA8, PA11 as alternate function output
    GPIOA->CRH |= 0X0000B00B;      // Configure PA8, PA11 as alternate function output
    
    TIM1->ARR = arr;               // Set auto-reload register 
    TIM1->PSC = psc;               // Set prescaler
    TIM1->CCMR2 |= 6 << 12;        // CH4 PWM1 mode    
    TIM1->CCMR1 |= 6 << 4;         // CH1 PWM1 mode    
    TIM1->CCMR2 |= 1 << 11;        // Enable CH4 preload     
    TIM1->CCMR1 |= 1 << 3;         // Enable CH1 preload      
    TIM1->CCER |= 1 << 12;         // Enable CH4 output       
    TIM1->CCER |= 1 << 0;          // Enable CH1 output    
    TIM1->BDTR |= 1 << 15;         // Required for TIM1 PWM output
    TIM1->CR1 = 0x80;              // Enable ARPE 
    TIM1->CR1 |= 0x01;             // Enable TIM1             
}

/**
 * Left motor forward
 */
void Motor::leftForward()
{
    AIN1 = 1;
    AIN2 = 0;
}

/**
 * Left motor backward
 */
void Motor::leftBackward()
{
    AIN1 = 0;
    AIN2 = 1;
}

/**
 * Right motor forward
 */
void Motor::rightForward()
{
    BIN1 = 1;
    BIN2 = 0;
}

/**
 * Right motor backward
 */
void Motor::rightBackward()
{
    BIN1 = 0;
    BIN2 = 1;
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
    PWMA = pwm;
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
    PWMB = pwm;
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
    AIN1 = 0;
    AIN2 = 0;
    BIN1 = 0;
    BIN2 = 0;
    PWMA = 0;
    PWMB = 0;
}
