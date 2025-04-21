#include "PWM.hpp"

// Initialize motor interface
void PWM::motorInit()
{
    RCC->APB2ENR |= 1 << 3;        // Enable clock for PORTB
    GPIOB->CRH &= 0X0000FFFF;      // Set PB12~PB15 as push-pull output
    GPIOB->CRH |= 0X22220000;      // Set PB12~PB15 as push-pull output
}

// Initialize PWM to drive motors
void PWM::init(uint16_t arr, uint16_t psc)
{         
    motorInit();                   // Initialize IO required for motor control
    RCC->APB2ENR |= 1 << 11;       // Enable TIM1 clock
    RCC->APB2ENR |= 1 << 2;        // Enable PORTA clock
    GPIOA->CRH &= 0XFFFF0FF0;      // Set PA8 and PA11 as alternate function output
    GPIOA->CRH |= 0X0000B00B;      // Set PA8 and PA11 as alternate function output
    TIM1->ARR = arr;               // Set auto-reload value
    TIM1->PSC = psc;               // Set prescaler
    TIM1->CCMR2 |= 6 << 12;        // CH4 in PWM1 mode
    TIM1->CCMR1 |= 6 << 4;         // CH1 in PWM1 mode
    TIM1->CCMR2 |= 1 << 11;        // Enable preload for CH4
    TIM1->CCMR1 |= 1 << 3;         // Enable preload for CH1
    TIM1->CCER |= 1 << 12;         // Enable CH4 output
    TIM1->CCER |= 1 << 0;          // Enable CH1 output
    TIM1->BDTR |= 1 << 15;         // Required for TIM1 PWM output
    TIM1->CR1 = 0x80;              // Enable auto-reload preload
    TIM1->CR1 |= 0x01;             // Enable TIM1
}

// Set left motor PWM
void PWM::setLeftPwm(int pwm)
{
    if (pwm > 0) {
        AIN1 = 1;
        AIN2 = 0;
    } else {
        AIN1 = 0;
        AIN2 = 1;
    }
    
    // Take absolute value
    pwm = (pwm < 0) ? -pwm : pwm;
    
    // Limit PWM range
    if (pwm > 7200) pwm = 7200;
    
    // Set PWM value
    PWMA = pwm;
}

// Set right motor PWM
void PWM::setRightPwm(int pwm)
{
    if (pwm > 0) {
        BIN1 = 1;
        BIN2 = 0;
    } else {
        BIN1 = 0;
        BIN2 = 1;
    }
    
    // Take absolute value
    pwm = (pwm < 0) ? -pwm : pwm;
    
    // Limit PWM range
    if (pwm > 7200) pwm = 7200;
    
    // Set PWM value
    PWMB = pwm;
}

// Set PWM for both left and right motors
void PWM::setPwm(int left_pwm, int right_pwm)
{
    setLeftPwm(left_pwm);
    setRightPwm(right_pwm);
}

// Stop the motors
void PWM::stop()
{
    AIN1 = 0;
    AIN2 = 0;
    BIN1 = 0;
    BIN2 = 0;
    PWMA = 0;
    PWMB = 0;
}

// C-compatible interface implementation
extern "C" {
    void MiniBalance_PWM_Init(void)
    {
        PWM::init(7199, 0);
    }
    
    void MiniBalance_Motor_Init(void)
    {
        PWM::motorInit();
    }
    
    void Set_Pwm(int moto1, int moto2)
    {
        PWM::setPwm(moto1, moto2);
    }
}
