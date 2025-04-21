#include "EXTI.hpp"
#include "System.hpp"

/**
 * @brief Initialize external interrupt
 * Configure PA12 as input and connect it to MPU6050's interrupt pin
 */
void EXTI1::init()
{
    // Enable PORTA clock
    RCC->APB2ENR |= 1 << 2;

    // Configure PA12 as input mode
    GPIOA->CRH &= 0XFFF0FFFF;
    GPIOA->CRH |= 0X00080000;
    
    // Enable pull-up on PA12
    GPIOA->ODR |= 1 << 12;

    // Configure falling edge trigger
    System::exNvicConfig(GPIO_A, 12, FTIR);
    
    // Configure NVIC: preemption priority = 2, subpriority = 1
    System::nvicInit(2, 1, EXTI15_10_IRQn, 2);
}
