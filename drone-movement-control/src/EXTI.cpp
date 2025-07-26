#include "EXTI.hpp"
#include "System.hpp"
#include "GPIOHelper.hpp"

/**
 * @brief Initialize external interrupt
 * Configure PA12 as input and connect it to MPU6050's interrupt pin
 */
void EXTI1::init()
{
    if (!GPIOHelper::isInitialized()) {
        GPIOHelper::init();
    }
    
    // Setup GPIO pin 12 with falling edge interrupt
    GPIOHelper::setupInterrupt(12, GPIOEdge::FALLING);
}