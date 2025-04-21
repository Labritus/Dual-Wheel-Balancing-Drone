#ifndef __EXTI_HPP
#define __EXTI_HPP

#include "System.hpp"

/**
 * @brief External interrupt class
 * Mainly used for handling MPU6050 interrupts
 */
class EXTI1 {
public:
    /**
     * @brief Initialize external interrupt
     * Configures PA12 as input, connected to MPU6050 interrupt pin
     */
    static void init();
    
    /**
     * @brief Get the status of the interrupt pin
     * @return Interrupt pin status
     */
    static bool isPinInterrupt() {
        return PAin(12);
    }
};

// Macro definition for compatibility with old code
#define INT PAin(12)   // PA12 connected to MPU6050 interrupt pin

#endif // __EXTI_HPP
