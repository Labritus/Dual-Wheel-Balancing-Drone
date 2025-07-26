#ifndef __EXTI_HPP
#define __EXTI_HPP

#include "System.hpp"

// External interrupt handler for Raspberry Pi GPIO
class EXTI1 {
public:
    // Initialize external interrupt on GPIO pin for MPU6050
    static void init();
};

#endif // __EXTI_HPP