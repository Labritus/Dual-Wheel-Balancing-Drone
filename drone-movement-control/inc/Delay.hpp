#ifndef __DELAY_HPP
#define __DELAY_HPP

#include "System.hpp"

// Delay utility functions
class Delay {
public:
    // Initialize delay functions
    // SYSTICK clock is fixed to 1/8 of HCLK
    // SYSCLK: system clock frequency in MHz
    static void init(uint8_t SYSCLK);
    
    // Delay in microseconds
    // nus: number of microseconds to delay
    static void us(uint32_t nus);
    
    // Delay in milliseconds
    // nms: number of milliseconds to delay
    static void ms(uint16_t nms);
};

#endif // __DELAY_HPP
