#ifndef __TIMER_HPP
#define __TIMER_HPP

#include "System.hpp"

// Timer class
class Timer {
public:
    // Initialize TIM2 for input capture
    static void initCapture(uint16_t arr, uint16_t psc);
    
    // Get TIM2 capture status
    static uint8_t getCaptureStatus();
    
    // Get TIM2 high-level duration
    static uint32_t getCaptureHighTime();
};

#endif // __TIMER_HPP
