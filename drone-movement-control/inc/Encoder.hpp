#ifndef __ENCODER_HPP
#define __ENCODER_HPP

#include "System.hpp"

#define ENCODER_TIM_PERIOD (uint16_t)(65535)   // Max count value for 16-bit timer (F103 timer is 16-bit)

class Encoder {
public:
    // Initialize TIM3 in encoder interface mode
    static void initTIM3();
    
    // Initialize TIM4 in encoder interface mode
    static void initTIM4();
    
    // Read encoder count in a unit time
    static int read(uint8_t TIMX);
};

// Interrupt handler declarations
extern "C" {
    void TIM3_IRQHandler(void);
    void TIM4_IRQHandler(void);
}

#endif // __ENCODER_HPP
