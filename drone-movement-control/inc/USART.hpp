#ifndef __USART_HPP
#define __USART_HPP

#include "System.hpp"
#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <cmath>

class USART {
public:
    // Initialize USART1
    // pclk2: clock frequency
    // bound: baud rate
    static void init(uint32_t pclk2, uint32_t bound);
    
    // Send a single byte
    static void send(uint8_t data);
    
    // Print formatted data to serial port
    static int printf(const char *fmt, ...);
};

// Interrupt handler declaration
extern "C" void USART3_IRQHandler(void);

#endif // __USART_HPP
