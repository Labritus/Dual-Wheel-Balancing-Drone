#ifndef __USART_HPP
#define __USART_HPP

#include "System.hpp"
#include <cstdio>
#include <cstdarg>

// UART/Serial communication class for Raspberry Pi
class USART {
public:
    // Initialize UART communication
    static void init();
    
    // Printf-style formatted output to UART
    static void printf(const char* format, ...);
    
    // Send a single character
    static void putchar(char c);
    
    // Send a string
    static void puts(const char* str);
    
    // Check if UART is initialized
    static bool isInitialized() { return initialized_; }

private:
    static bool initialized_;
};

#endif // __USART_HPP