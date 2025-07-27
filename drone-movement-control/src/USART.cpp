#include "USART.hpp"
#include <iostream>
#include <cstdio>
#include <cstdarg>

bool USART::initialized_ = false;

void USART::init() {
    // For Raspberry Pi, we'll use standard output
    // In a real implementation, this would initialize UART hardware
    initialized_ = true;
}

void USART::printf(const char* format, ...) {
    if (!initialized_) {
        init();
    }
    
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    fflush(stdout);
}

void USART::putchar(char c) {
    if (!initialized_) {
        init();
    }
    
    std::putchar(c);
    fflush(stdout);
}

void USART::puts(const char* str) {
    if (!initialized_) {
        init();
    }
    
    std::puts(str);
    fflush(stdout);
}