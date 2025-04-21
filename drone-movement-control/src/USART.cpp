#include "USART.hpp"

extern "C" {
#include <stdio.h>    // For vsnprintf
#include <stdarg.h>   // va_list, va_start, va_end
#include <math.h>     // pow
#include <string.h>
}

// Low-level: send a single character to UART register
static void send_char(char c) {
    while (!(USART3->SR & (1 << 7)));  // Wait for TXE flag
    USART3->DR = (uint8_t)c;
}

// Formatted print to UART: use vsnprintf to fill buffer, then send one char at a time
int USART::printf(const char *fmt, ...) {
    char buffer[256];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    for (int i = 0; i < len; ++i) {
        send_char(buffer[i]);
    }
    return len;
}

// Global variable definitions


// Initialize USART1
void USART::init(uint32_t pclk2, uint32_t bound) {
    float temp;
    uint16_t mantissa;
    uint16_t fraction;
    temp = (float)(pclk2 * 1000000) / (bound * 16); // Calculate USARTDIV
    mantissa = temp;                        // Integer part
    fraction = (temp - mantissa) * 16;      // Fractional part
    mantissa <<= 4;
    mantissa += fraction;
    
    RCC->APB2ENR |= 1 << 2;                 // Enable PORTA clock
    RCC->APB2ENR |= 1 << 14;                // Enable USART1 clock
    GPIOA->CRH &= 0XFFFFF00F;               // Configure IO
    GPIOA->CRH |= 0X000008B0;               // Configure IO
    
    RCC->APB2RSTR |= 1 << 14;               // Reset USART1
    RCC->APB2RSTR &= ~(1 << 14);            // Stop reset
    
    // Set baud rate
    USART1->BRR = mantissa;                 // Baud rate register
    USART1->CR1 |= 0X200C;                  // Enable USART, 1 stop bit, no parity
}

// Send a single byte of data
void USART::send(uint8_t data) {
    USART1->DR = data;
    while ((USART1->SR & 0x40) == 0); // Wait for transmission complete
}

// Initialize USART3
