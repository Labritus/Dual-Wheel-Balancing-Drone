#ifndef __USART3_HPP
#define __USART3_HPP

#include "System.hpp"

extern uint8_t Usart3_Receive;  // Received data

// USART3 Bluetooth communication class
class USART3Driver {
public:
    // Initialize USART3
    static void init(uint32_t pclk2, uint32_t bound);
    
    // Get received data
    static uint8_t getReceiveData();
    
    // Process received command
    static void processCommand(uint8_t command);
    
    // Interrupt handler
    static void irqHandler();
    
private:
    static uint8_t receive_data;  // Received data
};

// External variables

#endif // __USART3_HPP
