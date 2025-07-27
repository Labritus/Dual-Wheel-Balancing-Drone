#ifndef __USART3_HPP
#define __USART3_HPP

#include <stdint.h>

// USART3 driver class - adapted for Raspberry Pi
class USART3Driver {
public:
    // Initialize USART3 - parameters adapted for generic use
    static void init(uint32_t clock, uint32_t baudrate);
    
    // Send single byte
    static void sendByte(uint8_t byte);
    
    // Send data buffer
    static void sendData(const uint8_t* data, uint32_t length);
    
    // Check if data is available
    static bool isDataAvailable();
    
    // Read single byte
    static uint8_t readByte();
    
    // Get received data count
    static uint32_t getReceivedCount();
    
    // Clear receive buffer
    static void clearReceiveBuffer();
    
private:
    static bool initialized_;
    static uint32_t received_count_;
};

#endif // __USART3_HPP