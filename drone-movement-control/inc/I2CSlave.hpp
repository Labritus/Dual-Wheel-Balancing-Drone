#ifndef __I2CSLAVE_HPP
#define __I2CSLAVE_HPP

#include "System.hpp"
#include <string>

class I2CSlave {
public:
    // Initialize I2C slave with address 0x08 on Raspberry Pi
    static void init();
    
    // Cleanup I2C resources
    static void cleanup();
    
    // Check if a new message has been received
    static bool hasNewMessage();
    
    // Clear the message flag
    static void clearMessageFlag();
    
    // Check if the received message indicates PERSON_DETECTED
    static bool isPersonDetected();
    
    // Get the last received message (for debugging)
    static std::string getLastMessage();
    
    // Check if I2C is properly initialized
    static bool isInitialized();
};

#endif // __I2CSLAVE_HPP
