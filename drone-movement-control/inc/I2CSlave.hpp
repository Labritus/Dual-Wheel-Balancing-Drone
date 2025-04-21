#ifndef __I2CSLAVE_HPP
#define __I2CSLAVE_HPP

#include "System.hpp"

class I2CSlave {
public:
    // Initialize I2C slave with address 0x08
    static void init();
    
    // Check if a new message has been received
    static bool hasNewMessage();
    
    // Clear the message flag
    static void clearMessageFlag();
    
    // Check if the received message indicates PERSON_DETECTED
    static bool isPersonDetected();
};

#endif // __I2CSLAVE_HPP
