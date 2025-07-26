#ifndef __IOI2C_HPP
#define __IOI2C_HPP

#include "System.hpp"

// Hardware I2C operation functions (migrated from software I2C)
class IOI2C {
public:
    // Initialize I2C
    static void init();
    
    // Generate I2C start signal
    static void start();
    
    // Generate I2C stop signal
    static void stop();
    
    // Wait for ACK signal
    // Return: 1 = no ACK received, 0 = ACK received
    static uint8_t waitAck();
    
    // Generate ACK
    static void ack();
    
    // Generate NACK
    static void nack();
    
    // Send one byte over I2C
    // Return: 1 = ACK received, 0 = no ACK
    static void sendByte(uint8_t txd);
    
    // Read one byte from I2C
    // ack = 1: send ACK, ack = 0: send NACK
    static uint8_t readByte(uint8_t ack1);
    
    // Write one byte to a register of an I2C device
    static uint8_t writeByteToRegister(uint8_t deviceAddr, uint8_t registerAddr, uint8_t data);
    
    // Read one byte from a register of an I2C device
    static uint8_t readByteFromRegister(uint8_t deviceAddr, uint8_t registerAddr);
    
    // Read multiple bytes from a register of an I2C device
    static uint8_t readMultiBytesFromRegister(uint8_t deviceAddr, uint8_t registerAddr, uint8_t len, uint8_t* buffer);
    
    // Write multiple bytes to a register of an I2C device
    static uint8_t writeMultiBytesToRegister(uint8_t deviceAddr, uint8_t registerAddr, uint8_t len, uint8_t* buffer);
    
private:
    // Legacy functions - kept for API compatibility
    static void setSDAInput();
    static void setSDAOutput();
    static void delay();
};

#endif // __IOI2C_HPP
