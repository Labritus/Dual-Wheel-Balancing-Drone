#ifndef __IOI2C_HPP
#define __IOI2C_HPP

#include "System.hpp"

// SCL and SDA pin definitions
#define SCL       PBout(6)   // SCL
#define SDA       PBout(7)   // SDA
#define READ_SDA  PBin(7)    // SDA input

// Software I2C operation functions
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
    static uint8_t sendByte(uint8_t txd);
    
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
    // Set SDA as input
    static void setSDAInput();
    
    // Set SDA as output
    static void setSDAOutput();
    
    // Small delay
    static void delay();
};

#endif // __IOI2C_HPP
