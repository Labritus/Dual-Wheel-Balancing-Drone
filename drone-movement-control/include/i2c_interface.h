#ifndef I2C_INTERFACE_H
#define I2C_INTERFACE_H

#include <cstdint>
#include <string>

class I2CInterface {
public:
    I2CInterface();
    ~I2CInterface();
    
    // Initialize the I2C bus
    bool initialize(const std::string& device, uint8_t address);
    
    // Close the I2C bus
    void close();
    
    // Read the register value
    bool readByte(uint8_t reg, uint8_t* data);
    bool readBytes(uint8_t reg, uint8_t* data, size_t length);
    
    // Write the register value
    bool writeByte(uint8_t reg, uint8_t data);
    bool writeBytes(uint8_t reg, uint8_t* data, size_t length);
    
private:
    int fd_;          // File descriptor
    uint8_t address_; // Device address
    bool initialized_; // Initialization status
};

#endif // I2C_INTERFACE_H
