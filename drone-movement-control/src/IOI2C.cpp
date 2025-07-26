#include "IOI2C.hpp"
#include "I2CHelper.hpp"

// Initialize I2C
void IOI2C::init()
{
    if (!I2CHelper::isInitialized()) {
        I2CHelper::init("/dev/i2c-1");
    }
}

// Legacy bit-banged I2C functions - no longer used with hardware I2C
// Kept for API compatibility but not implemented

void IOI2C::delay() {
    // No longer needed with hardware I2C
}

void IOI2C::setSDAInput() {
    // No longer needed with hardware I2C
}

void IOI2C::setSDAOutput() {
    // No longer needed with hardware I2C
}

void IOI2C::start() {
    // No longer needed with hardware I2C
}

void IOI2C::stop() {
    // No longer needed with hardware I2C
}

uint8_t IOI2C::waitAck() {
    // No longer needed with hardware I2C
    return 0;
}

void IOI2C::ack() {
    // No longer needed with hardware I2C
}

void IOI2C::nack() {
    // No longer needed with hardware I2C
}

void IOI2C::sendByte(uint8_t txd) {
    // No longer needed with hardware I2C
    (void)txd;
}

uint8_t IOI2C::readByte(uint8_t ack1) {
    // No longer needed with hardware I2C
    (void)ack1;
    return 0;
}

// Write one byte to a specific register of an I2C device
uint8_t IOI2C::writeByteToRegister(uint8_t deviceAddr, uint8_t registerAddr, uint8_t data)
{
    return I2CHelper::writeByteToRegister(deviceAddr, registerAddr, data) ? 0 : 1;
}

// Read one byte from a specific register of an I2C device
uint8_t IOI2C::readByteFromRegister(uint8_t deviceAddr, uint8_t registerAddr)
{
    return I2CHelper::readByteFromRegister(deviceAddr, registerAddr);
}

// Read multiple bytes from an I2C device
uint8_t IOI2C::readMultiBytesFromRegister(uint8_t deviceAddr, uint8_t registerAddr, uint8_t len, uint8_t* buffer)
{
    return I2CHelper::readMultiBytesFromRegister(deviceAddr, registerAddr, len, buffer) ? 0 : 1;
}

// Write multiple bytes to a specific register of an I2C device
uint8_t IOI2C::writeMultiBytesToRegister(uint8_t deviceAddr, uint8_t registerAddr, uint8_t len, uint8_t* buffer)
{
    return I2CHelper::writeMultiBytesToRegister(deviceAddr, registerAddr, len, buffer) ? 0 : 1;
}
