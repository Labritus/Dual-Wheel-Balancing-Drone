#include "IOI2C.hpp"

// Initialize I2C
void IOI2C::init()
{
    RCC->APB2ENR |= 1 << 3;     // Enable PORTB clock

    GPIOB->CRL &= 0X00FFFFFF;   // Clear settings for PB6, PB7
    GPIOB->CRL |= 0X33000000;   // Set PB6, PB7 as push-pull output, 50MHz
    GPIOB->ODR |= 3 << 6;       // Set PB6, PB7 high
}

// Simple delay
void IOI2C::delay()
{
    uint8_t t = 10; // Simple software delay
    while (t--);
}

// Set SDA as input
void IOI2C::setSDAInput()
{
    GPIOB->CRL &= 0X0FFFFFFF;   // Clear settings for PB7
    GPIOB->CRL |= 0X80000000;   // Set PB7 as input with pull-up/pull-down
    GPIOB->ODR |= 1 << 7;       // Pull-up on PB7
}

// Set SDA as output
void IOI2C::setSDAOutput()
{
    GPIOB->CRL &= 0X0FFFFFFF;   // Clear settings for PB7
    GPIOB->CRL |= 0X30000000;   // Set PB7 as push-pull output, 50MHz
}

// Generate I2C start condition
void IOI2C::start()
{
    setSDAOutput();     // SDA line output
    SDA = 1;
    SCL = 1;
    delay();
    SDA = 0;            // START: SDA goes low while SCL is high
    delay();
    SCL = 0;            // Hold the I2C bus, ready to send or receive
}

// Generate I2C stop condition
void IOI2C::stop()
{
    setSDAOutput();     // SDA line output
    SCL = 0;
    SDA = 0;            // STOP: SDA goes high while SCL is high
    delay();
    SCL = 1;
    SDA = 1;            // Send I2C stop signal
    delay();
}

// Wait for ACK
// Return value: 1 - ACK not received, 0 - ACK received
uint8_t IOI2C::waitAck()
{
    uint8_t errCount = 0;

    setSDAInput();      // Set SDA as input
    SDA = 1;
    delay();
    SCL = 1;
    delay();

    while (READ_SDA) {
        errCount++;
        if (errCount > 250) {
            stop();
            return 1;
        }
    }

    SCL = 0;
    return 0;
}

// Generate ACK
void IOI2C::ack()
{
    SCL = 0;
    setSDAOutput();     // SDA output
    SDA = 0;            // ACK: SDA = 0
    delay();
    SCL = 1;
    delay();
    SCL = 0;
}

// Generate NACK
void IOI2C::nack()
{
    SCL = 0;
    setSDAOutput();     // SDA output
    SDA = 1;            // NACK: SDA = 1
    delay();
    SCL = 1;
    delay();
    SCL = 0;
}

// Send one byte over I2C
void IOI2C::sendByte(uint8_t txd)
{
    uint8_t i;

    setSDAOutput();     // SDA output
    SCL = 0;            // Pull SCL low to start sending

    for (i = 0; i < 8; i++) {
        SDA = (txd & 0x80) >> 7;
        txd <<= 1;      // Send MSB first
        delay();
        SCL = 1;
        delay();
        SCL = 0;
        delay();
    }
}

// Read one byte, send ACK if ack1 = 1, else send NACK
uint8_t IOI2C::readByte(uint8_t ack1)
{
    uint8_t i, receive = 0;

    setSDAInput();      // Set SDA as input

    for (i = 0; i < 8; i++) {
        SCL = 0;
        delay();
        SCL = 1;
        receive <<= 1;
        if (READ_SDA) receive++;   // Receive MSB first
        delay();
    }

    if (!ack1)
        nack();         // Send NACK
    else
        ack();          // Send ACK

    return receive;
}

// Write one byte to a specific register of an I2C device
uint8_t IOI2C::writeByteToRegister(uint8_t deviceAddr, uint8_t registerAddr, uint8_t data)
{
    start();
    sendByte(deviceAddr);   // Send device address + write
    if (waitAck()) {
        stop();
        return 1;
    }
    sendByte(registerAddr); // Send register address
    waitAck();
    sendByte(data);         // Send data
    if (waitAck()) {
        stop();
        return 1;
    }
    stop();
    return 0;
}

// Read one byte from a specific register of an I2C device
uint8_t IOI2C::readByteFromRegister(uint8_t deviceAddr, uint8_t registerAddr)
{
    char res = 0;
    start();
    sendByte(deviceAddr);      // Send device address + write
    res++;
    waitAck();
    stop();

    sendByte(registerAddr); res++; // Send register address
    waitAck();

    start();
    sendByte(deviceAddr + 1); res++; // Send device address + read
    waitAck();
    res = readByte(0);         // Read one byte, send NACK
    stop();

    return res;
}

// Read multiple bytes from an I2C device
uint8_t IOI2C::readMultiBytesFromRegister(uint8_t deviceAddr, uint8_t registerAddr, uint8_t len, uint8_t* buffer)
{
    uint8_t i;

    start();
    sendByte(deviceAddr);     // Send device address + write
    if (waitAck()) {
        stop();
        return 1;
    }
    sendByte(registerAddr);   // Send register address
    waitAck();

    start();
    sendByte(deviceAddr + 1); // Send device address + read
    waitAck();

    for (i = 0; i < len; i++) {
        if (i != len - 1)
            buffer[i] = readByte(1); // Read with ACK
        else
            buffer[i] = readByte(0); // Last byte, send NACK
    }
    stop();

    return 0;
}

// Write multiple bytes to a specific register of an I2C device
uint8_t IOI2C::writeMultiBytesToRegister(uint8_t deviceAddr, uint8_t registerAddr, uint8_t len, uint8_t* buffer)
{
    uint8_t i;

    start();
    sendByte(deviceAddr);     // Send device address + write
    if (waitAck()) {
        stop();
        return 1;
    }
    sendByte(registerAddr);   // Send register address
    waitAck();

    for (i = 0; i < len; i++) {
        sendByte(buffer[i]);  // Send data
        if (waitAck()) {
            stop();
            return 1;
        }
    }
    stop();

    return 0;
}
