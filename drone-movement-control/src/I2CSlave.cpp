#include "I2CSlave.hpp"
#include "System.hpp"
#include <string.h>  // Include string handling header

// Buffer size
#define I2C_SLAVE_BUFFER_SIZE 32

// Data buffer
static uint8_t i2c_rx_buffer[I2C_SLAVE_BUFFER_SIZE];
static uint8_t i2c_rx_count = 0;
static bool message_ready = false;

// Initialize I2C slave, address 0x08
void I2CSlave::init() {
    // Enable I2C1 and GPIOB clocks
    RCC->APB1ENR |= 1 << 21;  // Enable I2C1 clock
    RCC->APB2ENR |= 1 << 3;   // Enable PORTB clock

    // Configure PB6 (SCL) and PB7 (SDA) as alternate function open-drain output
    GPIOB->CRL &= 0X00FFFFFF;
    GPIOB->CRL |= 0XFF000000;
    GPIOB->ODR |= 3 << 6;     // Pull-up

    // Reset I2C
    I2C1->CR1 |= 1 << 15;     // Software reset I2C
    I2C1->CR1 &= ~(1 << 15);  // Reset done

    // Configure I2C clock speed
    I2C1->CR2 = 36;           // 36MHz PCLK1
    I2C1->TRISE = 37;         // (36MHz / 1MHz) + 1
    I2C1->CCR = 180;          // 100kHz SCL

    // Set slave address
    I2C1->OAR1 = 0x08 << 1;   // Slave address 0x08 (shifted left 1 bit for STM32 format)

    // Enable I2C ACK response
    I2C1->CR1 |= 1 << 10;

    // Enable I2C
    I2C1->CR1 |= 1 << 0;

    // Enable interrupts
    I2C1->CR2 |= 1 << 9;      // Event interrupt
    I2C1->CR2 |= 1 << 10;     // Buffer interrupt
    I2C1->CR2 |= 1 << 8;      // Error interrupt

    // Configure NVIC
    System::nvicInit(1, 1, I2C1_EV_IRQn, 2);
    System::nvicInit(1, 2, I2C1_ER_IRQn, 2);
}

// Check if a new message was received
bool I2CSlave::hasNewMessage() {
    return message_ready;
}

// Clear message flag
void I2CSlave::clearMessageFlag() {
    message_ready = false;
    i2c_rx_count = 0;
}

// Check if received message is "PERSON_DETECTED"
bool I2CSlave::isPersonDetected() {
    const char* person_detected = "PERSON_DETECTED";
    uint8_t detect_len = strlen(person_detected) + 1; // Length including '\0'

    if (i2c_rx_count != strlen(person_detected)) {
        return false;
    }

    // Compare received message with "PERSON_DETECTED"
    for (uint8_t i = 0; i < strlen(person_detected); i++) {
        if (i2c_rx_buffer[i] != person_detected[i]) {
            return false;
        }
    }

    return true;
}

// I2C event interrupt handler
extern "C" {
    void I2C1_EV_IRQHandler(void) {
        uint32_t status = I2C1->SR1;

        // Address matched event
        if (status & (1 << 1)) {
            // Clear ADDR flag
            uint32_t temp = I2C1->SR1;
            temp = I2C1->SR2;
            (void)temp;

            // Prepare to receive data
            i2c_rx_count = 0;
            message_ready = false;
        }

        // Receive data event
        if (status & (1 << 6)) {
            if (i2c_rx_count < I2C_SLAVE_BUFFER_SIZE) {
                i2c_rx_buffer[i2c_rx_count++] = I2C1->DR;
            }
        }

        // Stop condition event
        if (status & (1 << 4)) {
            // Message reception complete
            message_ready = true;

            // Clear STOPF flag
            uint32_t temp = I2C1->SR1;
            I2C1->CR1 |= 1 << 0;
            (void)temp;
        }
    }

    void I2C1_ER_IRQHandler(void) {
        // Clear all error flags
        if (I2C1->SR1 & 0x0F00) {
            I2C1->SR1 &= ~0x0F00;
        }
    }
}
