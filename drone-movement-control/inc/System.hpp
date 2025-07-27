#ifndef __SYSTEM_HPP
#define __SYSTEM_HPP

#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>

// Raspberry Pi GPIO pin definitions
#define GPIO_PIN_2  2
#define GPIO_PIN_3  3
#define GPIO_PIN_4  4
#define GPIO_PIN_17 17
#define GPIO_PIN_27 27
#define GPIO_PIN_22 22
#define GPIO_PIN_10 10
#define GPIO_PIN_9  9
#define GPIO_PIN_11 11
#define GPIO_PIN_5  5
#define GPIO_PIN_6  6
#define GPIO_PIN_13 13
#define GPIO_PIN_19 19
#define GPIO_PIN_26 26

// GPIO direction
#define GPIO_INPUT  0
#define GPIO_OUTPUT 1

// GPIO levels
#define GPIO_LOW  0
#define GPIO_HIGH 1

// Edge trigger types
#define FALLING_EDGE 1
#define RISING_EDGE  2
#define BOTH_EDGES   3

// Raspberry Pi System class
class System {
public:
    static bool init();                                    // Initialize system
    static void shutdown();                               // Shutdown system
    static void setGPIOMode(uint8_t pin, uint8_t mode);  // Set GPIO pin mode
    static void setGPIOValue(uint8_t pin, uint8_t value); // Set GPIO pin value
    static uint8_t getGPIOValue(uint8_t pin);            // Get GPIO pin value
    static void setPWM(uint8_t pin, uint32_t frequency, float dutyCycle); // Set PWM
    static bool setupI2C(uint8_t bus);                   // Setup I2C bus
    static bool setupSPI(uint8_t bus);                   // Setup SPI bus
    static void delayMicroseconds(uint32_t us);          // Non-blocking microsecond delay
    
private:
    static std::atomic<bool> initialized_;
    static std::mutex gpio_mutex_;
};

#endif // __SYSTEM_HPP
