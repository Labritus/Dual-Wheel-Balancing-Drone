#ifndef GPIO_INTERFACE_H
#define GPIO_INTERFACE_H

#include <cstdint>
#include <string>

enum class PinMode {
    INPUT,
    OUTPUT,
    PWM
};

enum class PinState {
    LOW,
    HIGH
};

class GPIOInterface {
public:
    GPIOInterface();
    ~GPIOInterface();
    
    // Initialize the GPIO pin
    bool initialize(int pin, PinMode mode);
    
    // Release the GPIO pin
    void release(int pin);
    
    // Set the digital output value
    bool digitalWrite(int pin, PinState state);
    
    // Read the digital input value
    PinState digitalRead(int pin);
    
    // Set the PWM value (0-255)
    bool setPWM(int pin, int value);
    
    // Close and release all resources
    void cleanup();
    
private:
    // Export the pin to sysfs
    bool exportPin(int pin);
    
    // Set the pin direction
    bool setDirection(int pin, PinMode mode);
    
    // Initialized pins
    int* initialized_pins_;
    int num_pins_;
};

#endif // GPIO_INTERFACE_H
