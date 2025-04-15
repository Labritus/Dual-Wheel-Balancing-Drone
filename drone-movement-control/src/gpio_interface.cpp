#include "../include/gpio_interface.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>

// Maximum number of pins that can be initialized
#define MAX_PINS 64

GPIOInterface::GPIOInterface() {
    initialized_pins_ = new int[MAX_PINS];
    num_pins_ = 0;
    
    for (int i = 0; i < MAX_PINS; i++) {
        initialized_pins_[i] = -1;
    }
}

GPIOInterface::~GPIOInterface() {
    cleanup();
    delete[] initialized_pins_;
}

bool GPIOInterface::initialize(int pin, PinMode mode) {
    // Check if the pin is already initialized
    for (int i = 0; i < num_pins_; i++) {
        if (initialized_pins_[i] == pin) {
            return true; // Already initialized
        }
    }
    
    // Export the pin
    if (!exportPin(pin)) {
        return false;
    }
    
    // Set the pin direction
    if (!setDirection(pin, mode)) {
        return false;
    }
    
    // Add to the list of initialized pins
    if (num_pins_ < MAX_PINS) {
        initialized_pins_[num_pins_++] = pin;
        return true;
    }
    
    return false;
}

void GPIOInterface::release(int pin) {
    std::ofstream unexport_file("/sys/class/gpio/unexport");
    if (!unexport_file.is_open()) {
        std::cerr << "Failed to open unexport file" << std::endl;
        return;
    }
    
    unexport_file << pin;
    unexport_file.close();
    
    // Remove from the list of initialized pins
    for (int i = 0; i < num_pins_; i++) {
        if (initialized_pins_[i] == pin) {
            for (int j = i; j < num_pins_ - 1; j++) {
                initialized_pins_[j] = initialized_pins_[j + 1];
            }
            num_pins_--;
            initialized_pins_[num_pins_] = -1;
            break;
        }
    }
}

bool GPIOInterface::digitalWrite(int pin, PinState state) {
    std::stringstream ss;
    ss << "/sys/class/gpio/gpio" << pin << "/value";
    
    std::ofstream value_file(ss.str());
    if (!value_file.is_open()) {
        std::cerr << "Failed to open GPIO value file: " << ss.str() << std::endl;
        return false;
    }
    
    value_file << (state == PinState::HIGH ? "1" : "0");
    value_file.close();
    
    return true;
}

PinState GPIOInterface::digitalRead(int pin) {
    std::stringstream ss;
    ss << "/sys/class/gpio/gpio" << pin << "/value";
    
    std::ifstream value_file(ss.str());
    if (!value_file.is_open()) {
        std::cerr << "Failed to open GPIO value file: " << ss.str() << std::endl;
        return PinState::LOW; // Default to returning LOW
    }
    
    char value;
    value_file >> value;
    value_file.close();
    
    return (value == '1') ? PinState::HIGH : PinState::LOW;
}

bool GPIOInterface::setPWM(int pin, int value) {
    // PWM control should be implemented according to hardware specifications
    // For a simple example, we can use software emulation of PWM
    // But in practice, hardware PWM should be used
    
    std::cout << "PWM feature not yet implemented, setting pin " << pin << " to value " << value << std::endl;
    
    // For non-hardware PWM implementation, consider using software PWM or simply setting the digital output
    if (value > 127) {
        return digitalWrite(pin, PinState::HIGH);
    } else {
        return digitalWrite(pin, PinState::LOW);
    }
}

void GPIOInterface::cleanup() {
    for (int i = 0; i < num_pins_; i++) {
        if (initialized_pins_[i] != -1) {
            release(initialized_pins_[i]);
        }
    }
    num_pins_ = 0;
}

bool GPIOInterface::exportPin(int pin) {
    std::ofstream export_file("/sys/class/gpio/export");
    if (!export_file.is_open()) {
        std::cerr << "Failed to open export file" << std::endl;
        return false;
    }
    
    export_file << pin;
    export_file.close();
    
    // Wait for the GPIO filesystem to be ready
    usleep(100000); // Wait 100ms
    
    return true;
}

bool GPIOInterface::setDirection(int pin, PinMode mode) {
    std::stringstream ss;
    ss << "/sys/class/gpio/gpio" << pin << "/direction";
    
    std::ofstream direction_file(ss.str());
    if (!direction_file.is_open()) {
        std::cerr << "Failed to open GPIO direction file: " << ss.str() << std::endl;
        return false;
    }
    
    if (mode == PinMode::INPUT) {
        direction_file << "in";
    } else {
        direction_file << "out";
    }
    
    direction_file.close();
    return true;
}
