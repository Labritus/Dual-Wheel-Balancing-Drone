#include "GPIOHelper.hpp"
#include <iostream>
#include <poll.h>
#include <unistd.h>

// Stub GPIO Helper implementation when libgpiod is not available
// This allows compilation without GPIO functionality

std::unique_ptr<void, void(*)(void*)> GPIOHelper::chip_{nullptr, [](void*){}};
std::unordered_map<unsigned int, void*> GPIOHelper::requests_;
std::mutex GPIOHelper::gpio_mutex_;
std::atomic<bool> GPIOHelper::initialized_{false};

bool GPIOHelper::init(const char* chipname) {
    std::lock_guard<std::mutex> lock(gpio_mutex_);
    
    if (initialized_.load()) {
        return true;
    }
    
    std::cout << "GPIO stub: Would initialize chip " << chipname << " (libgpiod not available)" << std::endl;
    initialized_.store(true);
    return true;
}

void GPIOHelper::shutdown() {
    std::lock_guard<std::mutex> lock(gpio_mutex_);
    
    if (!initialized_.load()) {
        return;
    }
    
    std::cout << "GPIO stub: Shutting down GPIO (libgpiod not available)" << std::endl;
    requests_.clear();
    chip_.reset();
    initialized_.store(false);
}


bool GPIOHelper::setMode(unsigned int pin, GPIOMode mode) {
    std::lock_guard<std::mutex> lock(gpio_mutex_);
    
    if (!initialized_.load()) {
        std::cerr << "GPIO stub: Not initialized" << std::endl;
        return false;
    }
    
    std::cout << "GPIO stub: Would set pin " << pin << " to mode " << (int)mode << std::endl;
    return true;
}

bool GPIOHelper::setValue(unsigned int pin, GPIOValue value) {
    std::lock_guard<std::mutex> lock(gpio_mutex_);
    
    if (!initialized_.load()) {
        std::cerr << "GPIO stub: Not initialized" << std::endl;
        return false;
    }
    
    std::cout << "GPIO stub: Would set pin " << pin << " to value " << (int)value << std::endl;
    return true;
}

GPIOValue GPIOHelper::getValue(unsigned int pin) {
    std::lock_guard<std::mutex> lock(gpio_mutex_);
    
    if (!initialized_.load()) {
        std::cerr << "GPIO stub: Not initialized" << std::endl;
        return GPIOValue::LOW;
    }
    
    std::cout << "GPIO stub: Would read pin " << pin << std::endl;
    return GPIOValue::LOW;  // Always return LOW in stub mode
}

bool GPIOHelper::setupInterrupt(unsigned int pin, GPIOEdge edge) {
    std::lock_guard<std::mutex> lock(gpio_mutex_);
    
    if (!initialized_.load()) {
        std::cerr << "GPIO stub: Not initialized" << std::endl;
        return false;
    }
    
    std::cout << "GPIO stub: Would setup interrupt on pin " << pin << " for edge " << (int)edge << std::endl;
    return true;
}

bool GPIOHelper::waitForEdge(unsigned int pin, int timeout_ms) {
    if (!initialized_.load()) {
        std::cerr << "GPIO stub: Not initialized" << std::endl;
        return false;
    }
    
    std::cout << "GPIO stub: Would wait for edge on pin " << pin << " with timeout " << timeout_ms << "ms" << std::endl;
    
    // Non-blocking timeout simulation for real-time systems
    if (timeout_ms > 0) {
        std::cout << "GPIO stub: Non-blocking timeout simulation (" << timeout_ms << "ms)" << std::endl;
        std::this_thread::yield();  // Non-blocking yield instead of usleep
    }
    
    return false;  // Always timeout in stub mode
}