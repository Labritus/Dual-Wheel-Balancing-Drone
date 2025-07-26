#ifndef __GPIOHELPER_HPP
#define __GPIOHELPER_HPP

#include <gpiod.h>
#include <memory>
#include <unordered_map>
#include <mutex>
#include <atomic>

// Forward declarations for libgpiod types
struct gpiod_chip;
struct gpiod_line_request;

enum class GPIOMode {
    INPUT,
    OUTPUT
};

enum class GPIOValue {
    LOW = 0,
    HIGH = 1
};

enum class GPIOEdge {
    NONE,
    RISING,
    FALLING,
    BOTH
};

class GPIOHelper {
private:
    static std::unique_ptr<gpiod_chip, void(*)(gpiod_chip*)> chip_;
    static std::unordered_map<unsigned int, gpiod_line_request*> requests_;
    static std::mutex gpio_mutex_;
    static std::atomic<bool> initialized_;
    
    static constexpr const char* DEFAULT_CHIP = "gpiochip0";
    static constexpr const char* CONSUMER = "drone-control";

public:
    static bool init(const char* chipname = DEFAULT_CHIP);
    static void shutdown();
    
    static bool setMode(unsigned int pin, GPIOMode mode);
    static bool setValue(unsigned int pin, GPIOValue value);
    static GPIOValue getValue(unsigned int pin);
    
    static bool setupInterrupt(unsigned int pin, GPIOEdge edge);
    static bool waitForEdge(unsigned int pin, int timeout_ms = -1);
    
    static bool isInitialized() { return initialized_.load(); }
};

#endif // __GPIOHELPER_HPP