#ifndef __I2CHELPER_HPP
#define __I2CHELPER_HPP

#include <cstdint>
#include <string>
#include <mutex>
#include <atomic>

class I2CHelper {
private:
    static int bus_fd_;
    static std::mutex i2c_mutex_;
    static std::atomic<bool> initialized_;
    static std::string bus_device_;

public:
    static bool init(const std::string& device = "/dev/i2c-1");
    static void shutdown();
    
    static bool writeByteToRegister(uint8_t deviceAddr, uint8_t registerAddr, uint8_t data);
    static uint8_t readByteFromRegister(uint8_t deviceAddr, uint8_t registerAddr);
    static bool readMultiBytesFromRegister(uint8_t deviceAddr, uint8_t registerAddr, uint8_t len, uint8_t* buffer);
    static bool writeMultiBytesToRegister(uint8_t deviceAddr, uint8_t registerAddr, uint8_t len, uint8_t* buffer);
    
    static bool isInitialized() { return initialized_.load(); }

private:
    static bool setSlaveAddress(uint8_t deviceAddr);
};

#endif // __I2CHELPER_HPP