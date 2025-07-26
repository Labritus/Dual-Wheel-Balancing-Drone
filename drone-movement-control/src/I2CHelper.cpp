#include "I2CHelper.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <iostream>
#include <cstring>

int I2CHelper::bus_fd_ = -1;
std::mutex I2CHelper::i2c_mutex_;
std::atomic<bool> I2CHelper::initialized_{false};
std::string I2CHelper::bus_device_;

bool I2CHelper::init(const std::string& device) {
    std::lock_guard<std::mutex> lock(i2c_mutex_);
    
    if (initialized_.load()) {
        return true;
    }
    
    bus_fd_ = open(device.c_str(), O_RDWR);
    if (bus_fd_ < 0) {
        std::cerr << "Failed to open I2C device: " << device << std::endl;
        return false;
    }
    
    bus_device_ = device;
    initialized_.store(true);
    return true;
}

void I2CHelper::shutdown() {
    std::lock_guard<std::mutex> lock(i2c_mutex_);
    
    if (!initialized_.load()) {
        return;
    }
    
    if (bus_fd_ >= 0) {
        close(bus_fd_);
        bus_fd_ = -1;
    }
    
    initialized_.store(false);
}

bool I2CHelper::setSlaveAddress(uint8_t deviceAddr) {
    if (ioctl(bus_fd_, I2C_SLAVE, deviceAddr >> 1) < 0) {
        std::cerr << "Failed to set I2C slave address: 0x" << std::hex << (deviceAddr >> 1) << std::endl;
        return false;
    }
    return true;
}

bool I2CHelper::writeByteToRegister(uint8_t deviceAddr, uint8_t registerAddr, uint8_t data) {
    std::lock_guard<std::mutex> lock(i2c_mutex_);
    
    if (!initialized_.load() || bus_fd_ < 0) {
        return false;
    }
    
    if (!setSlaveAddress(deviceAddr)) {
        return false;
    }
    
    uint8_t buffer[2] = {registerAddr, data};
    
    if (write(bus_fd_, buffer, 2) != 2) {
        std::cerr << "Failed to write byte to I2C register" << std::endl;
        return false;
    }
    
    return true;
}

uint8_t I2CHelper::readByteFromRegister(uint8_t deviceAddr, uint8_t registerAddr) {
    std::lock_guard<std::mutex> lock(i2c_mutex_);
    
    if (!initialized_.load() || bus_fd_ < 0) {
        return 0;
    }
    
    if (!setSlaveAddress(deviceAddr)) {
        return 0;
    }
    
    // Write register address
    if (write(bus_fd_, &registerAddr, 1) != 1) {
        std::cerr << "Failed to write register address" << std::endl;
        return 0;
    }
    
    // Read data
    uint8_t data = 0;
    if (read(bus_fd_, &data, 1) != 1) {
        std::cerr << "Failed to read from I2C register" << std::endl;
        return 0;
    }
    
    return data;
}

bool I2CHelper::readMultiBytesFromRegister(uint8_t deviceAddr, uint8_t registerAddr, uint8_t len, uint8_t* buffer) {
    std::lock_guard<std::mutex> lock(i2c_mutex_);
    
    if (!initialized_.load() || bus_fd_ < 0 || !buffer) {
        return false;
    }
    
    if (!setSlaveAddress(deviceAddr)) {
        return false;
    }
    
    // Write register address
    if (write(bus_fd_, &registerAddr, 1) != 1) {
        std::cerr << "Failed to write register address" << std::endl;
        return false;
    }
    
    // Read multiple bytes
    if (read(bus_fd_, buffer, len) != len) {
        std::cerr << "Failed to read multiple bytes from I2C" << std::endl;
        return false;
    }
    
    return true;
}

bool I2CHelper::writeMultiBytesToRegister(uint8_t deviceAddr, uint8_t registerAddr, uint8_t len, uint8_t* buffer) {
    std::lock_guard<std::mutex> lock(i2c_mutex_);
    
    if (!initialized_.load() || bus_fd_ < 0 || !buffer) {
        return false;
    }
    
    if (!setSlaveAddress(deviceAddr)) {
        return false;
    }
    
    // Create buffer with register address + data
    uint8_t* write_buffer = new uint8_t[len + 1];
    write_buffer[0] = registerAddr;
    std::memcpy(write_buffer + 1, buffer, len);
    
    bool success = (write(bus_fd_, write_buffer, len + 1) == (len + 1));
    
    delete[] write_buffer;
    
    if (!success) {
        std::cerr << "Failed to write multiple bytes to I2C" << std::endl;
        return false;
    }
    
    return true;
}