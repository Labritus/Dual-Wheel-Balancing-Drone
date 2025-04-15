#include "../include/i2c_interface.h"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstring>

I2CInterface::I2CInterface() : fd_(-1), address_(0), initialized_(false) {
}

I2CInterface::~I2CInterface() {
    close();
}

bool I2CInterface::initialize(const std::string& device, uint8_t address) {
    if (initialized_) {
        return true;
    }
    
    // 打开I2C设备
    fd_ = ::open(device.c_str(), O_RDWR);
    if (fd_ < 0) {
        std::cerr << "无法打开I2C设备：" << device << std::endl;
        return false;
    }
    
    address_ = address;
    
    // 设置I2C从设备地址
    if (ioctl(fd_, I2C_SLAVE, address_) < 0) {
        std::cerr << "无法设置I2C从设备地址：0x" << std::hex << (int)address_ << std::endl;
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    
    initialized_ = true;
    return true;
}

void I2CInterface::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    initialized_ = false;
}

bool I2CInterface::readByte(uint8_t reg, uint8_t* data) {
    if (!initialized_ || fd_ < 0) {
        return false;
    }
    
    if (write(fd_, &reg, 1) != 1) {
        return false;
    }
    
    if (read(fd_, data, 1) != 1) {
        return false;
    }
    
    return true;
}

bool I2CInterface::readBytes(uint8_t reg, uint8_t* data, size_t length) {
    if (!initialized_ || fd_ < 0) {
        return false;
    }
    
    if (write(fd_, &reg, 1) != 1) {
        return false;
    }
    
    if (read(fd_, data, length) != (ssize_t)length) {
        return false;
    }
    
    return true;
}

bool I2CInterface::writeByte(uint8_t reg, uint8_t data) {
    if (!initialized_ || fd_ < 0) {
        return false;
    }
    
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = data;
    
    if (write(fd_, buf, 2) != 2) {
        return false;
    }
    
    return true;
}

bool I2CInterface::writeBytes(uint8_t reg, uint8_t* data, size_t length) {
    if (!initialized_ || fd_ < 0) {
        return false;
    }
    
    uint8_t* buf = new uint8_t[length + 1];
    buf[0] = reg;
    memcpy(buf + 1, data, length);
    
    bool result = (write(fd_, buf, length + 1) == (ssize_t)(length + 1));
    
    delete[] buf;
    return result;
} 