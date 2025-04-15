#include "../include/gpio_interface.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>

// 最大可初始化引脚数
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
    // 检查引脚是否已经初始化
    for (int i = 0; i < num_pins_; i++) {
        if (initialized_pins_[i] == pin) {
            return true; // 已经初始化
        }
    }
    
    // 导出引脚
    if (!exportPin(pin)) {
        return false;
    }
    
    // 设置引脚方向
    if (!setDirection(pin, mode)) {
        return false;
    }
    
    // 添加到初始化列表
    if (num_pins_ < MAX_PINS) {
        initialized_pins_[num_pins_++] = pin;
        return true;
    }
    
    return false;
}

void GPIOInterface::release(int pin) {
    std::ofstream unexport_file("/sys/class/gpio/unexport");
    if (!unexport_file.is_open()) {
        std::cerr << "无法打开unexport文件" << std::endl;
        return;
    }
    
    unexport_file << pin;
    unexport_file.close();
    
    // 从已初始化列表中移除
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
        std::cerr << "无法打开GPIO值文件：" << ss.str() << std::endl;
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
        std::cerr << "无法打开GPIO值文件：" << ss.str() << std::endl;
        return PinState::LOW; // 默认返回低电平
    }
    
    char value;
    value_file >> value;
    value_file.close();
    
    return (value == '1') ? PinState::HIGH : PinState::LOW;
}

bool GPIOInterface::setPWM(int pin, int value) {
    // 这里应该根据硬件特性实现PWM控制
    // 对于简单的示例，我们可以使用软件模拟PWM
    // 但实际上应该使用硬件PWM功能
    
    std::cout << "PWM功能尚未实现，设置引脚 " << pin << " 值为 " << value << std::endl;
    
    // 对于非硬件PWM实现，可以考虑使用软件PWM或者简单地设置数字输出
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
        std::cerr << "无法打开export文件" << std::endl;
        return false;
    }
    
    export_file << pin;
    export_file.close();
    
    // 等待GPIO文件系统创建完成
    usleep(100000); // 等待100ms
    
    return true;
}

bool GPIOInterface::setDirection(int pin, PinMode mode) {
    std::stringstream ss;
    ss << "/sys/class/gpio/gpio" << pin << "/direction";
    
    std::ofstream direction_file(ss.str());
    if (!direction_file.is_open()) {
        std::cerr << "无法打开GPIO方向文件：" << ss.str() << std::endl;
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