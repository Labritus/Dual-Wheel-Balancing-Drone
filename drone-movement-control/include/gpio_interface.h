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
    
    // 初始化GPIO引脚
    bool initialize(int pin, PinMode mode);
    
    // 释放GPIO引脚
    void release(int pin);
    
    // 设置数字输出值
    bool digitalWrite(int pin, PinState state);
    
    // 读取数字输入值
    PinState digitalRead(int pin);
    
    // 设置PWM值 (0-255)
    bool setPWM(int pin, int value);
    
    // 关闭并释放所有资源
    void cleanup();
    
private:
    // 将引脚导出到sysfs
    bool exportPin(int pin);
    
    // 设置引脚方向
    bool setDirection(int pin, PinMode mode);
    
    // 已初始化的引脚
    int* initialized_pins_;
    int num_pins_;
};

#endif // GPIO_INTERFACE_H 