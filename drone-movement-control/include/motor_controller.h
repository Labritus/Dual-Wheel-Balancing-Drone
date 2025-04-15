#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "gpio_interface.h"
#include <cstdint>

class MotorController {
public:
    MotorController();
    ~MotorController();
    
    // 初始化电机控制器
    bool initialize();
    
    // 设置电机PWM值
    bool setPWM(int motor_left, int motor_right);
    
    // 停止电机
    void stop();
    
    // 检查控制器状态
    bool isInitialized() const;
    
private:
    GPIOInterface gpio_;
    
    // 限制PWM值在允许范围内
    int limitPWM(int pwm, int max_value, int min_value);
    
    // 电机引脚状态标志
    bool initialized_;
};

#endif // MOTOR_CONTROLLER_H 