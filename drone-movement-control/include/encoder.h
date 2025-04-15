#ifndef ENCODER_H
#define ENCODER_H

#include "gpio_interface.h"
#include <thread>
#include <atomic>
#include <chrono>

class Encoder {
public:
    Encoder();
    ~Encoder();
    
    // 初始化编码器
    bool initialize(int pin_a, int pin_b);
    
    // 读取当前计数值
    int getCount() const;
    
    // 读取当前速度 (计数/秒)
    int getVelocity() const;
    
    // 重置计数器
    void resetCount();
    
    // 开始计数
    void start();
    
    // 停止计数
    void stop();
    
    // 检查编码器是否在运行
    bool isRunning() const;
    
private:
    // 引脚编号
    int pin_a_;
    int pin_b_;
    
    // GPIO接口
    GPIOInterface gpio_;
    
    // 计数器和速度
    std::atomic<int> count_;
    std::atomic<int> velocity_;
    
    // 运行状态
    std::atomic<bool> running_;
    
    // 上次状态
    int last_state_a_;
    int last_state_b_;
    
    // 用于计算速度的时间戳
    std::chrono::steady_clock::time_point last_time_;
    
    // 监控线程
    std::thread monitor_thread_;
    
    // 监控函数
    void monitorPins();
};

#endif // ENCODER_H 