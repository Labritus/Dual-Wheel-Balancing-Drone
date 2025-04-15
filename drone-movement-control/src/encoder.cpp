#include "../include/encoder.h"
#include <iostream>
#include <unistd.h>

Encoder::Encoder() : 
    pin_a_(-1), 
    pin_b_(-1), 
    count_(0), 
    velocity_(0), 
    running_(false),
    last_state_a_(0),
    last_state_b_(0)
{
}

Encoder::~Encoder() {
    stop();
}

bool Encoder::initialize(int pin_a, int pin_b) {
    // 保存引脚编号
    pin_a_ = pin_a;
    pin_b_ = pin_b;
    
    // 初始化GPIO引脚
    if (!gpio_.initialize(pin_a_, PinMode::INPUT) ||
        !gpio_.initialize(pin_b_, PinMode::INPUT)) {
        std::cerr << "无法初始化编码器引脚" << std::endl;
        return false;
    }
    
    // 获取初始状态
    last_state_a_ = gpio_.digitalRead(pin_a_) == PinState::HIGH ? 1 : 0;
    last_state_b_ = gpio_.digitalRead(pin_b_) == PinState::HIGH ? 1 : 0;
    
    // 重置计数器
    resetCount();
    
    return true;
}

int Encoder::getCount() const {
    return count_;
}

int Encoder::getVelocity() const {
    return velocity_;
}

void Encoder::resetCount() {
    count_ = 0;
    velocity_ = 0;
    last_time_ = std::chrono::steady_clock::now();
}

void Encoder::start() {
    if (running_) {
        return;
    }
    
    running_ = true;
    monitor_thread_ = std::thread(&Encoder::monitorPins, this);
}

void Encoder::stop() {
    if (!running_) {
        return;
    }
    
    running_ = false;
    if (monitor_thread_.joinable()) {
        monitor_thread_.join();
    }
}

bool Encoder::isRunning() const {
    return running_;
}

void Encoder::monitorPins() {
    int last_count = 0;
    auto last_velocity_update = std::chrono::steady_clock::now();
    
    while (running_) {
        // 读取当前状态
        int state_a = gpio_.digitalRead(pin_a_) == PinState::HIGH ? 1 : 0;
        int state_b = gpio_.digitalRead(pin_b_) == PinState::HIGH ? 1 : 0;
        
        // 检测状态变化
        if (state_a != last_state_a_) {
            // A相变化
            if (state_b != state_a) {
                // 顺时针旋转
                count_++;
            } else {
                // 逆时针旋转
                count_--;
            }
        } else if (state_b != last_state_b_) {
            // B相变化
            if (state_a == state_b) {
                // 顺时针旋转
                count_++;
            } else {
                // 逆时针旋转
                count_--;
            }
        }
        
        // 保存当前状态
        last_state_a_ = state_a;
        last_state_b_ = state_b;
        
        // 每100ms更新一次速度
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_velocity_update).count();
        
        if (elapsed >= 100) {
            int count_diff = count_ - last_count;
            velocity_ = count_diff * 1000 / elapsed; // 计数/秒
            
            last_count = count_;
            last_velocity_update = now;
        }
        
        // 短暂休眠以避免CPU占用过高
        usleep(1000); // 1ms
    }
} 