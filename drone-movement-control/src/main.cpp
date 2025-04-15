#include <iostream>
#include <thread>
#include <signal.h>
#include <fstream>
#include <mutex>
#include <iomanip>
#include <chrono>

#include "../include/drone_controller.h"
#include "../include/drone_status_interface.h"

// 全局变量
bool running = true;
std::ofstream log_file;
std::mutex log_mutex;
DroneController drone_controller;
DroneStatusInterface& status_interface = DroneStatusInterface::getInstance();

// 处理Ctrl+C信号
void signalHandler(int signum) {
    std::cout << "中断信号 (" << signum << ") 已接收，正在关闭程序..." << std::endl;
    running = false;
}

// 记录消息到控制台和文件
void logMessage(const std::string& message) {
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    
    std::stringstream timestamp;
    timestamp << std::put_time(std::localtime(&now_time_t), "%Y-%m-%d %H:%M:%S");
    timestamp << '.' << std::setfill('0') << std::setw(3) << now_ms.count();
    
    std::string log_entry = "[" + timestamp.str() + "] " + message;
    
    // 线程安全的日志记录
    std::lock_guard<std::mutex> lock(log_mutex);
    std::cout << log_entry << std::endl;
    
    if (log_file.is_open()) {
        log_file << log_entry << std::endl;
    }
}

// 姿态更新回调
void onAttitudeUpdate(float roll, float pitch, float yaw) {
    static int counter = 0;
    static float last_roll = 0;
    static float last_pitch = 0;
    
    // 只有在姿态变化超过阈值或每50次更新时才记录
    if (counter++ % 50 == 0 || 
        std::abs(roll - last_roll) > 5.0f || 
        std::abs(pitch - last_pitch) > 5.0f) {
        
        std::stringstream ss;
        ss << "姿态更新: Roll=" << std::fixed << std::setprecision(2) << roll
           << "°, Pitch=" << pitch << "°, Yaw=" << yaw << "°";
        logMessage(ss.str());
        
        // 如果姿态角度超过警告阈值，记录警告
        if (std::abs(roll) > 30.0f || std::abs(pitch) > 30.0f) {
            status_interface.addEvent("姿态警告", "姿态角度超过安全范围");
            logMessage("警告: 姿态角度超过安全范围!");
        }
        
        last_roll = roll;
        last_pitch = pitch;
    }
    
    // 更新状态接口中的姿态数据
    DroneStatus current_status = status_interface.getStatus();
    current_status.roll = roll;
    current_status.pitch = pitch;
    current_status.yaw = yaw;
    status_interface.updateStatus(current_status);
}

// 电池更新回调
void onBatteryUpdate(int voltage_mv) {
    static int last_voltage = 0;
    
    // 只有在电压变化超过阈值时才记录
    if (std::abs(voltage_mv - last_voltage) > 100) {
        std::stringstream ss;
        ss << "电池电压: " << voltage_mv << "mV";
        logMessage(ss.str());
        
        if (voltage_mv < 11000) {
            status_interface.addEvent("电池警告", "电池电压低于11V");
            logMessage("警告: 电池电压低!");
        }
        
        last_voltage = voltage_mv;
    }
    
    // 更新状态接口中的电池数据
    DroneStatus current_status = status_interface.getStatus();
    current_status.battery_voltage = voltage_mv;
    status_interface.updateStatus(current_status);
}

// 电机输出回调
void onMotorOutput(int left_pwm, int right_pwm) {
    static int last_left_pwm = 0;
    static int last_right_pwm = 0;
    
    // 只有在PWM值变化超过阈值时才记录
    if (std::abs(left_pwm - last_left_pwm) > 500 || std::abs(right_pwm - last_right_pwm) > 500) {
        std::stringstream ss;
        ss << "电机输出: 左PWM=" << left_pwm << ", 右PWM=" << right_pwm;
        logMessage(ss.str());
        
        last_left_pwm = left_pwm;
        last_right_pwm = right_pwm;
    }
    
    // 更新状态接口中的电机数据
    DroneStatus current_status = status_interface.getStatus();
    current_status.motor_left_pwm = left_pwm;
    current_status.motor_right_pwm = right_pwm;
    status_interface.updateStatus(current_status);
}

// 错误回调
void onError(const std::string& error_message, int error_code) {
    std::stringstream ss;
    ss << "错误 [" << error_code << "]: " << error_message;
    logMessage(ss.str());
    
    // 记录错误事件
    status_interface.addEvent("系统错误", ss.str());
}

// 状态变化回调
void onStateChange(const std::string& state_name, bool state_active) {
    std::stringstream ss;
    ss << "状态变化: " << state_name << " 现在" << (state_active ? "激活" : "停用");
    logMessage(ss.str());
    
    // 记录状态变化事件
    status_interface.addEvent("状态变化", ss.str());
    
    // 更新状态接口中的状态描述
    DroneStatus current_status = status_interface.getStatus();
    current_status.status_description = state_name + (state_active ? " 激活" : " 停用");
    status_interface.updateStatus(current_status);
}

// 状态导出回调函数 - 用于显示状态摘要
void onStatusExport(const DroneStatus& status) {
    static int counter = 0;
    
    // 每50次更新打印一次状态摘要
    if (counter++ % 50 == 0) {
        std::stringstream ss;
        ss << "状态摘要: "
           << "Roll=" << std::fixed << std::setprecision(2) << status.roll
           << "°, Pitch=" << status.pitch
           << "°, 电池=" << status.battery_voltage
           << "mV, 位置=" << status.position
           << ", 电机=" << status.motor_left_pwm << "/" << status.motor_right_pwm;
        logMessage(ss.str());
    }
}

int main() {
    // 注册信号处理函数
    signal(SIGINT, signalHandler);
    
    // 打开日志文件
    log_file.open("drone_log.txt", std::ios::out | std::ios::app);
    if (!log_file.is_open()) {
        std::cerr << "无法打开日志文件" << std::endl;
        return 1;
    }
    
    logMessage("无人机控制系统启动");
    
    // 注册回调函数
    drone_controller.registerAttitudeCallback(onAttitudeUpdate);
    drone_controller.registerBatteryCallback(onBatteryUpdate);
    drone_controller.registerMotorOutputCallback(onMotorOutput);
    drone_controller.registerErrorCallback(onError);
    drone_controller.registerStateChangeCallback(onStateChange);
    
    // 注册状态导出回调
    status_interface.registerExportCallback(onStatusExport);
    
    // 启动CSV和JSON记录
    status_interface.startCSVRecording("drone_status.csv");
    status_interface.startJSONRecording("drone_events.json");
    
    // 初始化硬件
    if (!drone_controller.initializeHardware()) {
        logMessage("硬件初始化失败，程序退出");
        log_file.close();
        return 1;
    }
    
    logMessage("系统初始化完成，开始控制循环");
    
    // 设置目标位置
    drone_controller.setTargetPosition(1000);
    
    // 主控制循环
    int counter = 0;
    while (running) {
        // 更新控制器
        drone_controller.update();
        
        // 每100个循环显示一次状态
        if (counter++ % 100 == 0) {
            // 更新DroneStatus数据
            DroneStatus current_status = status_interface.getStatus();
            current_status.position = drone_controller.getCurrentPosition();
            current_status.velocity = drone_controller.getCurrentVelocity();
            current_status.position_mode = drone_controller.isPositionMode();
            status_interface.updateStatus(current_status);
            
            // 显示状态
            drone_controller.displayStats();
        }
        
        // 添加一些随机事件以演示事件记录
        if (counter % 1000 == 0) {
            status_interface.addEvent("循环事件", "完成了1000次控制循环");
            logMessage("已完成1000次控制循环");
        }
        
        // 短暂休眠，减少CPU使用率
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    // 停止电机
    drone_controller.stopMotors();
    
    // 停止记录
    status_interface.stopCSVRecording();
    status_interface.stopJSONRecording();
    
    logMessage("系统正常关闭");
    
    // 关闭日志文件
    log_file.close();
    
    return 0;
} 