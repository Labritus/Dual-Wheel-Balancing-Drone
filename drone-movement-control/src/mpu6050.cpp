#include "../include/mpu6050.h"
#include "../include/config.h"
#include <iostream>
#include <cmath>
#include <unistd.h>

MPU6050::MPU6050() : 
    gyro_range_(GyroRange::RANGE_250_DEG),
    accel_range_(AccelRange::RANGE_2_G)
{
    accel_offset_ = {0, 0, 0};
    gyro_offset_ = {0, 0, 0};
}

MPU6050::~MPU6050() {
    // 关闭I2C接口
    i2c_.close();
}

bool MPU6050::initialize(const std::string& i2c_device) {
    // 初始化I2C接口
    if (!i2c_.initialize(i2c_device, MPU6050_I2C_ADDRESS)) {
        std::cerr << "无法初始化I2C接口" << std::endl;
        return false;
    }
    
    // 唤醒MPU6050
    if (!i2c_.writeByte(MPU6050_REG_PWR_MGMT_1, 0x00)) {
        std::cerr << "无法唤醒MPU6050" << std::endl;
        return false;
    }
    
    // 检查设备ID
    if (!testConnection()) {
        std::cerr << "MPU6050连接测试失败" << std::endl;
        return false;
    }
    
    // 设置采样率分频器
    if (!i2c_.writeByte(MPU6050_REG_SMPLRT_DIV, 0x07)) {
        return false;
    }
    
    // 设置配置寄存器 (DLPF设置)
    if (!i2c_.writeByte(MPU6050_REG_CONFIG, 0x06)) {
        return false;
    }
    
    // 设置陀螺仪量程
    if (!setGyroRange(GyroRange::RANGE_250_DEG)) {
        return false;
    }
    
    // 设置加速度计量程
    if (!setAccelRange(AccelRange::RANGE_2_G)) {
        return false;
    }
    
    // 关闭FIFO
    if (!i2c_.writeByte(MPU6050_REG_FIFO_EN, 0x00)) {
        return false;
    }
    
    // 关闭中断
    if (!i2c_.writeByte(MPU6050_REG_INT_ENABLE, 0x00)) {
        return false;
    }
    
    // 完全初始化成功后，执行校准
    return calibrateSensors();
}

bool MPU6050::setGyroRange(GyroRange range) {
    if (!i2c_.writeByte(MPU6050_REG_GYRO_CONFIG, static_cast<uint8_t>(range))) {
        return false;
    }
    gyro_range_ = range;
    return true;
}

bool MPU6050::setAccelRange(AccelRange range) {
    if (!i2c_.writeByte(MPU6050_REG_ACCEL_CONFIG, static_cast<uint8_t>(range))) {
        return false;
    }
    accel_range_ = range;
    return true;
}

std::array<int16_t, 3> MPU6050::readRawAccel() {
    uint8_t buffer[6];
    std::array<int16_t, 3> accel = {0, 0, 0};
    
    if (!i2c_.readBytes(MPU6050_REG_ACCEL_XOUT_H, buffer, 6)) {
        return accel;
    }
    
    accel[0] = (buffer[0] << 8) | buffer[1];
    accel[1] = (buffer[2] << 8) | buffer[3];
    accel[2] = (buffer[4] << 8) | buffer[5];
    
    // 应用校准偏移
    accel[0] -= accel_offset_[0];
    accel[1] -= accel_offset_[1];
    accel[2] -= accel_offset_[2];
    
    return accel;
}

std::array<int16_t, 3> MPU6050::readRawGyro() {
    uint8_t buffer[6];
    std::array<int16_t, 3> gyro = {0, 0, 0};
    
    if (!i2c_.readBytes(MPU6050_REG_GYRO_XOUT_H, buffer, 6)) {
        return gyro;
    }
    
    gyro[0] = (buffer[0] << 8) | buffer[1];
    gyro[1] = (buffer[2] << 8) | buffer[3];
    gyro[2] = (buffer[4] << 8) | buffer[5];
    
    // 应用校准偏移
    gyro[0] -= gyro_offset_[0];
    gyro[1] -= gyro_offset_[1];
    gyro[2] -= gyro_offset_[2];
    
    return gyro;
}

float MPU6050::readTemperature() {
    uint8_t buffer[2];
    if (!i2c_.readBytes(MPU6050_REG_TEMP_OUT_H, buffer, 2)) {
        return 0;
    }
    
    int16_t raw_temp = (buffer[0] << 8) | buffer[1];
    
    // 根据数据手册的温度转换公式
    return (raw_temp / 340.0f) + 36.53f;
}

float MPU6050::accelSensitivity() {
    switch (accel_range_) {
        case AccelRange::RANGE_2_G:
            return 16384.0f;
        case AccelRange::RANGE_4_G:
            return 8192.0f;
        case AccelRange::RANGE_8_G:
            return 4096.0f;
        case AccelRange::RANGE_16_G:
            return 2048.0f;
        default:
            return 16384.0f;
    }
}

float MPU6050::gyroSensitivity() {
    switch (gyro_range_) {
        case GyroRange::RANGE_250_DEG:
            return 131.0f;
        case GyroRange::RANGE_500_DEG:
            return 65.5f;
        case GyroRange::RANGE_1000_DEG:
            return 32.8f;
        case GyroRange::RANGE_2000_DEG:
            return 16.4f;
        default:
            return 131.0f;
    }
}

AttitudeData MPU6050::calculateAttitude() {
    AttitudeData attitude = {0.0f, 0.0f, 0.0f};
    
    auto accel = readRawAccel();
    auto gyro = readRawGyro();
    
    float accel_x = accel[0] / accelSensitivity();
    float accel_y = accel[1] / accelSensitivity();
    float accel_z = accel[2] / accelSensitivity();
    
    // 简单地根据加速度计计算roll和pitch (弧度)
    attitude.roll = atan2(accel_y, sqrt(accel_x * accel_x + accel_z * accel_z));
    attitude.pitch = atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z));
    
    // 转换为角度
    attitude.roll *= 180.0f / M_PI;
    attitude.pitch *= 180.0f / M_PI;
    
    // 注意：这个简单的计算无法提供yaw角度，需要磁力计或者复杂的传感器融合算法
    attitude.yaw = 0.0f;
    
    return attitude;
}

bool MPU6050::calibrateSensors() {
    std::cout << "校准传感器中，请确保设备静止..." << std::endl;
    
    const int num_samples = 100;
    std::array<int32_t, 3> accel_sum = {0, 0, 0};
    std::array<int32_t, 3> gyro_sum = {0, 0, 0};
    
    // 收集多个样本
    for (int i = 0; i < num_samples; i++) {
        auto accel = readRawAccel();
        auto gyro = readRawGyro();
        
        for (int j = 0; j < 3; j++) {
            accel_sum[j] += accel[j];
            gyro_sum[j] += gyro[j];
        }
        
        usleep(10000); // 等待10ms
    }
    
    // 计算平均值作为偏移量
    for (int i = 0; i < 3; i++) {
        accel_offset_[i] = accel_sum[i] / num_samples;
        gyro_offset_[i] = gyro_sum[i] / num_samples;
    }
    
    // Z轴加速度计应该读取到1g (地球重力)，所以我们调整z轴偏移
    accel_offset_[2] -= static_cast<int16_t>(accelSensitivity());
    
    std::cout << "传感器校准完成" << std::endl;
    
    return true;
}

bool MPU6050::testConnection() {
    uint8_t who_am_i;
    if (!i2c_.readByte(MPU6050_REG_WHO_AM_I, &who_am_i)) {
        return false;
    }
    
    return (who_am_i == 0x68); // MPU6050应返回0x68
} 