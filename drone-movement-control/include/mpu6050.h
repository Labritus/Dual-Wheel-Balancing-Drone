#ifndef MPU6050_H
#define MPU6050_H

#include "i2c_interface.h"
#include <cstdint>
#include <array>
#include <string>

// MPU6050寄存器地址
constexpr uint8_t MPU6050_REG_SMPLRT_DIV = 0x19;
constexpr uint8_t MPU6050_REG_CONFIG = 0x1A;
constexpr uint8_t MPU6050_REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t MPU6050_REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t MPU6050_REG_FIFO_EN = 0x23;
constexpr uint8_t MPU6050_REG_INT_PIN_CFG = 0x37;
constexpr uint8_t MPU6050_REG_INT_ENABLE = 0x38;
constexpr uint8_t MPU6050_REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t MPU6050_REG_TEMP_OUT_H = 0x41;
constexpr uint8_t MPU6050_REG_GYRO_XOUT_H = 0x43;
constexpr uint8_t MPU6050_REG_USER_CTRL = 0x6A;
constexpr uint8_t MPU6050_REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t MPU6050_REG_PWR_MGMT_2 = 0x6C;
constexpr uint8_t MPU6050_REG_WHO_AM_I = 0x75;

// 陀螺仪量程配置
enum class GyroRange {
    RANGE_250_DEG = 0x00,
    RANGE_500_DEG = 0x08,
    RANGE_1000_DEG = 0x10,
    RANGE_2000_DEG = 0x18
};

// 加速度计量程配置
enum class AccelRange {
    RANGE_2_G = 0x00,
    RANGE_4_G = 0x08,
    RANGE_8_G = 0x10,
    RANGE_16_G = 0x18
};

// 姿态数据
struct AttitudeData {
    float roll;
    float pitch;
    float yaw;
};

class MPU6050 {
public:
    MPU6050();
    ~MPU6050();
    
    // 初始化MPU6050
    bool initialize(const std::string& i2c_device);
    
    // 设置陀螺仪量程
    bool setGyroRange(GyroRange range);
    
    // 设置加速度计量程
    bool setAccelRange(AccelRange range);
    
    // 读取原始加速度数据
    std::array<int16_t, 3> readRawAccel();
    
    // 读取原始陀螺仪数据
    std::array<int16_t, 3> readRawGyro();
    
    // 读取温度数据
    float readTemperature();
    
    // 通过简单算法计算姿态
    AttitudeData calculateAttitude();
    
    // 校准传感器
    bool calibrateSensors();
    
    // 检查传感器是否连接正常
    bool testConnection();
    
private:
    I2CInterface i2c_;
    
    // 当前传感器量程
    GyroRange gyro_range_;
    AccelRange accel_range_;
    
    // 传感器校准偏移值
    std::array<int16_t, 3> accel_offset_;
    std::array<int16_t, 3> gyro_offset_;
    
    // 内部辅助函数
    float accelSensitivity();
    float gyroSensitivity();
};

#endif // MPU6050_H 