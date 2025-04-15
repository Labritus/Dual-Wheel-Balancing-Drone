#ifndef MPU6050_H
#define MPU6050_H

#include "i2c_interface.h"
#include <cstdint>
#include <array>
#include <string>

// MPU6050 register addresses
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

// Gyroscope range configuration
enum class GyroRange {
    RANGE_250_DEG = 0x00,
    RANGE_500_DEG = 0x08,
    RANGE_1000_DEG = 0x10,
    RANGE_2000_DEG = 0x18
};

// Accelerometer range configuration
enum class AccelRange {
    RANGE_2_G = 0x00,
    RANGE_4_G = 0x08,
    RANGE_8_G = 0x10,
    RANGE_16_G = 0x18
};

// Attitude data
struct AttitudeData {
    float roll;
    float pitch;
    float yaw;
};

class MPU6050 {
public:
    MPU6050();
    ~MPU6050();
    
    // Initialize the MPU6050
    bool initialize(const std::string& i2c_device);
    
    // Set the gyroscope range
    bool setGyroRange(GyroRange range);
    
    // Set the accelerometer range
    bool setAccelRange(AccelRange range);
    
    // Read raw accelerometer data
    std::array<int16_t, 3> readRawAccel();
    
    // Read raw gyroscope data
    std::array<int16_t, 3> readRawGyro();
    
    // Read temperature data
    float readTemperature();
    
    // Calculate attitude using a simple algorithm
    AttitudeData calculateAttitude();
    
    // Calibrate the sensors
    bool calibrateSensors();
    
    // Test if the sensor is connected properly
    bool testConnection();
    
private:
    I2CInterface i2c_;
    
    // Current sensor ranges
    GyroRange gyro_range_;
    AccelRange accel_range_;
    
    // Sensor calibration offsets
    std::array<int16_t, 3> accel_offset_;
    std::array<int16_t, 3> gyro_offset_;
    
    // Internal helper functions
    float accelSensitivity();
    float gyroSensitivity();
};

#endif // MPU6050_H
