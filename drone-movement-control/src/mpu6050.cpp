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
    // Close the I2C interface
    i2c_.close();
}

bool MPU6050::initialize(const std::string& i2c_device) {
    // Initialize the I2C interface
    if (!i2c_.initialize(i2c_device, MPU6050_I2C_ADDRESS)) {
        std::cerr << "Failed to initialize I2C interface" << std::endl;
        return false;
    }
    
    // Wake up the MPU6050
    if (!i2c_.writeByte(MPU6050_REG_PWR_MGMT_1, 0x00)) {
        std::cerr << "Failed to wake up MPU6050" << std::endl;
        return false;
    }
    
    // Check the device ID
    if (!testConnection()) {
        std::cerr << "MPU6050 connection test failed" << std::endl;
        return false;
    }
    
    // Set the sample rate divider
    if (!i2c_.writeByte(MPU6050_REG_SMPLRT_DIV, 0x07)) {
        return false;
    }
    
    // Set the configuration register (DLPF setting)
    if (!i2c_.writeByte(MPU6050_REG_CONFIG, 0x06)) {
        return false;
    }
    
    // Set the gyroscope range
    if (!setGyroRange(GyroRange::RANGE_250_DEG)) {
        return false;
    }
    
    // Set the accelerometer range
    if (!setAccelRange(AccelRange::RANGE_2_G)) {
        return false;
    }
    
    // Disable FIFO
    if (!i2c_.writeByte(MPU6050_REG_FIFO_EN, 0x00)) {
        return false;
    }
    
    // Disable interrupts
    if (!i2c_.writeByte(MPU6050_REG_INT_ENABLE, 0x00)) {
        return false;
    }
    
    // After complete initialization, perform sensor calibration
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
    
    // Apply calibration offset
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
    
    // Apply calibration offset
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
    
    // Convert raw temperature using the formula from the datasheet
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
    
    // Calculate roll and pitch from accelerometer data (in radians)
    attitude.roll = atan2(accel_y, sqrt(accel_x * accel_x + accel_z * accel_z));
    attitude.pitch = atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z));
    
    // Convert to degrees
    attitude.roll *= 180.0f / M_PI;
    attitude.pitch *= 180.0f / M_PI;
    
    // Note: This simple calculation does not provide a yaw angle; a magnetometer or complex sensor fusion algorithms are needed.
    attitude.yaw = 0.0f;
    
    return attitude;
}

bool MPU6050::calibrateSensors() {
    std::cout << "Calibrating sensors, please ensure the device is stationary..." << std::endl;
    
    const int num_samples = 100;
    std::array<int32_t, 3> accel_sum = {0, 0, 0};
    std::array<int32_t, 3> gyro_sum = {0, 0, 0};
    
    // Collect multiple samples
    for (int i = 0; i < num_samples; i++) {
        auto accel = readRawAccel();
        auto gyro = readRawGyro();
        
        for (int j = 0; j < 3; j++) {
            accel_sum[j] += accel[j];
            gyro_sum[j] += gyro[j];
        }
        
        usleep(10000); // Wait 10ms
    }
    
    // Calculate the average as the offset
    for (int i = 0; i < 3; i++) {
        accel_offset_[i] = accel_sum[i] / num_samples;
        gyro_offset_[i] = gyro_sum[i] / num_samples;
    }
    
    // The accelerometer's Z-axis should read 1g (earth gravity), so adjust the Z-axis offset accordingly
    accel_offset_[2] -= static_cast<int16_t>(accelSensitivity());
    
    std::cout << "Sensor calibration completed" << std::endl;
    
    return true;
}

bool MPU6050::testConnection() {
    uint8_t who_am_i;
    if (!i2c_.readByte(MPU6050_REG_WHO_AM_I, &who_am_i)) {
        return false;
    }
    
    return (who_am_i == 0x68); // MPU6050 should return 0x68
}
