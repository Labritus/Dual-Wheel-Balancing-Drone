#include <iostream>
#include <array>
#include <cmath>
#include <cstdint>
#include "MPU6050.h"
#include "IOI2C.h"
#include "usart.h"

constexpr int DEFAULT_MPU_HZ = 200;
constexpr float q30 = 1073741824.0f;
constexpr uint8_t MPU6050_DEFAULT_ADDRESS = 0x68;
constexpr float RAD_TO_DEG = 57.295779513f;

using namespace std;

class MPU6050 {
public:
    MPU6050(uint8_t address = MPU6050_DEFAULT_ADDRESS) 
        : devAddr(address), sensors(0), Flag_Show(false) {
        gyro.fill(0);
        accel.fill(0);
        q0 = 1.0f;
        q1 = q2 = q3 = 0.0f;
        buffer.fill(0);
    }

    bool initialize() {
        setClockSource(MPU6050_CLOCK_PLL_YGYRO);
        setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
        setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
        setSleepEnabled(false);
        setI2CMasterModeEnabled(false);
        setI2CBypassEnabled(false);
        return testConnection();
    }

    bool runSelfTest() {
        array<long, 3> gyroBias, accelBias;
        int result = mpu_run_self_test(gyroBias.data(), accelBias.data());
        if (result == 0x7) {
            float sens;
            unsigned short accel_sens;
            mpu_get_gyro_sens(&sens);
            for (int i = 0; i < 3; ++i) {
                gyroBias[i] = static_cast<long>(gyroBias[i] * sens);
            }
            dmp_set_gyro_bias(gyroBias.data());
            mpu_get_accel_sens(&accel_sens);
            for (int i = 0; i < 3; ++i) {
                accelBias[i] *= accel_sens;
            }
            dmp_set_accel_bias(accelBias.data());
            return true;
        }
        return false;
    }

    bool dmpInit() {
        uint8_t temp[1] = {0};
        Flag_Show = true;
        i2cRead(devAddr, 0x75, 1, temp);
        if (temp[0] != MPU6050_DEFAULT_ADDRESS) {
            NVIC_SystemReset();
            return false;
        }
        
        if (!mpu_init()) return false;
        if (!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL)) return false;
        if (!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL)) return false;
        if (!mpu_set_sample_rate(DEFAULT_MPU_HZ)) return false;
        if (!dmp_load_motion_driver_firmware()) return false;
        if (!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation.data()))) return false;
        if (!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT |
                                DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL)) return false;
        if (!dmp_set_fifo_rate(DEFAULT_MPU_HZ)) return false;
        if (!runSelfTest()) return false;
        if (!mpu_set_dmp_state(1)) return false;

        Flag_Show = false;
        return true;
    }

    bool readDMP() {
        unsigned long sensor_timestamp;
        unsigned char more;
        array<long, 4> quat;

        if (!dmp_read_fifo(gyro.data(), accel.data(), quat.data(), &sensor_timestamp, &sensors, &more)) {
            return false;
        }

        if (sensors & INV_WXYZ_QUAT) {
            q0 = quat[0] / q30;
            q1 = quat[1] / q30;
            q2 = quat[2] / q30;
            q3 = quat[3] / q30;

            Roll = asin(-2 * q1 * q3 + 2 * q0 * q2) * RAD_TO_DEG;
            Pitch = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * RAD_TO_DEG;
            Yaw = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * RAD_TO_DEG;
        }
        return true;
    }

    uint8_t getDeviceID() {
        i2cRead(devAddr, MPU6050_RA_WHO_AM_I, 1, buffer.data());
        return buffer[0];
    }

    bool testConnection() {
        return getDeviceID() == MPU6050_DEFAULT_ADDRESS;
    }

private:
    uint8_t devAddr;
    array<int16_t, 3> gyro, accel;
    array<signed char, 9> gyro_orientation = {-1, 0, 0, 0, -1, 0, 0, 0, 1};
    array<uint8_t, 14> buffer;
    float q0, q1, q2, q3;
    float Roll, Pitch, Yaw;
    uint8_t sensors;
    bool Flag_Show;

    void setClockSource(uint8_t source) {
        IICwriteBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
    }

    void setFullScaleGyroRange(uint8_t range) {
        IICwriteBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
    }

    void setFullScaleAccelRange(uint8_t range) {
        IICwriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
    }

    void setSleepEnabled(bool enabled) {
        IICwriteBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
    }

    void setI2CMasterModeEnabled(bool enabled) {
        IICwriteBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
    }

    void setI2CBypassEnabled(bool enabled) {
        IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
    }
};
