#ifndef __MPU6050_HPP
#define __MPU6050_HPP

#include "System.hpp"
#include "IOI2C.hpp"

#define devAddr  0xD0

#define MPU6050_ADDRESS_AD0_LOW     0x68 // Address pin low level (GND)
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // Address pin high level (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW

// MPU6050 register address definitions
#define MPU6050_RA_XG_OFFS_TC       0x00
#define MPU6050_RA_YG_OFFS_TC       0x01
#define MPU6050_RA_ZG_OFFS_TC       0x02

#define MPU6050_RA_SMPLRT_DIV       0x19
#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C

#define MPU6050_RA_INT_PIN_CFG      0x37
#define MPU6050_RA_INT_ENABLE       0x38

#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40
#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_TEMP_OUT_L       0x42
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_GYRO_XOUT_L      0x44
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48
#define MPU6050_RA_FIFO_EN          0x23 // FIFO enable register
#define MPU6050_RA_FIFO_COUNT_H     0x72 // FIFO count high byte register
#define MPU6050_RA_FIFO_COUNT_L     0x73 // FIFO count low byte register
#define MPU6050_RA_FIFO_R_W         0x74 // FIFO read/write register
#define MPU6050_RA_BANK_SEL         0x6D
#define MPU6050_RA_USER_CTRL        0x6A
#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_PWR_MGMT_2       0x6C
#define MPU6050_RA_WHO_AM_I         0x75
#define DMP_MEMORY_SIZE        1929    // DMP firmware size (bytes)
#define CONFIG_SIZE            192     // DMP configuration data size (bytes)
#define UPDATES_SIZE           47      // DMP update data size (bytes)

// Gyroscope full-scale range definitions
#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x01
#define MPU6050_GYRO_FS_1000        0x02
#define MPU6050_GYRO_FS_2000        0x03

// Accelerometer full-scale range definitions
#define MPU6050_ACCEL_FS_2          0x00
#define MPU6050_ACCEL_FS_4          0x01
#define MPU6050_ACCEL_FS_8          0x02
#define MPU6050_ACCEL_FS_16         0x03

// DMP sensor data structure
struct MPU6050_DMP_Data {
    float pitch;   // Pitch angle
    float roll;    // Roll angle
    float yaw;     // Yaw angle
};

// MPU6050 class
#include "CallbackInterface.hpp"

class MPU6050 {
private:
    static SensorCallback* sensorCallback;

public:
    static void registerSensorCallback(SensorCallback* callback) { sensorCallback = callback; }

    // Initialize MPU6050
    static void initialize();
    
    // Test if MPU6050 connection is valid
    static uint8_t testConnection();
    
    // Get device ID
    static uint8_t getDeviceID();
    
    // Get 6-axis data (accelerometer + gyroscope)
    static uint8_t getData();
    
    // Get accelerometer data
    static void getAccel(int16_t* accel);
    
    // Get gyroscope data
    static void getGyro(int16_t* gyro);
    
    // Get temperature
    static int getTemperature();
    
    // Initialize DMP
    static void dmpInit();
    
    // Read DMP data
    static uint8_t getDMPData();
    
    // Set gyroscope range
    static void setGyroRange(uint8_t range);
    
    // Set accelerometer range
    static void setAccelRange(uint8_t range);
    
    // Initialize gyroscope offset
    static void initGyroOffset();
    
    // Enhanced DMP initialization
    static uint8_t dmpInitialize();
    
    // Set DMP FIFO rate
    static uint8_t setDMPFifoRate(uint16_t rate);
    
    // Get DMP FIFO rate
    static uint8_t getDMPFifoRate(uint16_t *rate);
    
    // Enable DMP features
    static uint8_t enableDMPFeature(uint16_t features);
    
    // Get enabled DMP features
    static uint8_t getEnabledDMPFeatures(uint16_t *features);
    
    // Set sensor orientation
    static uint8_t setDMPOrientation(uint16_t orient);
    
    // Set DMP gyroscope bias
    static uint8_t setDMPGyroBias(int32_t *bias);
    
    // Set DMP accelerometer bias
    static uint8_t setDMPAccelBias(int32_t *bias);
    
    // Enable low-power quaternion
    static uint8_t enableLPQuaternion(uint8_t enable);
    
    // Enable 6-axis quaternion
    static uint8_t enable6AxisQuaternion(uint8_t enable);
    
    // Enable gyroscope calibration
    static uint8_t enableGyroCalibration(uint8_t enable);
    
    // Pedometer functions
    static uint8_t getPedometerStepCount(uint32_t *count);
    static uint8_t setPedometerStepCount(uint32_t count);
    static uint8_t getPedometerWalkTime(uint32_t *time);
    static uint8_t setPedometerWalkTime(uint32_t time);
    
    // Enhanced read from DMP FIFO
    static uint8_t readDMPFifoPacket(uint8_t *packet, uint16_t length);
    static uint8_t readDMPFifo(int16_t *gyro, int16_t *accel, int32_t *quat, 
                               uint32_t *timestamp, int16_t *sensors, uint8_t *more);
private:
    // Read single byte data
    static uint8_t readByte(uint8_t regAddr);
    
    // Write single byte data
    static void writeByte(uint8_t regAddr, uint8_t data);
    
    // DMP memory read/write functions
    static uint8_t writeDMPMemory(uint16_t memAddr, uint16_t length, const uint8_t *buffer);
    static uint8_t readDMPMemory(uint16_t memAddr, uint16_t length, uint8_t *buffer);
    
    // Write DMP configuration set
    static uint8_t writeDMPConfigurationSet(const uint8_t *data, uint16_t size);
    
    // Get millisecond timestamp
    static void getMs(uint32_t *time);
    
    // DMP firmware data
    static const uint8_t dmpMemory[DMP_MEMORY_SIZE];
    static const uint16_t dmpMemorySize;
    
    // DMP configuration data
    static const uint8_t dmpConfig[CONFIG_SIZE];
    static const uint16_t dmpConfigSize;
    
    // DMP packet size
    static const uint8_t dmpPacketSize;
    
    // DMP feature flags
    static uint16_t dmpFeatureFlags;
    
    // Millisecond counter
    static uint32_t msCounter;
    
    // DMP operation related functions
    static uint8_t writeProgDMPConfigurationSet(const uint8_t *data, uint16_t size);
    static uint8_t resetDMP();
};

// DMP feature definitions
#define DMP_FEATURE_TAP             0x001
#define DMP_FEATURE_ANDROID_ORIENT  0x002
#define DMP_FEATURE_LP_QUAT         0x004
#define DMP_FEATURE_PEDOMETER       0x008
#define DMP_FEATURE_6X_LP_QUAT      0x010
#define DMP_FEATURE_GYRO_CAL        0x020
#define DMP_FEATURE_SEND_RAW_ACCEL  0x040
#define DMP_FEATURE_SEND_RAW_GYRO   0x080
#define DMP_FEATURE_SEND_CAL_GYRO   0x100

// DMP orientation definitions
#define ORIENT_PORTRAIT             0x00
#define ORIENT_LANDSCAPE            0x01
#define ORIENT_REVERSE_PORTRAIT     0x02
#define ORIENT_REVERSE_LANDSCAPE    0x03

#define MPU6050_RA_MEM_START_ADDR   0x6E // DMP memory start address register
#define MPU6050_RA_MEM_R_W          0x6F // DMP memory read/write register

// DMP parameter structure
struct DMP_InitTypeDef {
    uint16_t features;        // Enabled DMP features
    uint16_t fifoRate;        // DMP FIFO output rate (Hz)
    uint16_t orientation;     // Sensor orientation
    uint8_t  lpAccelMode;     // Low-power accelerometer mode
};

// Quaternion data
extern float MPU6050_Quat[4];     // Quaternion: w, x, y, z
extern uint32_t MPU6050_StepCount; // Pedometer step count
extern uint32_t MPU6050_WalkTime;  // Walk time

// DMP sensor data
extern MPU6050_DMP_Data MPU6050Data;
extern int16_t accel[3], gyro[3];
extern float Pitch, Roll, Yaw;

#endif // __MPU6050_HPP
