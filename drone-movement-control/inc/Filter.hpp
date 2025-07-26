#ifndef __COMPLEMENTARY_FILTER_HPP
#define __COMPLEMENTARY_FILTER_HPP

#include "System.hpp"
#include "IOI2C.hpp"

// Complementary Filter class
class ComplementaryFilter {
public:
    // Complementary filter for X-axis
    static float angleTempFilterX(float angle_m, float gyro_m);
    
    // Complementary filter for Y-axis
    static float angleTempFilterY(float angle_m, float gyro_m);
};

// Kalman Filter class
class KalmanFilter {
public:
    static float Kalman_Filter_x(float Accel, float Gyro);
    static float Kalman_Filter_y(float Accel, float Gyro);
    
    // Calculate angle from accelerometer
    static float angleCalculate(float acc_x, float acc_z);
};

#endif

#ifndef PI
#define PI 3.14159265358979323846f
#endif