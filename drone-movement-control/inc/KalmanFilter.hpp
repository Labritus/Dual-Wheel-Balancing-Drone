#ifndef __KALMANFILTER_HPP
#define __KALMANFILTER_HPP

#include "System.hpp"
#include <math.h>

#ifndef PI
#define PI 3.14159265f
#endif

// Forward declarations for C functions
extern "C" {
    float KF_X(float acce_Y, float acce_Z, float gyro_X);
    float KF_Y(float acce_X, float acce_Z, float gyro_Y);
    void mul(int A_row, int A_col, int B_row, int B_col, 
             float *A, float *B, float *C);
}

class KalmanFilter {
public:
    // Calculate angle from accelerometer readings
    static float angleCalculate(float accel_x, float accel_z);
    
    // Apply Kalman filter to compute X-axis angle
    static float angleXKalmanFilter(float accel_y, float accel_z, float gyro_x);
    
    // Apply Kalman filter to compute Y-axis angle
    static float angleYKalmanFilter(float accel_x, float accel_z, float gyro_y);
    
    // Apply Kalman filter to compute angle (simplified interface)
    static float angleKalmanFilter(float angle, float gyro);
    
    // Matrix multiplication helper
    static void matrixMultiply(int A_row, int A_col, int B_row, int B_col,
                              float A[][2], float B[][2], float C[][2]);
};

#endif // __KALMANFILTER_HPP