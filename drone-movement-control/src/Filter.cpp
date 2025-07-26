#include "Filter.hpp"
#include "math.h"

// Define static constants
const float dt = 0.005f;  // Sampling period: 5ms
const float K1 = 0.02f;   // Complementary filter coefficient

// X-axis complementary filter
float ComplementaryFilter::angleTempFilterX(float angle_m, float gyro_m)
{
    static float angle;
    angle = K1 * angle_m + (1 - K1) * (angle + gyro_m * dt);
    return angle;
}

// Y-axis complementary filter
float ComplementaryFilter::angleTempFilterY(float angle_m, float gyro_m)
{
    static float angle;
    angle = K1 * angle_m + (1 - K1) * (angle + gyro_m * dt);
    return angle;
}

// Kalman filter for X-axis
float KalmanFilter::Kalman_Filter_x(float Accel, float Gyro)
{
    static float angle_dot;
    static float angle;
    float Q_angle = 0.001; // Process noise covariance
    float Q_gyro = 0.003;  // Process noise covariance for gyro
    float R_angle = 0.5;   // Measurement noise covariance
    char C_0 = 1;
    static float Q_bias, Angle_err;
    static float PCt_0, PCt_1, E;
    static float K_0, K_1, t_0, t_1;
    static float Pdot[4] = {0, 0, 0, 0};
    static float PP[2][2] = { {1, 0}, {0, 1} };

    angle += (Gyro - Q_bias) * dt;  // Predict angle

    // Predict error covariance matrix derivative
    Pdot[0] = Q_angle - PP[0][1] - PP[1][0];
    Pdot[1] = -PP[1][1];
    Pdot[2] = -PP[1][1];
    Pdot[3] = Q_gyro;

    // Update error covariance matrix
    PP[0][0] += Pdot[0] * dt;
    PP[0][1] += Pdot[1] * dt;
    PP[1][0] += Pdot[2] * dt;
    PP[1][1] += Pdot[3] * dt;

    Angle_err = Accel - angle;  // Measurement residual

    PCt_0 = C_0 * PP[0][0];
    PCt_1 = C_0 * PP[1][0];
    E = R_angle + C_0 * PCt_0;

    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    t_0 = PCt_0;
    t_1 = C_0 * PP[0][1];

    // Update Kalman gain
    PP[0][0] -= K_0 * t_0;
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;

    // Update estimates
    angle += K_0 * Angle_err;
    Q_bias += K_1 * Angle_err;

    angle_dot = Gyro - Q_bias;  // Output: corrected angular velocity
    return angle;
}

// Kalman filter for Y-axis
float KalmanFilter::Kalman_Filter_y(float Accel, float Gyro)
{
    static float angle_dot;
    static float angle;
    float Q_angle = 0.001; // Process noise covariance
    float Q_gyro = 0.003;  // Process noise covariance for gyro
    float R_angle = 0.5;   // Measurement noise covariance
    char C_0 = 1;
    static float Q_bias, Angle_err;
    static float PCt_0, PCt_1, E;
    static float K_0, K_1, t_0, t_1;
    static float Pdot[4] = {0, 0, 0, 0};
    static float PP[2][2] = { {1, 0}, {0, 1} };

    angle += (Gyro - Q_bias) * dt;  // Predict angle

    // Predict error covariance matrix derivative
    Pdot[0] = Q_angle - PP[0][1] - PP[1][0];
    Pdot[1] = -PP[1][1];
    Pdot[2] = -PP[1][1];
    Pdot[3] = Q_gyro;

    // Update error covariance matrix
    PP[0][0] += Pdot[0] * dt;
    PP[0][1] += Pdot[1] * dt;
    PP[1][0] += Pdot[2] * dt;
    PP[1][1] += Pdot[3] * dt;

    Angle_err = Accel - angle;  // Measurement residual

    PCt_0 = C_0 * PP[0][0];
    PCt_1 = C_0 * PP[1][0];
    E = R_angle + C_0 * PCt_0;

    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    t_0 = PCt_0;
    t_1 = C_0 * PP[0][1];

    // Update Kalman gain
    PP[0][0] -= K_0 * t_0;
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;

    // Update estimates
    angle += K_0 * Angle_err;
    Q_bias += K_1 * Angle_err;

    angle_dot = Gyro - Q_bias;  // Output: corrected angular velocity
    return angle;
}

// Kalman filter angle calculation