#include "KalmanFilter.hpp"
#include <math.h>

// Calculate angle from accelerometer
float KalmanFilter::angleCalculate(float accel_x, float accel_z)
{
    return atan2(-accel_x, accel_z) * 180.0f / PI;
}

// Apply Kalman filter to compute X-axis angle
float KalmanFilter::angleXKalmanFilter(float accel_y, float accel_z, float gyro_x)
{
    return KF_X(accel_y, accel_z, gyro_x);
}

// Apply Kalman filter to compute Y-axis angle
float KalmanFilter::angleYKalmanFilter(float accel_x, float accel_z, float gyro_y)
{
    return KF_Y(accel_x, accel_z, gyro_y);
}

// Apply Kalman filter to compute angle (default Y-axis)
float KalmanFilter::angleKalmanFilter(float angle, float gyro)
{
    // Simplified Kalman filter interface, internally uses Y-axis filter
    // Here we assume `angle` is pre-computed and convert it into accelerometer format
    float accel_x = -sin(angle * PI / 180.0f);
    float accel_z = cos(angle * PI / 180.0f);
    return angleYKalmanFilter(accel_x, accel_z, gyro);
}

// Matrix multiplication - internal helper function
void KalmanFilter::matrixMultiply(int A_row, int A_col, int B_row, int B_col, 
                                  float A[][2], float B[][2], float C[][2])
{
    // This function is not used directly, kept for interface compatibility
    // More complex multiplication is done in the C implementation
}

extern "C" {
    // Kalman filter for X-axis
    float KF_X(float acce_Y, float acce_Z, float gyro_X)
    {
        static float x_hat[2][1] = {0};       // State estimate
        static float x_hat_minus[2][1] = {0}; // Prior estimate
        static float p_hat[2][2] = {{1, 0}, {0, 1}}; // Covariance estimate
        static float p_hat_minus[2][2] = {0};        // Prior covariance
        static float K[2][1] = {0};           // Kalman gain
        const float Ts = 0.005;               // Sample period (5ms)
        const float I[2][2] = {{1, 0}, {0, 1}};
        float u[1][1] = {{gyro_X}};
        float A[2][2] = {{1, -Ts}, {0, 1}};    // State transition matrix
        float B[2][1] = {{Ts}, {0}};           // Input matrix
        float C[1][2] = {{1, 0}};              // Output matrix
        float Q[2][2] = {{1e-10, 0}, {0, 1e-10}}; // Process noise
        float R[1][1] = {{1e-4}};              // Measurement noise
        float A_T[2][2] = {{1, 0}, {-Ts, 1}};  // Transpose of A
        float C_T[2][1] = {{1}, {0}};          // Transpose of C

        // Temp buffers for intermediate results
        float temp_1[2][1] = {0};
        float temp_2[2][1] = {0};
        float temp_3[2][2] = {0};
        float temp_4[2][2] = {0};
        float temp_5[1][2] = {0};
        float temp_6[1][1] = {0};

        float y = atan2(-acce_Y, acce_Z); // Compute angle from acceleration

        // --- Prediction Step ---
        mul(2, 2, 2, 1, (float*)A, (float*)x_hat, (float*)temp_1);
        mul(2, 1, 1, 1, (float*)B, (float*)u, (float*)temp_2);
        x_hat_minus[0][0] = temp_1[0][0] + temp_2[0][0];
        x_hat_minus[1][0] = temp_1[1][0] + temp_2[1][0];

        mul(2, 2, 2, 2, (float*)A, (float*)p_hat, (float*)temp_3);
        mul(2, 2, 2, 2, (float*)temp_3, (float*)A_T, (float*)temp_4);
        p_hat_minus[0][0] = temp_4[0][0] + Q[0][0];
        p_hat_minus[0][1] = temp_4[0][1] + Q[0][1];
        p_hat_minus[1][0] = temp_4[1][0] + Q[1][0];
        p_hat_minus[1][1] = temp_4[1][1] + Q[1][1];

        // --- Correction Step ---
        mul(1, 2, 2, 2, (float*)C, (float*)p_hat_minus, (float*)temp_5);
        mul(1, 2, 2, 1, (float*)temp_5, (float*)C_T, (float*)temp_6);
        temp_6[0][0] = 1.0f / (temp_6[0][0] + R[0][0]);
        mul(2, 2, 2, 1, (float*)p_hat_minus, (float*)C_T, (float*)temp_1);
        mul(2, 1, 1, 1, (float*)temp_1, (float*)temp_6, (float*)K);

        mul(1, 2, 2, 1, (float*)C, (float*)x_hat_minus, (float*)temp_6);
        temp_6[0][0] = y - temp_6[0][0];
        mul(2, 1, 1, 1, (float*)K, (float*)temp_6, (float*)temp_1);
        x_hat[0][0] = x_hat_minus[0][0] + temp_1[0][0];
        x_hat[1][0] = x_hat_minus[1][0] + temp_1[1][0];

        mul(2, 1, 1, 2, (float*)K, (float*)C, (float*)temp_3);
        temp_3[0][0] = I[0][0] - temp_3[0][0];
        temp_3[0][1] = I[0][1] - temp_3[0][1];
        temp_3[1][0] = I[1][0] - temp_3[1][0];
        temp_3[1][1] = I[1][1] - temp_3[1][1];
        mul(2, 2, 2, 2, (float*)temp_3, (float*)p_hat_minus, (float*)p_hat);

        return x_hat[0][0];
    }

    // Kalman filter for Y-axis
    float KF_Y(float acce_X, float acce_Z, float gyro_Y)
    {
        // Identical to KF_X but using gyro_Y and acce_X/Z
        // (Only comments have been removed for brevity)
        static float x_hat[2][1] = {0};
        static float x_hat_minus[2][1] = {0};
        static float p_hat[2][2] = {{1, 0}, {0, 1}};
        static float p_hat_minus[2][2] = {0};
        static float K[2][1] = {0};
        const float Ts = 0.005;
        const float I[2][2] = {{1, 0}, {0, 1}};
        float u[1][1] = {{gyro_Y}};
        float A[2][2] = {{1, -Ts}, {0, 1}};
        float B[2][1] = {{Ts}, {0}};
        float C[1][2] = {{1, 0}};
        float Q[2][2] = {{1e-10, 0}, {0, 1e-10}};
        float R[1][1] = {{1e-4}};
        float A_T[2][2] = {{1, 0}, {-Ts, 1}};
        float C_T[2][1] = {{1}, {0}};
        float temp_1[2][1] = {0};
        float temp_2[2][1] = {0};
        float temp_3[2][2] = {0};
        float temp_4[2][2] = {0};
        float temp_5[1][2] = {0};
        float temp_6[1][1] = {0};
        float y = atan2(-acce_X, acce_Z);

        mul(2, 2, 2, 1, (float*)A, (float*)x_hat, (float*)temp_1);
        mul(2, 1, 1, 1, (float*)B, (float*)u, (float*)temp_2);
        x_hat_minus[0][0] = temp_1[0][0] + temp_2[0][0];
        x_hat_minus[1][0] = temp_1[1][0] + temp_2[1][0];

        mul(2, 2, 2, 2, (float*)A, (float*)p_hat, (float*)temp_3);
        mul(2, 2, 2, 2, (float*)temp_3, (float*)A_T, (float*)temp_4);
        p_hat_minus[0][0] = temp_4[0][0] + Q[0][0];
        p_hat_minus[0][1] = temp_4[0][1] + Q[0][1];
        p_hat_minus[1][0] = temp_4[1][0] + Q[1][0];
        p_hat_minus[1][1] = temp_4[1][1] + Q[1][1];

        mul(1, 2, 2, 2, (float*)C, (float*)p_hat_minus, (float*)temp_5);
        mul(1, 2, 2, 1, (float*)temp_5, (float*)C_T, (float*)temp_6);
        temp_6[0][0] = 1.0f / (temp_6[0][0] + R[0][0]);
        mul(2, 2, 2, 1, (float*)p_hat_minus, (float*)C_T, (float*)temp_1);
        mul(2, 1, 1, 1, (float*)temp_1, (float*)temp_6, (float*)K);

        mul(1, 2, 2, 1, (float*)C, (float*)x_hat_minus, (float*)temp_6);
        temp_6[0][0] = y - temp_6[0][0];
        mul(2, 1, 1, 1, (float*)K, (float*)temp_6, (float*)temp_1);
        x_hat[0][0] = x_hat_minus[0][0] + temp_1[0][0];
        x_hat[1][0] = x_hat_minus[1][0] + temp_1[1][0];

        mul(2, 1, 1, 2, (float*)K, (float*)C, (float*)temp_3);
        temp_3[0][0] = I[0][0] - temp_3[0][0];
        temp_3[0][1] = I[0][1] - temp_3[0][1];
        temp_3[1][0] = I[1][0] - temp_3[1][0];
        temp_3[1][1] = I[1][1] - temp_3[1][1];
        mul(2, 2, 2, 2, (float*)temp_3, (float*)p_hat_minus, (float*)p_hat);

        return x_hat[0][0];
    }

    // Matrix multiplication (generic implementation for Kalman filter)
    void mul(int A_row, int A_col, int B_row, int B_col, 
             float *A, float *B, float *C)
    {
        if (A_col != B_row) {
            // Error: matrix dimensions do not match
            return;
        }
        
        for (int i = 0; i < A_row; i++) {
            for (int j = 0; j < B_col; j++) {
                float sum = 0.0f;
                for (int k = 0; k < A_col; k++) {
                    // Calculate matrix element positions
                    float a_val = A[i * A_col + k];
                    float b_val = B[k * B_col + j];
                    sum += a_val * b_val;
                }
                C[i * B_col + j] = sum;
            }
        }
    }
}
