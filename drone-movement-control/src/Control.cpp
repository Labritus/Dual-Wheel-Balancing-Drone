#include "MPU6050.hpp"

#include "Control.hpp"
#include "Motor.hpp"

#include "EXTI.hpp"
#include "Encoder.hpp"
#include "Ultrasonic.hpp"
#include "Filter.hpp"
#include "LED.hpp"
#include "ADC.hpp"
#include "Key.hpp"
#include "I2CSlave.hpp"

/**************************************************************************
 * Control function
 * 5ms external interrupt triggered by MPU6050's INT pin
 * Ensures real-time and synchronized data processing
 **************************************************************************/
int Control::exti15_10_IRQHandler(void)
{
    int Encoder_Left, Encoder_Right;                          // Left and right encoder data
    int Balance_Pwm, Velocity_Pwm, Turn_Pwm;                  // PWM variables for balance loop, speed loop, and turn loop
    static uint8_t Flag_Target;                               // Control function related variable, provides 10ms reference
    static int Voltage_Temp, Voltage_Count, Voltage_All;      // Variables for voltage measurement
    
    if(EXTI1::isPinInterrupt())
    {
        EXTI->PR = 1<<12;                                     // Clear interrupt flag
        Flag_Target = !Flag_Target;
        
        // Get attitude angle, every 5ms
        getAngle(Way_Angle);
        
        // Read encoder data
        Encoder_Left = -Encoder::read(3);                     // Read left encoder value
        Encoder_Right = -Encoder::read(4);                    // Read right encoder value
        
        // Calculate encoder velocity
        getVelocityFromEncoder(Encoder_Left, Encoder_Right);
        
        // Delay function
        if(delay_flag == 1)
        {
            if(++delay_50 == 10) delay_50 = 0, delay_flag = 0; // Provide accurate 50ms delay to external
        }
        
        if(Flag_Target == 1)                                     
        {
            // Measure battery voltage every 10ms
            Voltage_Temp = ADC::getBatteryVolt();           // Read battery voltage
            Voltage_Count++;                                   // Averaging counter
            Voltage_All += Voltage_Temp;                       // Accumulate
            if(Voltage_Count == 100)
            {
                Voltage = Voltage_All / 100;
                Voltage_All = 0;
                Voltage_Count = 0;                             // Calculate average
            }
            
            // Check I2C message - every 10ms
            if (I2CSlave::hasNewMessage()) {
                if (I2CSlave::isPersonDetected()) {
                    // Received PERSON_DETECTED message, switch to follow mode
                    Flag_follow = 1;
                    Flag_avoid = 0;
                }
                I2CSlave::clearMessageFlag();
            }
            
            return 0;
        }
        
        // Enter every 10ms to ensure MPU6050 data reading frequency
        
        // Read ultrasonic distance
        Ultrasonic::readDistance();
        
        // LED blinking
        if(Flag_avoid == 0 && Flag_follow == 0) LED1::flash(100);  // Normal mode, change LED state every 1s
        if(Flag_avoid == 1 || Flag_follow == 1) LED1::flash(0);    // Obstacle avoidance / Follow mode
        
        // Scan key state
        key();
        
        // Calculate three control loops
        Balance_Pwm = balance(Angle_Balance, Gyro_Balance);    // Balance PID control
        Velocity_Pwm = velocity(Encoder_Left, Encoder_Right);  // Speed loop PID control
        Turn_Pwm = turn(Gyro_Turn);                           // Turn PID control
        
        // Calculate final output PWM
        Motor_Left = Balance_Pwm + Velocity_Pwm + Turn_Pwm;    // Final PWM for left motor
        Motor_Right = Balance_Pwm + Velocity_Pwm - Turn_Pwm;   // Final PWM for right motor
        
        // PWM limiting
        Motor_Left = pwmLimit(Motor_Left, 6900, -6900);
        Motor_Right = pwmLimit(Motor_Right, 6900, -6900);
        
        // Car protection function
        if(pickUp(Acceleration_Z, Angle_Balance, Encoder_Left, Encoder_Right))  // Detect if the car is picked up
            Flag_Stop = 1;                                      // Stop motor if picked up
            
        if(putDown(Angle_Balance, Encoder_Left, Encoder_Right))   // Detect if the car is put down
            Flag_Stop = 0;                                      // Enable motor if put down
            
        // Switch function modes
        choose(Encoder_Left, Encoder_Right);
        
        // Check for abnormal conditions
        if(turnOff(Angle_Balance, Voltage) == 0)                   // If no abnormal condition
            setPwm(Motor_Left, Motor_Right);                       // Assign values to PWM registers
    }
    return 0;
}

/**************************************************************************
 * 直立PD控制
 * 输入参数：Angle:角度，Gyro:角速度
 * 返回值：平衡控制PWM
 **************************************************************************/
int Control::balance(float Angle, float Gyro)
{
    float Angle_bias, Gyro_bias;
    int balance;
    
    Angle_bias = Middle_angle - Angle;                       // 求出平衡的角度中值与机械相关
    Gyro_bias = 0 - Gyro;
    
    // 计算平衡控制的电机PWM  PD控制   kp是P系数，kd是D系数
    balance = -Balance_Kp/100 * Angle_bias - Gyro_bias * Balance_Kd/100;
    
    return balance;
}

/**************************************************************************
 * Upright PD control
 * Input parameters: Angle: angle, Gyro: angular velocity
 * Return value: balance control PWM
 **************************************************************************/
int Control::balance(float Angle, float Gyro)
{
    float Angle_bias, Gyro_bias;
    int balance;
    
    Angle_bias = Middle_angle - Angle;                       // Calculate balance angle bias, related to hardware
    Gyro_bias = 0 - Gyro;
    
    // Calculate motor PWM for balance control using PD control
    // kp is the proportional gain, kd is the derivative gain
    balance = -Balance_Kp/100 * Angle_bias - Gyro_bias * Balance_Kd/100;
    
    return balance;
}

/**************************************************************************
 * Speed control PWM
 * Input parameters: encoder_left: left wheel encoder count, encoder_right: right wheel encoder count
 * Return value: speed control PWM
 **************************************************************************/
int Control::velocity(int encoder_left, int encoder_right)
{
    static float velocity, Encoder_Least, Encoder_bias, Movement;
    static float Encoder_Integral, Target_Velocity;
    
    // Target speed setting
    if(Flag_follow == 1 || Flag_avoid == 1)
        Target_Velocity = 30;      // Follow/obstacle avoidance mode, automatically control speed
    else
        Target_Velocity = 50;
        
    // Remote forward/backward handling
    if(Flag_front == 1)
        Movement = Target_Velocity / Flag_velocity;  // Received forward command
    else if(Flag_back == 1)
        Movement = -Target_Velocity / Flag_velocity; // Received backward command
    else
        Movement = 0;
        
    // Follow/obstacle avoidance feature handling
    if(Flag_follow == 1 && (Distance > 200 && Distance < 500) && Flag_Left != 1 && Flag_Right != 1) // Follow
        Movement = Target_Velocity / Flag_velocity;
    if(Flag_follow == 1 && Distance < 200 && Flag_Left != 1 && Flag_Right != 1) 
        Movement = -Target_Velocity / Flag_velocity;
    if(Flag_avoid == 1 && Distance < 450 && Flag_Left != 1 && Flag_Right != 1)     // Ultrasonic obstacle avoidance
        Movement = -Target_Velocity / Flag_velocity;
        
    // Speed PI controller
    Encoder_Least = 0 - (encoder_left + encoder_right);                  // Get latest speed deviation
    Encoder_bias *= 0.86;                                                // First-order low-pass filter
    Encoder_bias += Encoder_Least * 0.14;                                // First-order low-pass filter
    
    Encoder_Integral += Encoder_bias;                                    // Integrate to get displacement
    Encoder_Integral = Encoder_Integral + Movement;                      // Receive remote data, control forward/backward
    
    // Integral limit
    if(Encoder_Integral > 10000)
        Encoder_Integral = 10000;
    if(Encoder_Integral < -10000)
        Encoder_Integral = -10000;
        
    // Speed control
    velocity = -Encoder_bias * Velocity_Kp / 100 - Encoder_Integral * Velocity_Ki / 100;
    
    // Clear integral after motor shut down
    if(turnOff(Angle_Balance, Voltage) == 1 || Flag_Stop == 1)
        Encoder_Integral = 0;
        
    return velocity;
}

/**************************************************************************
 * Turning control
 * Input parameter: Z-axis gyroscope
 * Return value: turning control PWM
 **************************************************************************/
int Control::turn(float gyro)
{
    static float Turn_Target, turn, Turn_Amplitude = 54;
    float Kp = Turn_Kp, Kd;
    
    // Remote left/right turning handling
    if(Flag_Left == 1)
        Turn_Target = -Turn_Amplitude / Flag_velocity;
    else if(Flag_Right == 1)
        Turn_Target = Turn_Amplitude / Flag_velocity;
    else
        Turn_Target = 0;
        
    // Cancel rolling forward/backward when turning
    if(Flag_front == 1 || Flag_back == 1)
        Kd = Turn_Kd;
    else
        Kd = 0;
        
    // Turning PD controller
    turn = Turn_Target * Kp / 100 + gyro * Kd / 100;     // PD control using Z-axis gyroscope
    
    return turn;
}

/**************************************************************************
 * Assign values to PWM registers
 * Input parameters: left PWM, right PWM
 **************************************************************************/
void Control::setPwm(int motor_left, int motor_right)
{
    // Left motor direction control
    if(motor_left > 0)
    {
        Motor::leftForward();  // Forward
    }
    else
    {
        Motor::leftBackward(); // Backward
    }
    
    Motor::setLeftPwm(myAbs(motor_left));
    
    // Right motor direction control
    if(motor_right > 0)
    {
        Motor::rightForward(); // Forward
    }
    else
    {
        Motor::rightBackward(); // Backward
    }
    
    Motor::setRightPwm(myAbs(motor_right));
}

/**************************************************************************
 * Limit PWM range
 * Input parameters: IN: input value, max: upper limit, min: lower limit
 * Return value: clamped value
 **************************************************************************/
int Control::pwmLimit(int IN, int max, int min)
{
    int OUT = IN;
    if(OUT > max) OUT = max;
    if(OUT < min) OUT = min;
    return OUT;
}

/**************************************************************************
 * Key press changes the running state of the car
 **************************************************************************/
void Control::key(void)
{
    uint8_t tmp, tmp2;
    
    // Single click to start/stop the car
    tmp = Key::clickNDouble(50);
    if(tmp == 1)
    {
        Flag_Stop = !Flag_Stop;
    }
    
    // Long press to switch display mode
    tmp2 = Key::longPress();
    if(tmp2 == 1)
    {
        Flag_Show = !Flag_Show;
    }
}

/**************************************************************************
 * Emergency motor shutdown
 * Input parameters: angle: tilt angle of car, voltage: battery voltage
 * Return value: 1 = error, 0 = normal
 **************************************************************************/
uint8_t Control::turnOff(float angle, int voltage)
{
    uint8_t temp;
    
    // If tilt angle exceeds ±40 degrees or voltage is below 10V, shut down motor
    if(angle < -40 || angle > 40 || Flag_Stop == 1 || voltage < 1000)
    {
        temp = 1;
        Motor::stop(); // Stop motor
    }
    else
    {
        temp = 0;
    }
    
    return temp;
}

/**************************************************************************
 * Get angle
 * Input parameter: way: algorithm used to get angle (1: DMP, 2: Kalman, 3: Complementary filter)
 **************************************************************************/
void Control::getAngle(uint8_t way)
{
    float gyro_x, gyro_y, accel_x, accel_y, accel_z;
    float Accel_Y, Accel_Z, Accel_X, Accel_Angle_x, Accel_Angle_y, Gyro_X, Gyro_Z, Gyro_Y;
    
    Temperature = MPU6050::getTemperature(); // Read internal temperature sensor of MPU6050 in Celsius
    
    if(way == 1) // DMP is read in data acquisition interrupt
    {
        MPU6050::getDMPData();       // Read accelerometer, gyroscope, angle
        Angle_Balance = Pitch;     // Update balance tilt angle
        Gyro_Balance = gyro[0];    // Update balance angular velocity
        Gyro_Turn = gyro[2];       // Update turn angular velocity
        Acceleration_Z = accel[2]; // Update Z-axis acceleration
    }
    else
    {
        // Read raw data from MPU6050
        Gyro_X = (IOI2C::readByteFromRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_XOUT_H) << 8) + IOI2C::readByteFromRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_XOUT_L);
        Gyro_Y = (IOI2C::readByteFromRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_YOUT_H) << 8) + IOI2C::readByteFromRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_YOUT_L);
        Gyro_Z = (IOI2C::readByteFromRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_ZOUT_H) << 8) + IOI2C::readByteFromRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_ZOUT_L);
        Accel_X = (IOI2C::readByteFromRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H) << 8) + IOI2C::readByteFromRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_L);
        Accel_Y = (IOI2C::readByteFromRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_YOUT_H) << 8) + IOI2C::readByteFromRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_YOUT_L);
        Accel_Z = (IOI2C::readByteFromRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_ZOUT_H) << 8) + IOI2C::readByteFromRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_ZOUT_L);
        
        // Data type conversion
        if(Gyro_X > 32768) Gyro_X -= 65536;
        if(Gyro_Y > 32768) Gyro_Y -= 65536;
        if(Gyro_Z > 32768) Gyro_Z -= 65536;
        if(Accel_X > 32768) Accel_X -= 65536;
        if(Accel_Y > 32768) Accel_Y -= 65536;
        if(Accel_Z > 32768) Accel_Z -= 65536;
        
        Gyro_Balance = -Gyro_X;                           // Update balance angular velocity
        Accel_Angle_x = KalmanFilter::angleCalculate(Accel_Y, Accel_Z); // Calculate tilt angle, in degrees
        Accel_Angle_y = KalmanFilter::angleCalculate(Accel_X, Accel_Z); // Calculate tilt angle, in degrees
        
        // Convert raw data to standard units
        accel_x = Accel_X / 16384; // Accelerometer is ±2g, raw data is 16-bit, divide by 16384 to get acceleration in g
        accel_y = Accel_Y / 16384;
        accel_z = Accel_Z / 16384;
        
        gyro_x = Gyro_X / 16.4;    // Gyroscope is ±2000°/s, raw data maps ±32768 to ±2000°/s
        gyro_y = Gyro_Y / 16.4;
        
        // Select algorithm to get angle
        if(Way_Angle == 2)
        {
            Pitch = -KalmanFilter::Kalman_Filter_x(Accel_Angle_x, gyro_x); // Kalman filter, in degrees
            Roll = -KalmanFilter::Kalman_Filter_y(Accel_Angle_y, gyro_y);
        }
        else if(Way_Angle == 3)
        {
            Pitch = -ComplementaryFilter::angleTempFilterX(Accel_Angle_x, gyro_x); // Complementary filter
            Roll = -ComplementaryFilter::angleTempFilterY(Accel_Angle_y, gyro_y);
        }
        
        Angle_Balance = Pitch;      // Update balance tilt angle
        Gyro_Turn = Gyro_Z;         // Update turn angular velocity
        Acceleration_Z = Accel_Z;   // Update Z-axis acceleration
    }
}

/**************************************************************************
 * Absolute value function
 * Input parameter: a: number to convert
 * Return value: unsigned integer
 **************************************************************************/
int Control::myAbs(int a)
{
    int temp;
    if(a < 0) temp = -a;
    else temp = a;
    return temp;
}

/**************************************************************************
 * Detect if the car is picked up
 * Input parameters: Acceleration: Z-axis acceleration, Angle: balance angle, encoder_left/right: encoder counts
 * Return value: 1 = picked up, 0 = not picked up
 **************************************************************************/
int Control::pickUp(float Acceleration, float Angle, int encoder_left, int encoder_right)
{
    static uint16_t flag, count0, count1, count2;
    
    if(flag == 0) // Step 1
    {
        if(myAbs(encoder_left) + myAbs(encoder_right) < 30) // Condition 1: Car is nearly stationary
            count0++;
        else
            count0 = 0;
        
        if(count0 > 10)
            flag = 1, count0 = 0;
    }
    
    if(flag == 1) // Step 2
    {
        if(++count1 > 200) count1 = 0, flag = 0;             // Timeout after 2000ms
        if(Acceleration > 26000 && (Angle > (-20 + Middle_angle)) && (Angle < (20 + Middle_angle))) // Condition 2: Car is picked up near 0 degree
            flag = 2;
    }
    
    if(flag == 2) // Step 3
    {
        if(++count2 > 100) count2 = 0, flag = 0;           // Timeout after 1000ms
        if(myAbs(encoder_left + encoder_right) > 70)       // Condition 3: Wheels rotate slightly after being picked up
        {
            flag = 0;
            return 1; // Car has been picked up
        }
    }
    
    return 0;
}

/**************************************************************************
 * Detect if the car is placed down
 * Input parameters: balance angle, left/right encoder counts
 * Return value: 1 = placed down, 0 = not placed down
 **************************************************************************/
int Control::putDown(float Angle, int encoder_left, int encoder_right)
{
    static uint16_t flag, count;
    
    if(Flag_Stop == 0) // Not stopped
        return 0;
        
    if(flag == 0)
    {
        if(Angle > (-10 + Middle_angle) && Angle < (10 + Middle_angle) && encoder_left == 0 && encoder_right == 0) // Condition 1: Placed near 0 degree
            flag = 1;
    }
    
    if(flag == 1)
    {
        if(++count > 50) // Timeout after 500ms
        {
            count = 0;
            flag = 0;
        }
        
        if(encoder_left > 3 && encoder_right > 3 && encoder_left < 40 && encoder_right < 40) // Condition 2: Wheels rotate slightly on the ground
        {
            flag = 0;
            return 1; // Car has been placed down
        }
    }
    
    return 0;
}

/**************************************************************************
 * Convert encoder data to speed (mm/s)
 **************************************************************************/
void Control::getVelocityFromEncoder(int encoder_left, int encoder_right)
{
    float Velocity_Left;
    float Velocity_Right;
    float Rotation_Speed_L, Rotation_Speed_R;
    
    // Calculate RPM: encoder count (5ms each time) * sampling frequency / multiplier / gear ratio / encoder precision
    Rotation_Speed_L = encoder_left * Control_Frequency / EncoderMultiples / Reduction_Ratio / Encoder_precision;
    Velocity_Left = Rotation_Speed_L * PI * Diameter_67; // Left wheel speed = RPM * circumference
    
    Rotation_Speed_R = encoder_right * Control_Frequency / EncoderMultiples / Reduction_Ratio / Encoder_precision;
    Velocity_Right = Rotation_Speed_R * PI * Diameter_67; // Right wheel speed = RPM * circumference
}

/**************************************************************************
 * Select car operation mode
 * Input parameters: encoder_left: left encoder, encoder_right: right encoder
 **************************************************************************/
void Control::choose(int encoder_left, int encoder_right)
{
    static int count;
    
    if(Flag_Stop == 0)
        count = 0;
        
    if((Flag_Stop == 1) && (encoder_left < 10)) // Temporarily stopped, right wheel sliding
    {
        count += myAbs(encoder_right);
        
        if(count > 6 && count < 180) // Normal mode
        {
            Flag_follow = 0;
            Flag_avoid = 0;
        }
        
        if(count > 180 && count < 360) // Obstacle avoidance mode
        {
            Flag_avoid = 1;
            Flag_follow = 0;
        }
        
        if(count > 360 && count < 540) // Follow mode
        {
            Flag_avoid = 0;
            Flag_follow = 1;
        }
        
        if(count > 540)
            count = 0;
    }
}
