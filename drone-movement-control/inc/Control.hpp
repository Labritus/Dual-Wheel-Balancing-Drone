#ifndef __CONTROL_HPP
#define __CONTROL_HPP

#include "System.hpp"
#include "CallbackInterface.hpp"
#include "../src/control_classes/PIDController.hpp"

#define PI 3.14159265              // PI constant
#define Control_Frequency  200.0   // Control frequency / sampling frequency
#define Diameter_67  67.0          // Wheel diameter: 67 mm
#define EncoderMultiples   4.0     // Encoder multiplier
#define Encoder_precision  13.0    // Encoder resolution: 13 lines
#define Reduction_Ratio  30.0      // Gear reduction ratio: 30
#define Perimeter  210.4867        // Wheel circumference in mm

#define MIDDLE_ANGLE_DEFAULT 0
#define DIFFERENCE 100

class Control : public SensorCallback {
private:
    // Control flags
    static bool Flag_follow;
    static bool Flag_avoid;
    static bool Flag_Stop;
    static bool Flag_Show;
    static bool Flag_front;
    static bool Flag_back;
    static bool Flag_Left;
    static bool Flag_Right;
    static uint8_t Flag_velocity;
    static int Voltage;
    static float Distance;
    static float Middle_angle;
    static float Angle_Balance;
    static float Gyro_Balance;
    static float Gyro_Turn;
    static int Acceleration_Z;
    static uint8_t Way_Angle;
    static int Balance_Kp;
    static int Balance_Kd;
    static int Velocity_Kp;
    static int Velocity_Ki;
    static int Turn_Kp;
    static int Turn_Kd;
    static int Motor_Left;
    static int Motor_Right;
    static int delay_50;
    static uint8_t delay_flag;
    static int Temperature;
    
    // Static PID controllers
    static PIDController balancePID_;
    static PIDController velocityPID_;
    static PIDController turnPID_;

public:
    // Getter and setter methods for control flags
    static bool getFlagFollow() { return Flag_follow; }
    static void setFlagFollow(bool value) { Flag_follow = value; }
    static bool getFlagAvoid() { return Flag_avoid; }
    static void setFlagAvoid(bool value) { Flag_avoid = value; }
    static bool getFlagStop() { return Flag_Stop; }
    static void setFlagStop(bool value) { Flag_Stop = value; }
    static bool getFlagShow() { return Flag_Show; }
    static void setFlagShow(bool value) { Flag_Show = value; }
    static bool getFlagFront() { return Flag_front; }
    static void setFlagFront(bool value) { Flag_front = value; }
    static bool getFlagBack() { return Flag_back; }
    static void setFlagBack(bool value) { Flag_back = value; }
    static bool getFlagLeft() { return Flag_Left; }
    static void setFlagLeft(bool value) { Flag_Left = value; }
    static bool getFlagRight() { return Flag_Right; }
    static void setFlagRight(bool value) { Flag_Right = value; }
    static uint8_t getFlagVelocity() { return Flag_velocity; }
    static void setFlagVelocity(uint8_t value) { Flag_velocity = value; }
    static int getVoltage() { return Voltage; }
    static void setVoltage(int value) { Voltage = value; }
    static float getDistance() { return Distance; }
    static void setDistance(float value) { Distance = value; }
    static float getMiddleAngle() { return Middle_angle; }
    static void setMiddleAngle(float value) { Middle_angle = value; }
    static float getAngleBalance() { return Angle_Balance; }
    static void setAngleBalance(float value) { Angle_Balance = value; }
    static float getGyroBalance() { return Gyro_Balance; }
    static void setGyroBalance(float value) { Gyro_Balance = value; }
    static float getGyroTurn() { return Gyro_Turn; }
    static void setGyroTurn(float value) { Gyro_Turn = value; }
    static int getAccelerationZ() { return Acceleration_Z; }
    static void setAccelerationZ(int value) { Acceleration_Z = value; }
    static uint8_t getWayAngle() { return Way_Angle; }
    static void setWayAngle(uint8_t value) { Way_Angle = value; }
    static int getBalanceKp() { return Balance_Kp; }
    static void setBalanceKp(int value) { Balance_Kp = value; }
    static int getBalanceKd() { return Balance_Kd; }
    static void setBalanceKd(int value) { Balance_Kd = value; }
    static int getVelocityKp() { return Velocity_Kp; }
    static void setVelocityKp(int value) { Velocity_Kp = value; }
    static int getVelocityKi() { return Velocity_Ki; }
    static void setVelocityKi(int value) { Velocity_Ki = value; }
    static int getTurnKp() { return Turn_Kp; }
    static void setTurnKp(int value) { Turn_Kp = value; }
    static int getTurnKd() { return Turn_Kd; }
    static void setTurnKd(int value) { Turn_Kd = value; }
    static int getMotorLeft() { return Motor_Left; }
    static void setMotorLeft(int value) { Motor_Left = value; }
    static int getMotorRight() { return Motor_Right; }
    static void setMotorRight(int value) { Motor_Right = value; }
    static int getDelay50() { return delay_50; }
    static void setDelay50(int value) { delay_50 = value; }
    static uint8_t getDelayFlag() { return delay_flag; }
    static void setDelayFlag(uint8_t value) { delay_flag = value; }

    // Sensor callback implementation
    void onSensorDataUpdated(const SensorData& data) override;

    // Control loop processor (adapted from STM32 interrupt handler)
    static int processControlLoop(void);
    
    // Balance control
    static int balance(float angle, float gyro);
    
    // Velocity control
    static int velocity(int encoder_left, int encoder_right);
    
    // Turn control
    static int turn(float gyro);
    
    // Set PWM for motors
    static void setPwm(int motor_left, int motor_right);
    
    // Key press handling
    static void key(void);
    
    // Limit PWM output
    static int pwmLimit(int input, int max, int min);
    
    // Turn off motors based on conditions
    static uint8_t turnOff(float angle, int voltage);
    
    // Get angle from sensor
    static void getAngle(uint8_t way);
    
    // Absolute value
    static int myAbs(int a);
    
    // Pick-up detection
    static int pickUp(float acceleration, float angle, int encoder_left, int encoder_right);
    
    // Put-down detection
    static int putDown(float angle, int encoder_left, int encoder_right);
    
    // Compute speed from encoder values
    static void getVelocityFromEncoder(int encoder_left, int encoder_right);
    
    // Mode selection logic
    static void choose(int encoder_left, int encoder_right);
};

// Raspberry Pi compatible control loop - no external interrupt handler needed

#endif // __CONTROL_HPP