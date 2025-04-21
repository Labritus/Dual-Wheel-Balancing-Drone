#include "Show.hpp"
#include "OLED.hpp"
#include "DataScope_DP.hpp"
#include "USART3.hpp"
#include "USART.hpp"
#include <stdio.h>
#include "Show.hpp"
#include <string.h>

bool Show::waveformEnable0d = false;

// Left and right wheel speed
float Velocity_Left, Velocity_Right;

/**
 * Display data on OLED
 */
void Show::oledShow()
{
    // First row: car mode
    if(Way_Angle == 1)        OLED::showString(0, 0, "DMP");
    else if(Way_Angle == 2)   OLED::showString(0, 0, "Kalman");
    else if(Way_Angle == 3)   OLED::showString(0, 0, "C F");
    
    if(Flag_follow == 1)      OLED::showString(70, 0, "Follow");
    else if(Flag_avoid == 1)  OLED::showString(70, 0, "Avoid ");
    else                      OLED::showString(70, 0, "Normal");
    
    // Second row: angle
    OLED::showString(0, 10, "Angle");
    if(Angle_Balance < 0)     OLED::showString(48, 10, "-");
    if(Angle_Balance >= 0)    OLED::showString(48, 10, "+");
    OLED::showNumber(56, 10, myabs((int)Angle_Balance), 3, 12);
    
    // Third row: angular velocity and distance
    OLED::showString(0, 20, "Gyrox");
    if(Gyro_Balance < 0)      OLED::showString(42, 20, "-");
    if(Gyro_Balance >= 0)     OLED::showString(42, 20, "+");
    OLED::showNumber(50, 20, myabs((int)Gyro_Balance), 4, 12);
    
    OLED::showNumber(82, 20, (uint16_t)Distance, 5, 12);
    OLED::showString(114, 20, "mm");
    
    // Fourth row: left motor PWM and speed
    OLED::showString(0, 30, "L");
    if(Motor_Left < 0) {
        OLED::showString(16, 30, "-");
        OLED::showNumber(26, 30, myabs((int)Motor_Left), 4, 12);
    }
    if(Motor_Left >= 0) {
        OLED::showString(16, 30, "+");
        OLED::showNumber(26, 30, myabs((int)Motor_Left), 4, 12);
    }
    
    if(Velocity_Left < 0)     OLED::showString(60, 30, "-");
    if(Velocity_Left >= 0)    OLED::showString(60, 30, "+");
    OLED::showNumber(68, 30, myabs((int)Velocity_Left), 4, 12);
    OLED::showString(96, 30, "mm/s");
    
    // Fifth row: right motor PWM and speed
    OLED::showString(0, 40, "R");
    if(Motor_Right < 0) {
        OLED::showString(16, 40, "-");
        OLED::showNumber(26, 40, myabs((int)Motor_Right), 4, 12);
    }
    if(Motor_Right >= 0) {
        OLED::showString(16, 40, "+");
        OLED::showNumber(26, 40, myabs((int)Motor_Right), 4, 12);
    }
    
    if(Velocity_Right < 0)    OLED::showString(60, 40, "-");
    if(Velocity_Right >= 0)   OLED::showString(60, 40, "+");
    OLED::showNumber(68, 40, myabs((int)Velocity_Right), 4, 12);
    OLED::showString(96, 40, "mm/s");
    
    // Sixth row: voltage and switch status
    OLED::showString(0, 50, "V");
    OLED::showString(30, 50, ".");
    OLED::showString(64, 50, "V");
    OLED::showNumber(19, 50, Voltage/100, 2, 12);
    OLED::showNumber(42, 50, Voltage/10%10, 1, 12);
    OLED::showNumber(50, 50, Voltage%10, 1, 12);
    
    if(Flag_Stop)     OLED::showString(95, 50, "OFF");
    if(!Flag_Stop)    OLED::showString(95, 50, "ON ");
    
    // Refresh screen
    OLED::refresh();
}

/**
 * Display data on mobile APP
 */
void Show::appShow()
{
    static uint8_t flag;
    int Encoder_Left_Show, Encoder_Right_Show, Voltage_Show;
    
    // Process voltage data
    Voltage_Show = (Voltage - 1000) * 2 / 3;
    if(Voltage_Show < 0) Voltage_Show = 0;
    if(Voltage_Show > 100) Voltage_Show = 100;
    
    // Process speed data
    Encoder_Right_Show = Velocity_Right * 1.1;
    if(Encoder_Right_Show < 0) Encoder_Right_Show = -Encoder_Right_Show;
    
    Encoder_Left_Show = Velocity_Left * 1.1;
    if(Encoder_Left_Show < 0) Encoder_Left_Show = -Encoder_Left_Show;
    
    flag = !flag;
}

/**
 * DataScope waveform display on upper computer
 */
void Show::dataScope()
{
    uint8_t i;
    float Vol;
    unsigned char Send_Count;
    
    Vol = (float)Voltage / 100;
    
    // Set channel data
    DataScope_DP::setChannelData(Angle_Balance, 1);   // Show angle (degrees)
    DataScope_DP::setChannelData(Distance/10, 2);     // Show ultrasonic distance (cm)
    DataScope_DP::setChannelData(Vol, 3);             // Show battery voltage (V)
    
    Send_Count = DataScope_DP::generateDataFrame(3);
    
    // Send data
    for(i = 0; i < Send_Count; i++)
    {
        while((USART1->SR & 0X40) == 0);
        USART1->DR = DataScope_DP::getOutputBuffer()[i];
    }
}

/**
 * Return absolute value
 */
int Show::myabs(int a)
{
    if(a < 0) return -a;
    else return a;
}
