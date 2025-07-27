#include "Show.hpp"
#include "OLED.hpp"
#include "DataScope_DP.hpp"
#include "USART3.hpp"
#include "USART.hpp"
#include "Control.hpp"
#include <stdio.h>
#include <string.h>

// Remove invalid static member - not declared in header
// bool Show::waveformEnable0d = false;

// Left and right wheel speed
float Velocity_Left, Velocity_Right;

/**
 * Display data on OLED
 */
void Show::oledShow()
{
    // First row: car mode
    if(Control::getWayAngle() == 1)        OLED::showString(0, 0, (const uint8_t*)"DMP");
    else if(Control::getWayAngle() == 2)   OLED::showString(0, 0, (const uint8_t*)"Kalman");
    else if(Control::getWayAngle() == 3)   OLED::showString(0, 0, (const uint8_t*)"C F");
    
    if(Control::getFlagFollow() == 1)      OLED::showString(70, 0, (const uint8_t*)"Follow");
    else if(Control::getFlagAvoid() == 1)  OLED::showString(70, 0, (const uint8_t*)"Avoid ");
    else                      OLED::showString(70, 0, (const uint8_t*)"Normal");
    
    // Second row: angle
    OLED::showString(0, 10, (const uint8_t*)"Angle");
    if(Control::getAngleBalance() < 0)     OLED::showString(48, 10, (const uint8_t*)"-");
    if(Control::getAngleBalance() >= 0)    OLED::showString(48, 10, (const uint8_t*)"+");
    OLED::showNumber(56, 10, myabs((int)Control::getAngleBalance()), 3, 12);
    
    // Third row: angular velocity and distance
    OLED::showString(0, 20, (const uint8_t*)"Gyrox");
    if(Control::getGyroBalance() < 0)      OLED::showString(42, 20, (const uint8_t*)"-");
    if(Control::getGyroBalance() >= 0)     OLED::showString(42, 20, (const uint8_t*)"+");
    OLED::showNumber(50, 20, myabs((int)Control::getGyroBalance()), 4, 12);
    
    OLED::showNumber(82, 20, (uint16_t)Control::getDistance(), 5, 12);
    OLED::showString(114, 20, (const uint8_t*)"mm");
    
    // Fourth row: left motor PWM and speed
    OLED::showString(0, 30, (const uint8_t*)"L");
    if(Control::getMotorLeft() < 0) {
        OLED::showString(16, 30, (const uint8_t*)"-");
        OLED::showNumber(26, 30, myabs((int)Control::getMotorLeft()), 4, 12);
    }
    if(Control::getMotorLeft() >= 0) {
        OLED::showString(16, 30, (const uint8_t*)"+");
        OLED::showNumber(26, 30, myabs((int)Control::getMotorLeft()), 4, 12);
    }
    
    if(Velocity_Left < 0)     OLED::showString(60, 30, (const uint8_t*)"-");
    if(Velocity_Left >= 0)    OLED::showString(60, 30, (const uint8_t*)"+");
    OLED::showNumber(68, 30, myabs((int)Velocity_Left), 4, 12);
    OLED::showString(96, 30, (const uint8_t*)"mm/s");
    
    // Fifth row: right motor PWM and speed
    OLED::showString(0, 40, (const uint8_t*)"R");
    if(Control::getMotorRight() < 0) {
        OLED::showString(16, 40, (const uint8_t*)"-");
        OLED::showNumber(26, 40, myabs((int)Control::getMotorRight()), 4, 12);
    }
    if(Control::getMotorRight() >= 0) {
        OLED::showString(16, 40, (const uint8_t*)"+");
        OLED::showNumber(26, 40, myabs((int)Control::getMotorRight()), 4, 12);
    }
    
    if(Velocity_Right < 0)    OLED::showString(60, 40, (const uint8_t*)"-");
    if(Velocity_Right >= 0)   OLED::showString(60, 40, (const uint8_t*)"+");
    OLED::showNumber(68, 40, myabs((int)Velocity_Right), 4, 12);
    OLED::showString(96, 40, (const uint8_t*)"mm/s");
    
    // Sixth row: voltage and switch status
    OLED::showString(0, 50, (const uint8_t*)"V");
    OLED::showString(30, 50, (const uint8_t*)".");
    OLED::showString(64, 50, (const uint8_t*)"V");
    OLED::showNumber(19, 50, Control::getVoltage()/100, 2, 12);
    OLED::showNumber(42, 50, Control::getVoltage()/10%10, 1, 12);
    OLED::showNumber(50, 50, Control::getVoltage()%10, 1, 12);
    
    if(Control::getFlagStop())     OLED::showString(95, 50, (const uint8_t*)"OFF");
    if(!Control::getFlagStop())    OLED::showString(95, 50, (const uint8_t*)"ON ");
    
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
    Voltage_Show = (Control::getVoltage() - 1000) * 2 / 3;
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
    float Vol;
    unsigned char Send_Count;
    
    Vol = (float)Control::getVoltage() / 100;
    
    // Set channel data
    DataScope_DP::setChannelData(Control::getAngleBalance(), 1);   // Show angle (degrees)
    DataScope_DP::setChannelData(Control::getDistance()/10, 2);     // Show ultrasonic distance (cm)
    DataScope_DP::setChannelData(Vol, 3);             // Show battery voltage (V)
    
    Send_Count = DataScope_DP::generateDataFrame(3);
    
    // Send data - STM32 USART code removed for Raspberry Pi compatibility
    // TODO: Implement data transmission using appropriate communication method
    // (e.g., printf, file output, or network transmission)
}

/**
 * Return absolute value
 */
int Show::myabs(int a)
{
    if(a < 0) return -a;
    else return a;
}
