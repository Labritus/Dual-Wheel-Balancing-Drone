#ifndef __ADC_HPP
#define __ADC_HPP

#include "System.hpp"

#define Battery_Ch 8

class ADC {
public:
    // Initialize ADC
    static void init();
    
    // Perform ADC sampling
    // ch: ADC1 channel
    // return: ADC conversion result
    static uint16_t getAdc(uint8_t ch);
    
    // Read battery voltage
    // return: battery voltage in millivolts (mV)
    static int getBatteryVolt();
};

#endif // __ADC_HPP
