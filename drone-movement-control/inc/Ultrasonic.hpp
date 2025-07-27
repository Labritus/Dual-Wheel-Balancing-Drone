#ifndef __ULTRASONIC_HPP
#define __ULTRASONIC_HPP

#include "System.hpp"

// Ultrasonic sensor - GPIO pin definitions moved to implementation
// TRIG and ECHO pins are now handled by GPIOHelper in the .cpp file

class Ultrasonic {
public:
    // Initialize ultrasonic sensor
    static void init();
    
    // Measure distance
    static void readDistance();
};

#endif // __ULTRASONIC_HPP
