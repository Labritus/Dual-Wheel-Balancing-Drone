#ifndef __ULTRASONIC_HPP
#define __ULTRASONIC_HPP

#include "System.hpp"

// Ultrasonic sensor pin definitions
#define TRIG PCout(1)  // Trigger signal (send)
#define ECHO PCin(2)   // Echo signal (receive)

class Ultrasonic {
public:
    // Initialize ultrasonic sensor
    static void init();
    
    // Measure distance
    static void readDistance();
};

#endif // __ULTRASONIC_HPP
