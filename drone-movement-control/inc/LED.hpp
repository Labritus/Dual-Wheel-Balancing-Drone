#ifndef __LED_HPP
#define __LED_HPP

#include "System.hpp"

// LED port definition
#define LED PAout(4) // PA4

class LED1 {
public:
    // Initialize LED
    static void init();
    
    // Flash the LED
    // time: flashing interval in ms, 0 = always on
    static void flash(uint16_t time);
};

#endif // __LED_HPP
