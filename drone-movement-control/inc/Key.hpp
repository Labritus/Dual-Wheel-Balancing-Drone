#ifndef __KEY_HPP
#define __KEY_HPP

#include "System.hpp"

#define KEY PAin(5)

class Key {
public:
    // Initialize the key
    static void init();
    
    // Scan for click and double-click
    // time: wait time for detecting double click
    // Return: 0 = no action, 1 = single click, 2 = double click
    static uint8_t clickNDouble(uint8_t time);
    
    // Scan for single click only
    // Return: 0 = no action, 1 = single click
    static uint8_t click();
    
    // Detect long press
    // Return: 0 = no action, 1 = long press (2 seconds)
    static uint8_t longPress();
    
    // Read and interpret key state (combined detection)
    // Return: 0 = no action, 1 = double click, 2 = single click, 3 = long press
    static uint8_t keyRead();
};

#endif // __KEY_HPP
