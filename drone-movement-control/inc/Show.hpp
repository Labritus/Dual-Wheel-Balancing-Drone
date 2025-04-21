#ifndef __SHOW_HPP
#define __SHOW_HPP

#include "System.hpp"
#include "OLED.hpp"
#include "USART3.hpp"
#include "DataScope_DP.hpp"

// Encoder to velocity conversion coefficient
#define ENCODER_TO_VELOCITY 0.5397  // Velocity per pulse; wheel diameter is 67mm, 390 pulses per rotation, frequency division factor is 1560

// Global variable declarations
extern float Velocity_Left, Velocity_Right;  // Left and right wheel velocities

/**
 * Display functionality class
 */
class Show {
public:
    /**
     * OLED display
     */
    static void oledShow();

    static bool waveformEnable0d;

    /**
     * Data display on mobile app
     */
    static void appShow();
    
    /**
     * Oscilloscope display for upper computer
     */
    static void dataScope();
    
    /**
     * Get absolute value
     */
    static int myabs(int a);
};

#endif // __SHOW_HPP
