#include "Ultrasonic.hpp"
#include "Delay.hpp"

// Initialize ultrasonic sensor
void Ultrasonic::init()
{
    // Initialize IO pins for ultrasonic measurement
    RCC->APB2ENR |= 1 << 4; // Enable clock for PORTC

    // Configure PC1 as output for TRIG signal
    GPIOC->CRL &= 0xFFFFFF0F;
    GPIOC->CRL |= 0x00000030; // PC1 push-pull output

    // Configure PC2 as input for ECHO signal
    GPIOC->CRL &= 0xFFFFF0FF;
    GPIOC->CRL |= 0x00000800; // PC2 pull-up/down input
    GPIOC->ODR |= 1 << 2;     // Pull-up on PC2

    // Initial state: TRIG pin low
    TRIG = 0;
}

// Measure distance
void Ultrasonic::readDistance()
{
    uint32_t t = 0;

    // Send a trigger pulse: high level pulse >10us
    TRIG = 1;
    Delay::us(20);
    TRIG = 0;

    // Wait for rising edge of echo signal
    while (ECHO == 0) {
        t++;
        Delay::us(1);
        if (t > 100000) return; // Timeout
    }

    // Measure duration of echo signal
    t = 0;
    while (ECHO == 1) {
        t++; 
        Delay::us(10); // Count every 10us
        if (t > 10000) {
            Distance = 0;
            return; // Timeout
        }
    }

    // Calculate distance (unit: mm), speed of sound = 340 m/s = 0.34 mm/us
    // t * 10us * 0.34 mm/us / 2 = t * 1.7 mm
    Distance = t * 17 / 10; // Final distance calculation
}
