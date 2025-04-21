#include "LED.hpp"

// Initialize LED
void LED1::init()
{
    RCC->APB2ENR |= 1 << 2;          // Enable PORTA clock  
    GPIOA->CRL &= 0XFFF00FFF;
    GPIOA->CRL |= 0X00033000;        // Set PA4 as push-pull output
    GPIOA->ODR |= 1 << 4;            // Set PA4 high
}

// LED blink
void LED1::flash(uint16_t time)
{
    static int temp;
    if (0 == time) 
        LED = 0;                     // Turn off LED immediately
    else if (++temp == time)
        LED = ~LED, temp = 0;        // Toggle LED and reset counter
}
