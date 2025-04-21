#include "Delay.hpp"

static uint8_t  fac_us = 0; // Microsecond delay multiplier
static uint16_t fac_ms = 0; // Millisecond delay multiplier

// Initialize delay function
void Delay::init(uint8_t SYSCLK)
{
    SysTick->CTRL &= ~(1 << 2); // Select external clock, HCLK/8
    fac_us = SYSCLK / 8;        // 1/8 of system clock
    fac_ms = (uint16_t)fac_us * 1000;
}

// Delay for nus microseconds
void Delay::us(uint32_t nus)
{
    uint32_t temp;
    SysTick->LOAD = nus * fac_us;              // Set countdown time
    SysTick->VAL = 0x00;                       // Clear counter
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; // Start countdown
    
    do {
        temp = SysTick->CTRL;
    } while ((temp & 0x01) && !(temp & (1 << 16))); // Wait until time is up
    
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; // Stop counter
    SysTick->VAL = 0X00;                       // Clear counter
}

// Delay for nms milliseconds
void Delay::ms(uint16_t nms)
{
    uint32_t temp;
    SysTick->LOAD = (uint32_t)nms * fac_ms;     // Set countdown time (SysTick->LOAD is 24-bit)
    SysTick->VAL = 0x00;                        // Clear counter
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;   // Start countdown
    
    do {
        temp = SysTick->CTRL;
    } while ((temp & 0x01) && !(temp & (1 << 16))); // Wait until time is up
    
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;  // Stop counter
    SysTick->VAL = 0X00;                        // Clear counter
}
