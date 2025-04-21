#include "Timer.hpp"
#include "Delay.hpp"

// Global variables
static uint16_t TIM2CH2_CAPTURE_STA = 0, TIM2CH2_CAPTURE_VAL = 0;

// Initialize TIM2 for input capture
void Timer::initCapture(uint16_t arr, uint16_t psc)
{
    RCC->APB1ENR |= 1 << 0;      // Enable TIM2 clock     
    RCC->APB2ENR |= 1 << 2;      // Enable PORTA clock        
    GPIOA->CRL &= 0XFFFF0F0F; 
    GPIOA->CRL |= 0X00002080;    // PA.1 as input, PA.3 as output
    GPIOA->CRL |= 3 << 4;
    
    TIM2->ARR = arr;             // Set auto-reload value   
    TIM2->PSC = psc;             // Set prescaler
    TIM2->CCMR1 |= 1 << 8;       // Select input source 
    TIM2->CCMR1 |= 0 << 12;      // Input capture filter: none
    TIM2->CCMR1 |= 0 << 10;      // No prescaler on input capture

    TIM2->CCER |= 0 << 5;        // Capture on rising edge
    TIM2->CCER |= 1 << 4;        // Enable capture into capture register

    TIM2->DIER |= 1 << 2;        // Enable capture interrupt                
    TIM2->DIER |= 1 << 0;        // Enable update interrupt    
    TIM2->CR1 |= 0x01;           // Enable TIM2
    System::nvicInit(1, 1, TIM2_IRQn, 2); // Set NVIC interrupt for TIM2
}

// Get TIM2 capture status
uint8_t Timer::getCaptureStatus()
{
    return TIM2CH2_CAPTURE_STA;
}

// Get high level duration from TIM2
uint32_t Timer::getCaptureHighTime()
{
    uint32_t highTime = 0;
    if(TIM2CH2_CAPTURE_STA & 0X80) // Successfully captured a high-level pulse
    {
        highTime = TIM2CH2_CAPTURE_STA & 0X3F;
        highTime *= 65536;                      // Overflow time sum
        highTime += TIM2CH2_CAPTURE_VAL;        // Total high-level duration
        TIM2CH2_CAPTURE_STA = 0;                // Reset capture for next round
    }
    return highTime;
}

// Ultrasonic ranging initialization
extern "C" {

    // Initialize TIM2 for input capture
    void TIM2_Cap_Init(uint16_t arr, uint16_t psc)
    {
        Timer::initCapture(arr, psc);
    }

    // Read distance
    void Read_Distane(void)
    {   
        PAout(3) = 1;
        Delay::us(15);  
        PAout(3) = 0;    
        
        if(TIM2CH2_CAPTURE_STA & 0X80) // Successfully captured a high-level pulse
        {
            Distance = TIM2CH2_CAPTURE_STA & 0X3F;
            Distance *= 65536;                      // Overflow time sum
            Distance += TIM2CH2_CAPTURE_VAL;        // Total high-level duration
            Distance = Distance * 170 / 1000;       // Convert time to distance
            TIM2CH2_CAPTURE_STA = 0;                // Reset for next capture
        }                
    }

    // TIM2 interrupt handler
    void TIM2_IRQHandler(void)
    { 
        uint16_t tsr;
        tsr = TIM2->SR;
        if((TIM2CH2_CAPTURE_STA & 0X80) == 0) // Not yet successfully captured
        {
            if(tsr & 0X01) // Overflow
            {    
                if(TIM2CH2_CAPTURE_STA & 0X40) // Already captured rising edge
                {
                    if((TIM2CH2_CAPTURE_STA & 0X3F) == 0X3F) // High pulse too long
                    {
                        TIM2CH2_CAPTURE_STA |= 0X80; // Mark as successfully captured
                        TIM2CH2_CAPTURE_VAL = 0XFFFF;
                    } else {
                        TIM2CH2_CAPTURE_STA++;
                    }
                }     
            }
            if(tsr & 0x04) // Capture event occurred on channel 2
            {    
                if(TIM2CH2_CAPTURE_STA & 0X40)        // Falling edge captured         
                {                
                    TIM2CH2_CAPTURE_STA |= 0X80;      // Mark high-level width captured
                    TIM2CH2_CAPTURE_VAL = TIM2->CCR2; // Read capture value
                    TIM2->CCER &= ~(1 << 5);          // Set to capture rising edge
                }
                else                                  // First capture, rising edge
                {
                    TIM2CH2_CAPTURE_STA = 0;          // Clear state
                    TIM2CH2_CAPTURE_VAL = 0;
                    TIM2CH2_CAPTURE_STA |= 0X40;      // Mark rising edge detected
                    TIM2->CNT = 0;                    // Reset counter
                    TIM2->CCER |= 1 << 5;             // Set to capture falling edge
                }            
            }                                         
        }
        TIM2->SR = 0; // Clear interrupt flags         
    }
}
