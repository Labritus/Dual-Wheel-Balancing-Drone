#include "ADC.hpp"

// Initialize ADC
void ADC::init()
{    
    // First initialize IO port
    RCC->APB2ENR |= 1 << 3;      // Enable PORTA clock 
    GPIOB->CRL &= 0XFFFFFFF0;    // PB0 analog input 
    RCC->APB2ENR |= 1 << 9;      // Enable ADC1 clock    
    RCC->APB2RSTR |= 1 << 9;     // Reset ADC1
    RCC->APB2RSTR &= ~(1 << 9);  // End of reset    
    RCC->CFGR &= ~(3 << 14);     // Clear prescaler bits    
    // SYSCLK/DIV2 = 12M, ADC clock set to 12M, ADC max clock must not exceed 14M!
    // Otherwise, ADC accuracy will decrease!
    RCC->CFGR |= 2 << 14;           
    ADC1->CR1 &= 0XF0FFFF;       // Clear operation mode bits
    ADC1->CR1 |= 0 << 16;        // Independent mode  
    ADC1->CR1 &= ~(1 << 8);      // Non-scan mode    
    ADC1->CR2 &= ~(1 << 1);      // Single conversion mode
    ADC1->CR2 &= ~(7 << 17);       
    ADC1->CR2 |= 7 << 17;        // Software controlled conversion  
    ADC1->CR2 |= 1 << 20;        // Use external trigger (SWSTART)!!! A trigger event is required
    ADC1->CR2 &= ~(1 << 11);     // Right alignment     
    ADC1->SQR1 &= ~(0XF << 20);
    ADC1->SQR1 &= 0 << 20;       // 1 conversion in the regular sequence, i.e., only convert regular sequence 1                
    // Set sampling time for channel 4
    ADC1->SMPR2 &= 0XF0FFFFFF;   // Clear sampling time    
    ADC1->SMPR2 |= 7 << 18;      // 239.5 cycles, increasing sampling time improves accuracy        

    ADC1->CR2 |= 1 << 0;         // Enable ADC conversion    
    ADC1->CR2 |= 1 << 3;         // Enable reset calibration  
    while(ADC1->CR2 & 1 << 3);   // Wait for calibration to finish             
    // This bit is set by software and cleared by hardware. It is cleared after calibration registers are initialized.      
    ADC1->CR2 |= 1 << 2;         // Start ADC calibration    
    while(ADC1->CR2 & 1 << 2);   // Wait for calibration to finish
}        

// ADC sampling
uint16_t ADC::getAdc(uint8_t ch)   
{
    // Set conversion sequence    
    ADC1->SQR3 &= 0XFFFFFFE0;   // Regular sequence 1, channel ch
    ADC1->SQR3 |= ch;                     
    ADC1->CR2 |= 1 << 22;       // Start regular channel conversion 
    while(!(ADC1->SR & 1 << 1)); // Wait for conversion to finish        
    return ADC1->DR;            // Return ADC value    
}

// Read battery voltage
int ADC::getBatteryVolt()   
{  
    int Volt; // Battery voltage
    Volt = getAdc(Battery_Ch) * 3.3 * 11 * 100 / 4096;   // Calculate voltage; refer to schematic diagram for analysis    
    return Volt;
}  
