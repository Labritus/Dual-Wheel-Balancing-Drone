#include "../inc/Encoder.hpp"

// Initialize TIM3 as encoder interface mode
void Encoder::initTIM3()
{
    RCC->APB1ENR |= 1 << 1;       // Enable TIM3 clock
    RCC->APB2ENR |= 1 << 2;       // Enable PORTA clock
    GPIOA->CRL &= 0X00FFFFFF;     // Configure PA6 PA7
    GPIOA->CRL |= 0X44000000;     // Floating input
    
    TIM3->DIER |= 1 << 0;         // Enable update interrupt            
    TIM3->DIER |= 1 << 6;         // Enable trigger interrupt
    System::nvicInit(1, 3, TIM3_IRQn, 1);  // Set interrupt priority
    
    TIM3->PSC = 0x0;              // Prescaler
    TIM3->ARR = ENCODER_TIM_PERIOD; // Set auto-reload value 
    TIM3->CR1 &= ~(3 << 8);       // Select clock division: no division
    TIM3->CR1 &= ~(3 << 5);       // Select count mode: edge-aligned mode
        
    TIM3->CCMR1 |= 1 << 0;        // CC1S='01' IC1FP1 mapped to TI1
    TIM3->CCMR1 |= 1 << 8;        // CC2S='01' IC2FP2 mapped to TI2
    TIM3->CCER &= ~(1 << 1);      // CC1P='0' IC1FP1 non-inverted, IC1FP1=TI1
    TIM3->CCER &= ~(1 << 5);      // CC2P='0' IC2FP2 non-inverted, IC2FP2=TI2
    TIM3->CCMR1 |= 3 << 4;        // IC1F='1000' Input capture 1 filter
    TIM3->SMCR |= 3 << 0;         // SMS='011' Both rising and falling edges are valid
    TIM3->CR1 |= 0x01;            // CEN=1, enable timer
}

// Initialize TIM4 as encoder interface mode
void Encoder::initTIM4()
{
    RCC->APB1ENR |= 1 << 2;       // Enable TIM4 clock
    RCC->APB2ENR |= 1 << 3;       // Enable PORTB clock
    GPIOB->CRL &= 0X00FFFFFF;     // Configure PB6 PB7
    GPIOB->CRL |= 0X44000000;     // Floating input
    
    TIM4->DIER |= 1 << 0;         // Enable update interrupt              
    TIM4->DIER |= 1 << 6;         // Enable trigger interrupt
    System::nvicInit(1, 3, TIM4_IRQn, 1);  // Set interrupt priority
    
    TIM4->PSC = 0x0;              // Prescaler
    TIM4->ARR = ENCODER_TIM_PERIOD; // Set auto-reload value 
    TIM4->CR1 &= ~(3 << 8);       // Select clock division: no division
    TIM4->CR1 &= ~(3 << 5);       // Select count mode: edge-aligned mode
        
    TIM4->CCMR1 |= 1 << 0;        // CC1S='01' IC1FP1 mapped to TI1
    TIM4->CCMR1 |= 1 << 8;        // CC2S='01' IC2FP2 mapped to TI2
    TIM4->CCER &= ~(1 << 1);      // CC1P='0' IC1FP1 non-inverted, IC1FP1=TI1
    TIM4->CCER &= ~(1 << 5);      // CC2P='0' IC2FP2 non-inverted, IC2FP2=TI2
    TIM4->CCMR1 |= 3 << 4;        // IC1F='1000' Input capture 1 filter
    TIM4->SMCR |= 3 << 0;         // SMS='011' Both rising and falling edges are valid
    TIM4->CR1 |= 0x01;            // CEN=1, enable timer
}

// Read encoder count within a unit time
int Encoder::read(uint8_t TIMX)
{
    int Encoder_TIM;    
    switch(TIMX)
    {
        case 2:  
            Encoder_TIM = (short)TIM2->CNT;  
            TIM2->CNT = 0;
            break;
        case 3:  
            Encoder_TIM = (short)TIM3->CNT;  
            TIM3->CNT = 0;
            break;	
        case 4:  
            Encoder_TIM = (short)TIM4->CNT;  
            TIM4->CNT = 0;
            break;	
        default:  
            Encoder_TIM = 0;
    }
    return Encoder_TIM;
}

// TIM4 interrupt service routine
extern "C" void TIM4_IRQHandler(void)
{             
    if(TIM4->SR & 0X0001) // Overflow interrupt
    {                          
    }             
    TIM4->SR &= ~(1 << 0); // Clear interrupt flag     
}

// TIM3 interrupt service routine
extern "C" void TIM3_IRQHandler(void)
{             
    if(TIM3->SR & 0X0001) // Overflow interrupt
    {                          
    }             
    TIM3->SR &= ~(1 << 0); // Clear interrupt flag     
}
