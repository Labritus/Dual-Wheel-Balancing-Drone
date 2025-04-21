#include "USART3.hpp"
extern "C" {
#include <math.h>     // pow
#include <string.h>   // memset
}

// Static member variable definition
uint8_t USART3Driver::receive_data = 0;

// Global variable - for compatibility with legacy code
uint8_t Usart3_Receive = 0;

// Initialize USART3
void USART3Driver::init(uint32_t pclk2, uint32_t bound)
{
    float temp;
    uint16_t mantissa;
    uint16_t fraction;
    
    temp = (float)(pclk2 * 1000000) / (bound * 16); // Calculate USARTDIV
    mantissa = temp;                                 // Integer part
    fraction = (temp - mantissa) * 16;               // Fractional part
    mantissa <<= 4;
    mantissa += fraction;
    
    RCC->APB2ENR |= 1 << 3;   // Enable PORTB clock
    RCC->APB1ENR |= 1 << 18;  // Enable USART3 clock
    GPIOB->CRH &= 0XFFFF00FF;
    GPIOB->CRH |= 0X00008B00; // Configure IO state
    GPIOB->ODR |= 1 << 10;
    
    RCC->APB1RSTR |= 1 << 18;     // Reset USART3
    RCC->APB1RSTR &= ~(1 << 18);  // Stop reset
    
    // Set baud rate
    USART3->BRR = mantissa;
    USART3->CR1 |= 0X200C;     // 1 stop bit, no parity
    
    // Enable receive interrupt
    USART3->CR1 |= 1 << 8;     // Enable PE interrupt
    USART3->CR1 |= 1 << 5;     // Enable RXNE (receive not empty) interrupt
    System::nvicInit(0, 1, USART3_IRQn, 2); // Group 2
}

// Get received data
uint8_t USART3Driver::getReceiveData()
{
    return receive_data;
}

// Handle received command
void USART3Driver::processCommand(uint8_t command)
{
    // Handle speed control commands
    if(command == 0x59) Flag_velocity = 2;  // High speed mode (default)
    if(command == 0x58) Flag_velocity = 1;  // Low speed mode
    
    // Handle ASCII commands (for app: A~H, Z)
    if(command > 10)
    {
        if(command == 0x5A)       Flag_front = 0, Flag_back = 0, Flag_Left = 0, Flag_Right = 0; // Brake
        else if(command == 0x41)  Flag_front = 1, Flag_back = 0, Flag_Left = 0, Flag_Right = 0; // Forward
        else if(command == 0x45)  Flag_front = 0, Flag_back = 1, Flag_Left = 0, Flag_Right = 0; // Backward
        else if(command == 0x42 || command == 0x43 || command == 0x44)  
                                  Flag_front = 0, Flag_back = 0, Flag_Left = 0, Flag_Right = 1;  // Right
        else if(command == 0x46 || command == 0x47 || command == 0x48)      
                                  Flag_front = 0, Flag_back = 0, Flag_Left = 1, Flag_Right = 0;  // Left
        else Flag_front = 0, Flag_back = 0, Flag_Left = 0, Flag_Right = 0; // Brake
    }
    
    // Handle HEX commands (for MiniBalance V1.0 remote control)
    if(command < 10)
    {
        if(command == 0x00)       Flag_front = 0, Flag_back = 0, Flag_Left = 0, Flag_Right = 0; // Brake
        else if(command == 0x01)  Flag_front = 1, Flag_back = 0, Flag_Left = 0, Flag_Right = 0; // Forward
        else if(command == 0x05)  Flag_front = 0, Flag_back = 1, Flag_Left = 0, Flag_Right = 0; // Backward
        else if(command == 0x02 || command == 0x03 || command == 0x04)  
                                  Flag_front = 0, Flag_back = 0, Flag_Left = 0, Flag_Right = 1;  // Right
        else if(command == 0x06 || command == 0x07 || command == 0x08)      
                                  Flag_front = 0, Flag_back = 0, Flag_Left = 1, Flag_Right = 0;  // Left
        else Flag_front = 0, Flag_back = 0, Flag_Left = 0, Flag_Right = 0; // Brake
    }
}

// USART3 interrupt handler
void USART3Driver::irqHandler()
{
    if(USART3->SR & (1 << 5))  // Received data
    {
        static int uart_receive = 0; // Temporary buffer for received data
        static uint8_t Flag_PID = 0, i = 0, j = 0, Receive[50] = {0};
        static float Data = 0;
        
        uart_receive = USART3->DR;
        receive_data = uart_receive;
        Usart3_Receive = uart_receive;
        
        // Handle basic control commands
        processCommand(uart_receive);
        
        // Handle PID parameter command
        if(Usart3_Receive == 0x7B) Flag_PID = 1;   // Start byte for PID command
        if(Usart3_Receive == 0x7D) Flag_PID = 2;   // End byte for PID command
        
        if(Flag_PID == 1)  // Collect data
        {
            Receive[i] = Usart3_Receive;
            i++;
        }
        
        if(Flag_PID == 2)  // Analyze data
        {
            if(Receive[3] == 0x50) PID_Send = 1;
            else if(Receive[1] != 0x23)
            {
                for(j = i; j >= 4; j--)
                {
                    Data += (Receive[j-1] - 48) * pow(10, double(i-j));
                }
                
                switch(Receive[1])
                {
                    case 0x30: Balance_Kp = Data; break;
                    case 0x31: Balance_Kd = Data; break;
                    case 0x32: Velocity_Kp = Data; break;
                    case 0x33: Velocity_Ki = Data; break;
                    case 0x34: Turn_Kp = Data; break;
                    case 0x35: Turn_Kd = Data; break;
                    case 0x36: break; // Reserved
                    case 0x37: break; // Reserved
                    case 0x38: break; // Reserved
                }
            }
            
            Flag_PID = 0;
            i = 0;
            j = 0;
            Data = 0;
            memset(Receive, 0, sizeof(uint8_t) * 50); // Clear array
        }
    }
}
