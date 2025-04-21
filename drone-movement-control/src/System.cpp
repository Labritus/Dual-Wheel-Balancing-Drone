#include "System.hpp"

// Global variable definitions
uint8_t Way_Angle = 2;                             // Algorithm for obtaining angle
uint8_t Flag_front, Flag_back, Flag_Left, Flag_Right, Flag_velocity = 2; // Bluetooth control related flags
uint8_t Flag_Stop = 1, Flag_Show = 0;              // Stop flag and display flag
int Motor_Left, Motor_Right;                       // Motor PWM values
int Temperature;                                   // Temperature
int Voltage;                                       // Battery voltage
float Angle_Balance, Gyro_Balance, Gyro_Turn;      // Balance angle, gyro for balance, gyro for turning
uint32_t Distance = 0;                             // Ultrasonic distance measurement
uint8_t delay_50, delay_flag, PID_Send;            // Timing and debug-related variables
uint8_t Flag_avoid = 0, Flag_follow;               // Obstacle avoidance / follow mode flags
float Acceleration_Z;                              // Acceleration in Z-axis
float Balance_Kp = 25500, Balance_Kd = 135, Velocity_Kp = 16000, Velocity_Ki = 80, Turn_Kp = 4200, Turn_Kd = 0; // PID parameters

// Set vector table offset address
void System::nvicSetVectorTable(uint32_t NVIC_VectTab, uint32_t Offset) 
{ 
    SCB->VTOR = NVIC_VectTab | (Offset & (uint32_t)0x1FFFFF80);
}

// Set NVIC priority group
void System::nvicPriorityGroupConfig(uint8_t NVIC_Group) 
{ 
    uint32_t temp, temp1;  
    temp1 = (~NVIC_Group) & 0x07; // Take the last three bits
    temp1 <<= 8;
    temp = SCB->AIRCR;  // Read current register
    temp &= 0X0000F8FF; // Clear current group configuration
    temp |= 0X05FA0000; // Write the key
    temp |= temp1;      
    SCB->AIRCR = temp;  // Set group
}

// Configure NVIC interrupt
void System::nvicInit(uint8_t NVIC_PreemptionPriority, uint8_t NVIC_SubPriority, uint8_t NVIC_Channel, uint8_t NVIC_Group) 
{ 
    uint32_t temp;  
    nvicPriorityGroupConfig(NVIC_Group);      // Set priority group
    temp = NVIC_PreemptionPriority << (4 - NVIC_Group);  
    temp |= NVIC_SubPriority & (0x0f >> NVIC_Group);
    temp &= 0xf; // Take lower 4 bits  
    NVIC->ISER[NVIC_Channel / 32] |= (1 << NVIC_Channel % 32); // Enable interrupt
    NVIC->IP[NVIC_Channel] |= temp << 4;                       // Set priority
} 

// External interrupt configuration function
void System::exNvicConfig(uint8_t GPIOx, uint8_t BITx, uint8_t TRIM) 
{
    uint8_t EXTADDR;
    uint8_t EXTOFFSET;
    EXTADDR = BITx / 4; // Get the register group index for the interrupt
    EXTOFFSET = (BITx % 4) * 4; 
    RCC->APB2ENR |= 0x01; // Enable alternate function clock         
    AFIO->EXTICR[EXTADDR] &= ~(0x000F << EXTOFFSET); // Clear previous config
    AFIO->EXTICR[EXTADDR] |= GPIOx << EXTOFFSET;     // Map EXTI.BITx to GPIOx.BITx 
    
    EXTI->IMR |= 1 << BITx;           // Enable interrupt on line BITx
    if (TRIM & 0x01) EXTI->FTSR |= 1 << BITx; // Trigger on falling edge
    if (TRIM & 0x02) EXTI->RTSR |= 1 << BITx; // Trigger on rising edge
}

// Reset all clock registers
static void MYRCC_DeInit(void)
{  
    RCC->APB1RSTR = 0x00000000; // Reset complete         
    RCC->APB2RSTR = 0x00000000; 
    
    RCC->AHBENR = 0x00000014;  // Enable Flash and SRAM clocks in sleep mode, others off    
    RCC->APB2ENR = 0x00000000; // Disable peripheral clocks         
    RCC->APB1ENR = 0x00000000;   
    RCC->CR |= 0x00000001;     // Enable internal high-speed clock (HSION)                                              
    RCC->CFGR &= 0xF8FF0000;   // Reset SW[1:0], HPRE[3:0], PPRE1[2:0], PPRE2[2:0], ADCPRE[1:0], MCO[2:0]                  
    RCC->CR &= 0xFEF6FFFF;     // Reset HSEON, CSSON, PLLON
    RCC->CR &= 0xFFFBFFFF;     // Reset HSEBYP       
    RCC->CFGR &= 0xFF80FFFF;   // Reset PLLSRC, PLLXTPRE, PLLMUL[3:0], and USBPRE 
    RCC->CIR = 0x00000000;     // Disable all interrupts     
                 
#ifdef VECT_TAB_RAM
    System::nvicSetVectorTable(0x20000000, 0x0);
#else   
    System::nvicSetVectorTable(0x08000000, 0x0);
#endif
}

// Enter standby mode
void System::standby(void)
{
    SCB->SCR |= 1 << 2;           // Enable SLEEPDEEP bit in system control register    
    RCC->APB1ENR |= 1 << 28;      // Enable power clock     
    PWR->CSR |= 1 << 8;           // Set WKUP as wake-up source
    PWR->CR |= 1 << 2;            // Clear wake-up flag
    PWR->CR |= 1 << 1;            // Set PDDS bit       
    wfiSet();                     // Execute WFI instruction     
}      

// System software reset
void System::softReset(void)
{   
    SCB->AIRCR = 0X05FA0000 | (uint32_t)0x04;    
}   

// Set JTAG mode
void System::jtagSet(uint8_t mode)
{
    uint32_t temp;
    temp = mode;
    temp <<= 25;
    RCC->APB2ENR |= 1 << 0;      // Enable auxiliary clock      
    AFIO->MAPR &= 0xF8FFFFFF;    // Clear [26:24] in MAPR
    AFIO->MAPR |= temp;          // Set JTAG mode
}

// Initialize system clock
void System::clockInit(uint8_t PLL)
{
    uint8_t CKSEL = 0;
    MYRCC_DeInit();              // Reset and configure vector table
    RCC->CR |= 0x00010000;       // Enable HSE (external high-speed clock)
    while (!(RCC->CR >> 17));    // Wait until HSE is ready
    RCC->CFGR = 0X00000400;      // APB1=DIV2; APB2=DIV1; AHB=DIV1
    PLL -= 2;                    // Get PLL multiplier
    RCC->CFGR |= PLL << 18;      // Set PLL multiplier
    RCC->CFGR |= 1 << 16;        // Set PLLSRC
    FLASH->ACR |= 0x32;          // FLASH latency: 2 wait states
    RCC->CR |= 0x01000000;       // Enable PLL
    while (!(RCC->CR >> 25));    // Wait until PLL is locked
    RCC->CFGR |= 0x00000002;     // Set PLL as system clock
    while (CKSEL != 0x02) {      // Wait until PLL is used as system clock
        CKSEL = RCC->CFGR >> 2;
        CKSEL &= 0x03;
    }
}

// Assembly function wrappers (to keep original functionality)
extern "C" {
    void __WFI_SET(void);
    void __INTX_DISABLE(void);
    void __INTX_ENABLE(void);
    void __MSR_MSP(uint32_t addr);
}

void System::wfiSet(void)
{
    __WFI_SET();
}

void System::intxDisable(void)
{
    __INTX_DISABLE();
}

void System::intxEnable(void)
{
    __INTX_ENABLE();
}

void System::msrMsp(uint32_t addr)
{
    __MSR_MSP(addr);
}

// Actual assembly implementations
__asm void __WFI_SET(void)
{
    WFI;
}

__asm void __INTX_DISABLE(void)
{
    CPSID I;
}

__asm void __INTX_ENABLE(void)
{
    CPSIE I;
}

__asm void __MSR_MSP(uint32_t addr)
{
    MSR MSP, r0;       // Set Main Stack Pointer
    BX r14;
}
