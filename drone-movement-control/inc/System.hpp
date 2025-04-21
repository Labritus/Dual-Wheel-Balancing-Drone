#ifndef __SYSTEM_HPP
#define __SYSTEM_HPP

#include <stm32f10x.h>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cmath>

// Bit-banding operations to achieve 8051-like GPIO control
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 

// IO port address mapping
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12)    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12)    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) 
 
// IO operation macros
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)

// Definitions for Ex_NVIC_Config
#define GPIO_A 0
#define GPIO_B 1
#define GPIO_C 2
#define GPIO_D 3
#define GPIO_E 4
#define GPIO_F 5
#define GPIO_G 6 

#define FTIR   1  // Falling edge trigger
#define RTIR   2  // Rising edge trigger

// JTAG mode settings
#define JTAG_SWD_DISABLE   0X02  // Disable JTAG and SWD
#define SWD_ENABLE         0X01  // Enable SWD, disable JTAG
#define JTAG_SWD_ENABLE    0X00  // Enable both JTAG and SWD

// System class
class System {
public:
    static void clockInit(uint8_t PLL);  // Clock initialization
    static void softReset(void);         // Software reset
    static void standby(void);           // Enter standby mode
    static void nvicSetVectorTable(uint32_t NVIC_VectTab, uint32_t Offset); // Set vector table offset
    static void nvicPriorityGroupConfig(uint8_t NVIC_Group); // Set NVIC priority grouping
    static void nvicInit(uint8_t NVIC_PreemptionPriority, uint8_t NVIC_SubPriority, uint8_t NVIC_Channel, uint8_t NVIC_Group); // Configure interrupt
    static void exNvicConfig(uint8_t GPIOx, uint8_t BITx, uint8_t TRIM); // External interrupt configuration
    static void jtagSet(uint8_t mode); // JTAG mode setting

    // The following are assembly functions
    static void wfiSet(void);          // Execute WFI instruction
    static void intxDisable(void);     // Disable all interrupts
    static void intxEnable(void);      // Enable all interrupts
    static void msrMsp(uint32_t addr); // Set MSP (Main Stack Pointer) address
};

// Global variable declarations
extern uint8_t Way_Angle;                  // Angle acquisition algorithm: 1-DMP; 2-Kalman; 3-Complementary filter
extern int Motor_Left, Motor_Right;        // Motor PWM variables
extern uint8_t Flag_front, Flag_back, Flag_Left, Flag_Right, Flag_velocity; // Bluetooth control flags
extern uint8_t Flag_Stop, Flag_Show;       // Stop flag and display flag
extern int Voltage;                        // Battery voltage
extern float Angle_Balance, Gyro_Balance, Gyro_Turn; // Balance angle, balance gyroscope, turn gyroscope
extern int Temperature;                    // Temperature
extern uint32_t Distance;                  // Ultrasonic distance
extern uint8_t Flag_avoid, delay_50, delay_flag, PID_Send, Flag_follow;
extern float Acceleration_Z;
extern float Balance_Kp, Balance_Kd, Velocity_Kp, Velocity_Ki, Turn_Kp, Turn_Kd;

#endif // __SYSTEM_HPP
