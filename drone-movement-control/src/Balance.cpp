#include "Balance.hpp"
#include "Delay.hpp"
#include "USART.hpp"
#include "EXTI.hpp"
#include "USART3.hpp"
#include "Show.hpp" 

// Initialize callback function pointer
BalanceCallback Balance::m_callback = 0;

// Initialize the balance car
void Balance::init()
{
    System::clockInit(9);                // System clock configuration
    System::nvicPriorityGroupConfig(2);  // Interrupt priority grouping
		Delay::init(72);                      // Initialize delay
    System::jtagSet(JTAG_SWD_DISABLE);   // Disable JTAG interface
    System::jtagSet(SWD_ENABLE);         // Enable SWD interface for online debugging
    
		LED1::init();                         // Initialize hardware interface related to LED
    Key::init();                         // Initialize buttons
    OLED::init();                        // Initialize OLED
    USART::init(72, 115200);             // Initialize UART1
    USART3Driver::init(36, 9600);        // Initialize UART3
    Motor::pwmInit(7199, 0);             // Initialize PWM at 10KHz for motor driving.
                                          // To change frequency: Motor::pwmInit(9999,35) for 200Hz
    Encoder::initTIM3();                 // Initialize encoder interface
    Encoder::initTIM4();                 // Initialize second encoder 
    ADC::init();                         // Initialize ADC
    IOI2C::init();                       // Initialize software IIC
    MPU6050::initialize();               // Initialize MPU6050    
    MPU6050::dmpInit();                  // Initialize DMP     
    Ultrasonic::init();                  // Initialize ultrasonic sensor
    EXTI1::init();                       // Initialize MPU6050 internal 5ms data ready interrupt
                                         // because data acquisition rate is 200Hz 
}

// Run the balance car
void Balance::run()
{
    while(1)
    {     
        if(Flag_Show == 0)               // Use MiniBalance APP and OLED display
        {
            Show::appShow();   
            Show::oledShow();            // Display on OLED
        }
        else                             // Use MiniBalance upper computer; disable app and OLED display during use
        {
            Show::dataScope();           // Send data to MiniBalance upper computer
        }
        
        handleCallback();                // Handle callback function
    }
}

// Set callback function
void Balance::setCallback(BalanceCallback callback)
{
    m_callback = callback;
}

// Handle callback
void Balance::handleCallback()
{
    if (m_callback != 0) {
        // Get current sensor data
        int left_speed = Encoder::read(3);
        int right_speed = Encoder::read(4);
        
        // Call callback function
        m_callback(Angle_Balance, Gyro_Balance, left_speed, right_speed);
    }
}  
