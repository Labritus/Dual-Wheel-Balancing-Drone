#include "Balance.hpp"
#include "USART.hpp"
#include "Delay.hpp"
#include "I2CSlave.hpp"
#include "System.hpp"

// Callback function for testing balance car data
void balanceStatusCallback(float angle, float gyro, int left_speed, int right_speed)
{
    // Print current balance car status to serial port
    USART::printf("Angle: %.2f, Gyro: %.2f, Left Speed: %d, Right Speed: %d\r\n", 
                 angle, gyro, left_speed, right_speed);
    
    // Delay to avoid printing too fast
    Delay::ms(500);
    
    // Check for I2C messages
    if (I2CSlave::hasNewMessage()) {
        if (I2CSlave::isPersonDetected()) {
            // Received PERSON_DETECTED message, switch to follow mode
            Flag_follow = 1;
            Flag_avoid = 0;
            USART::printf("I2C received PERSON_DETECTED message, switching to follow mode\r\n");
        }
        I2CSlave::clearMessageFlag();
    }
}

int main(void)
{ 
    // Initialize the balance car
    Balance::init();
    
    // Initialize I2C slave with address 0x08
    I2CSlave::init();
    
    // Set status callback function
    Balance::setCallback(balanceStatusCallback);
    
    // Run the balance car
    Balance::run();
    
    return 0;
}
