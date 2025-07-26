#include "Key.hpp"
#include "GPIOHelper.hpp"

// Initialize the key (button)
void Key::init()
{
    if (!GPIOHelper::isInitialized()) {
        GPIOHelper::init();
    }
    GPIOHelper::setMode(5, GPIOMode::INPUT);
} 

// Button scan, supports single and double click detection
uint8_t Key::clickNDouble(uint8_t time)
{
    static uint8_t flag_key, count_key, double_key;    
    static uint16_t count_single, Forever_count;
    
    if (GPIOHelper::getValue(5) == GPIOValue::LOW)  
        Forever_count++;           // Long press flag not set
    else        
        Forever_count = 0;
        
    if (GPIOHelper::getValue(5) == GPIOValue::LOW && 0 == flag_key)        
        flag_key = 1;              // First press detected
        
    if (0 == count_key)                                            
    {
        if (flag_key == 1) 
        {
            double_key++;
            count_key = 1;         // Mark that a press occurred
        }
        if (double_key == 2) 
        {                          // Second press detected
            double_key = 0;
            count_single = 0;
            return 2;              // Execute double-click command
        }
    }
    if (GPIOHelper::getValue(5) == GPIOValue::HIGH)            
        flag_key = 0, count_key = 0;
    
    if (1 == double_key)
    {
        count_single++;
        if (count_single > time && Forever_count < time)  
        {            
            double_key = 0;
            count_single = 0;      // Timeout waiting for second click
            return 1;              // Execute single-click command
        }
        if (Forever_count > time)
        {
            double_key = 0;
            count_single = 0;    
        }
    }    
    return 0;
}

// Button scan, only detects single click
uint8_t Key::click()
{
    static uint8_t flag_key = 1;   // Button release flag
    if (flag_key && GPIOHelper::getValue(5) == GPIOValue::LOW)      // Button pressed
    {
        flag_key = 0;
        return 1;                  // Return press event
    }
    else if (GPIOHelper::getValue(5) == GPIOValue::HIGH)             
        flag_key = 1;
    return 0;                      // No press detected
}

// Long press detection
uint8_t Key::longPress()
{
    static uint16_t Long_Press_count, Long_Press;
    if (Long_Press == 0 && GPIOHelper::getValue(5) == GPIOValue::LOW)  
        Long_Press_count++;        // Long press flag not yet set
    else                       
        Long_Press_count = 0; 
        
    if (Long_Press_count > 200)    // 10ms scan interval
    {
        Long_Press = 1;    
        Long_Press_count = 0;
        return 1;
    }                
    if (Long_Press == 1)           // Long press flag was set
    {
        Long_Press = 0;
    }
    return 0;
}

// Combined key reading function
uint8_t Key::keyRead()
{
    // Check for double click first
    uint8_t doubleClickResult = clickNDouble(50);
    if (doubleClickResult == 2)
        return 1;   // Double click
    else if (doubleClickResult == 1)
        return 2;   // Single click
    
    // Check for long press
    if (longPress())
        return 3;   // Long press
    
    return 0;       // No action
}
