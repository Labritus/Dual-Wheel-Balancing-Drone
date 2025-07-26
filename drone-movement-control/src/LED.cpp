#include "LED.hpp"
#include "GPIOHelper.hpp"

// Initialize LED
void LED1::init()
{
    if (!GPIOHelper::isInitialized()) {
        GPIOHelper::init();
    }
    GPIOHelper::setMode(4, GPIOMode::OUTPUT);
    GPIOHelper::setValue(4, GPIOValue::HIGH);
}

// LED blink
void LED1::flash(uint16_t time)
{
    static int temp;
    if (0 == time) 
        GPIOHelper::setValue(4, GPIOValue::LOW);  // Turn off LED immediately
    else if (++temp == time) {
        GPIOValue current = GPIOHelper::getValue(4);
        GPIOHelper::setValue(4, (current == GPIOValue::HIGH) ? GPIOValue::LOW : GPIOValue::HIGH);
        temp = 0;
    }
}
