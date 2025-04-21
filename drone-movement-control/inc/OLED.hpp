#ifndef __OLED_HPP
#define __OLED_HPP

#include "System.hpp"
#include "oledfont.hpp"

// OLED control functions
class OLED {
public:
    // Initialize the OLED
    static void init();
    
    // Clear the screen
    static void clear();
    
    // Turn on OLED display
    static void displayOn();
    
    // Turn off OLED display
    static void displayOff();
    
    // Refresh the display
    static void refresh();
    
    static void DrawPoint(u8 x, u8 y, u8 t);

    // Display a character at a specified position
    // x: 0~127
    // y: 0~63
    // chr: character to display
    // size: font size 16/12
    static void showChar(uint8_t x, uint8_t y, uint8_t chr, int size,  uint8_t mode);
    
    // Display a string at a specified position
    // x: 0~127
    // y: 0~63
    // str: string to display
    // size: font size 16/12
    static void showString(uint8_t x, uint8_t y, const uint8_t *p);
    
    // Display an integer number
    // x, y: starting coordinates
    // num: number (0~4294967295)
    // len: length (number of digits to display)
    // size: font size
    static void showNumber(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);
    
    // Display a floating point number
    // x, y: starting coordinates
    // num: floating point number
    // len: number of digits before the decimal point
    // decimal: number of digits after the decimal point
    // size: font size
    static void showFloat(uint8_t x, uint8_t y, float num, uint8_t len, uint8_t decimal, uint8_t size);
    
    // Display status information of the self-balancing car

private:
    // Write a command to the OLED
    static void writeCommand(uint8_t cmd);
    
    // Write data to the OLED
    static void writeData(uint8_t data);
    
    // Set cursor position
    static void setCursor(uint8_t x, uint8_t y);
};

#endif // __OLED_HPP
