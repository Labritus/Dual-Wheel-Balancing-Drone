#ifndef __OLED_HPP
#define __OLED_HPP

#include <stdint.h>
#include <functional>

// Type definitions for compatibility
typedef uint8_t u8;

class OLED {
public:
    // Initialize OLED display
    static void init();
    
    // Async initialization
    static void initAsync(std::function<void()> callback);
    
    // Clear display
    static void clear();
    
    // Display control
    static void displayOn();
    static void displayOff();
    
    // Refresh display
    static void refresh();
    
    // Set cursor position
    static void setCursor(uint8_t x, uint8_t y);
    
    // Draw point
    static void DrawPoint(u8 x, u8 y, u8 t);
    
    // Show character
    static void showChar(uint8_t x, uint8_t y, uint8_t chr, int size, uint8_t mode);
    
    // Show string
    static void showString(uint8_t x, uint8_t y, const uint8_t *p);
    
    // Show number
    static void showNumber(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);
    
    // Show float
    static void showFloat(uint8_t x, uint8_t y, float num, uint8_t len, uint8_t decimal, uint8_t size);

private:
    // Write command to OLED
    static void writeCommand(uint8_t cmd);
    
    // Write data to OLED
    static void writeData(uint8_t data);
};

#endif // __OLED_HPP