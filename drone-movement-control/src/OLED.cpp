#include "IOI2C.hpp"
#include <math.h>
#include <cstring>
#include "OLED.hpp"
#include "Delay.hpp"

// I2C address of OLED
#define OLED_ADDR 0x78

// Pixel storage area for the OLED screen
static uint8_t OLED_GRAM[128][8];

// Write command to OLED
void OLED::writeCommand(uint8_t cmd)
{
    IOI2C::writeByteToRegister(OLED_ADDR, 0x00, cmd);
}

// Write data to OLED
void OLED::writeData(uint8_t data)
{
    IOI2C::writeByteToRegister(OLED_ADDR, 0x40, data);
}

// Set the display position on OLED
void OLED::setCursor(uint8_t x, uint8_t y)
{
    writeCommand(0xB0 + y);                  // Set page address (0~7)
    writeCommand(((x & 0xF0) >> 4) | 0x10);  // Set higher column address
    writeCommand((x & 0x0F) | 0x00);         // Set lower column address
}

// Initialize OLED
void OLED::init()
{
    Delay::ms(200);

    writeCommand(0xAE); // Turn off display

    writeCommand(0x00); // Set lower column address
    writeCommand(0x10); // Set higher column address
    writeCommand(0x40); // Set start line address
    writeCommand(0xB0); // Set page address

    writeCommand(0x81); // Set contrast
    writeCommand(0xCF); // 128

    writeCommand(0xA1); // Set left-right direction, 0xA1 normal, 0xA0 reverse
    writeCommand(0xC8); // Set up-down direction, 0xC8 normal, 0xC0 reverse

    writeCommand(0xA6); // Set normal/inverse display

    writeCommand(0xA8); // Set multiplex ratio
    writeCommand(0x3F); // 1/64 duty

    writeCommand(0xD3); // Set display offset
    writeCommand(0x00); // No offset

    writeCommand(0xD5); // Set clock divide ratio / oscillator frequency
    writeCommand(0x80); // Default

    writeCommand(0xD9); // Set pre-charge period
    writeCommand(0xF1); // Pre-charge and discharge

    writeCommand(0xDA); // Set COM pins hardware configuration
    writeCommand(0x12); // Sequential COM pins configuration

    writeCommand(0xDB); // Set VCOMH deselect level
    writeCommand(0x30); // 0.83 * VCC

    writeCommand(0x8D); // Set charge pump
    writeCommand(0x14); // Enable charge pump

    writeCommand(0xAF); // Turn on display

    clear();           // Clear screen
    setCursor(0, 0);   // Set cursor to origin (0,0)
}

// Clear screen
void OLED::clear()
{
    uint8_t i, n;
    for (i = 0; i < 8; i++) {
        for (n = 0; n < 128; n++) {
            OLED_GRAM[n][i] = 0;
        }
    }
    refresh(); // Refresh display
}

// Turn on OLED display
void OLED::displayOn()
{
    writeCommand(0x8D); // Charge pump setting
    writeCommand(0x14); // Turn on charge pump
    writeCommand(0xAF); // Turn on screen
}

// Turn off OLED display
void OLED::displayOff()
{
    writeCommand(0x8D); // Charge pump setting
    writeCommand(0x10); // Turn off charge pump
    writeCommand(0xAE); // Turn off screen
}

// Refresh display
void OLED::refresh()
{
    uint8_t i, n;
    for (i = 0; i < 8; i++) {
        writeCommand(0xB0 + i); // Set page address (0~7)
        writeCommand(0x00);     // Set lower column address
        writeCommand(0x10);     // Set higher column address

        for (n = 0; n < 128; n++) {
            writeData(OLED_GRAM[n][i]);
        }
    }
}

void OLED::DrawPoint(u8 x, u8 y, u8 t)
{
    int pos, bx, temp = 0;
    if (x > 127 || y > 63) return; // Out of range
    pos = 7 - y / 8;
    bx = y % 8;
    temp = 1 << (7 - bx);
    if (t)
        OLED_GRAM[x][pos] |= temp;
    else
        OLED_GRAM[x][pos] &= ~temp;
}

// Display a character
void OLED::showChar(uint8_t x, uint8_t y, uint8_t chr, int size, uint8_t mode)
{
    // Simplified implementation, real project needs font lookup
    // Placeholder functionality, actual projects should include font library
    uint8_t temp, t, t1;
    uint8_t y0 = y;

    chr = chr - ' ';
    for (t = 0; t < size; t++) {
        if (size == 12)      temp = oled_asc2_1206[chr][t];
        else                 temp = oled_asc2_1608[chr][t];
        for (t1 = 0; t1 < 8; t1++) {
            if (temp & 0x80)
                DrawPoint(x, y, mode);
            else
                DrawPoint(x, y, !mode);
            temp <<= 1;
            y++;
            if ((y - y0) == size) {
                y = y0;
                x++;
                break;
            }
        }
    }
}

// Display a string
void OLED::showString(uint8_t x, uint8_t y, const uint8_t *p)
{
#define MAX_CHAR_POSX 122
#define MAX_CHAR_POSY 58
    while (*p != '\0') {
        if (x > MAX_CHAR_POSX) {
            x = 0;
            y += 16;
        }
        if (y > MAX_CHAR_POSY) {
            y = x = 0;
            clear();
        }
        showChar(x, y, *p, 12, 1);
        x += 8;
        p++;
    }
}

// Display an integer number
void OLED::showNumber(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size)
{
    uint8_t t, temp;
    uint8_t enshow = 0;

    for (t = 0; t < len; t++) {
        float p = powf(10.0f, float(len - t - 1));
        uint32_t v = (uint32_t)(num / p);  // Convert to integer
        temp = v % 10;                     // Modulo to get digit
        if (enshow == 0 && t < (len - 1)) {
            if (temp == 0) {
                showChar(x + (size / 2) * t, y, ' ', size, 1);
                continue;
            } else {
                enshow = 1;
            }
        }
        showChar(x + (size / 2) * t, y, temp + '0', size, 1);
    }
}

// Display a floating-point number
void OLED::showFloat(uint8_t x, uint8_t y, float num, uint8_t len, uint8_t decimal, uint8_t size)
{
    uint8_t t;
    uint32_t temp;

    // Handle negative numbers
    if (num < 0) {
        showChar(x, y, '-', size, 1);
        x += size / 2;
        num = -num;
    }

    // Display integer part
    temp = (uint32_t)num;
    showNumber(x, y, temp, len, size);

    // Display decimal point
    x += (size / 2) * len;
    showChar(x, y, '.', size, 1);
    x += size / 2;

    // Display decimal part
    float q = powf(10.0f, float(decimal));
    temp = (uint32_t)((num - (uint32_t)num) * q);
    showNumber(x, y, temp, decimal, size);
}
