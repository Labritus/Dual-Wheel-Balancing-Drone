#ifndef __OLED_FONTS_HPP
#define __OLED_FONTS_HPP

#include <stdint.h>

// 12x6 ASCII font (12 height, 6 width)
// Basic ASCII characters 32-126 (space to ~)
extern const uint8_t oled_asc2_1206[95][12];

// 16x8 ASCII font (16 height, 8 width)  
// Basic ASCII characters 32-126 (space to ~)
extern const uint8_t oled_asc2_1608[95][16];

#endif // __OLED_FONTS_HPP