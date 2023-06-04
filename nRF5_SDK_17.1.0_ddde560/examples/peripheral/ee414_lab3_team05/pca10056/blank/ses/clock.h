#ifndef clock_h
#define clock_h
#include <stdio.h>
#include <stdint.h>
#include "nrf_font.h"
// Font data for Microsoft Sans Serif 10pt
extern const uint8_t microsoftSansSerif_10ptBitmaps[];
extern const FONT_INFO microsoftSansSerif_10ptFontInfo;
extern const FONT_CHAR_INFO microsoftSansSerif_10ptDescriptors[];
const uint8_t digit_0[];
const uint8_t digit_1[];
const uint8_t digit_2[];
const uint8_t digit_3[];
const uint8_t digit_4[];
const uint8_t digit_5[];
const uint8_t digit_6[];
const uint8_t digit_7[];
const uint8_t digit_8[];
const uint8_t digit_9[];
const uint8_t digit_colon[];
const uint8_t* p_digit_array[11];
#endif