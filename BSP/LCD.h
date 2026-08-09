#ifndef __LCD_H
#define __LCD_H

#include "main.h"

#define LCD_CMD_ADDR    ((volatile uint16_t*)0x60000000)
#define LCD_DATA_ADDR   ((volatile uint16_t*)0x60000002)

#define LCD_WIDTH       240
#define LCD_HEIGHT      320

#define LCD_BLACK       0x0000
#define LCD_WHITE       0xFFFF
#define LCD_RED         0xF800
#define LCD_GREEN       0x07E0
#define LCD_BLUE        0x001F

void LCD_Init(void);
void LCD_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void LCD_FillScreen(uint16_t color);

void LCD_DrawChar_NoDMA(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg);
void LCD_DrawString_NoDMA(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bg);

#endif
