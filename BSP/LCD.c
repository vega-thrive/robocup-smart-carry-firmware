#include "LCD.h"
#include "LCD_FONT.h"

static void WriteCmd(uint16_t cmd) {
    *LCD_CMD_ADDR = cmd;
}

static void WriteData(uint16_t data) {
    *LCD_DATA_ADDR = data;
}

void LCD_Init(void) {
    HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_SET);

    WriteCmd(0x01);
    HAL_Delay(100);

    WriteCmd(0xCB);
    WriteData(0x39);
    WriteData(0x2C);
    WriteData(0x00);
    WriteData(0x34);
    WriteData(0x02);

    WriteCmd(0xCF);
    WriteData(0x00);
    WriteData(0xC1);
    WriteData(0x30);

    WriteCmd(0xE8);
    WriteData(0x85);
    WriteData(0x00);
    WriteData(0x78);

    WriteCmd(0xEA);
    WriteData(0x00);
    WriteData(0x00);

    WriteCmd(0xED);
    WriteData(0x64);
    WriteData(0x03);
    WriteData(0x12);
    WriteData(0x81);

    WriteCmd(0xF7);
    WriteData(0x20);

    WriteCmd(0xC0);
    WriteData(0x23);

    WriteCmd(0xC1);
    WriteData(0x10);

    WriteCmd(0xC5);
    WriteData(0x3E);
    WriteData(0x28);

    WriteCmd(0xC7);
    WriteData(0x86);

    WriteCmd(0x36);
    WriteData(0x48);

    WriteCmd(0x3A);
    WriteData(0x55);

    WriteCmd(0xB1);
    WriteData(0x00);
    WriteData(0x18);

    WriteCmd(0xF2);
    WriteData(0x00);

    WriteCmd(0x26);
    WriteData(0x01);

    WriteCmd(0xE0);
    WriteData(0x0F);
    WriteData(0x31);
    WriteData(0x2B);
    WriteData(0x0C);
    WriteData(0x0E);
    WriteData(0x08);
    WriteData(0x4E);
    WriteData(0xF1);
    WriteData(0x37);
    WriteData(0x07);
    WriteData(0x10);
    WriteData(0x03);
    WriteData(0x0E);
    WriteData(0x09);
    WriteData(0x00);

    WriteCmd(0xE1);
    WriteData(0x00);
    WriteData(0x0E);
    WriteData(0x14);
    WriteData(0x03);
    WriteData(0x11);
    WriteData(0x07);
    WriteData(0x31);
    WriteData(0xC1);
    WriteData(0x48);
    WriteData(0x08);
    WriteData(0x0F);
    WriteData(0x0C);
    WriteData(0x31);
    WriteData(0x36);
    WriteData(0x0F);

    WriteCmd(0x11);
    HAL_Delay(120);

    WriteCmd(0x29);
}

void LCD_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    WriteCmd(0x2A);
    WriteData(x0 >> 8);
    WriteData(x0 & 0xFF);
    WriteData(x1 >> 8);
    WriteData(x1 & 0xFF);

    WriteCmd(0x2B);
    WriteData(y0 >> 8);
    WriteData(y0 & 0xFF);
    WriteData(y1 >> 8);
    WriteData(y1 & 0xFF);

    WriteCmd(0x2C);
}

void LCD_FillScreen(uint16_t color) {
    LCD_SetWindow(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);
    for (uint32_t i = 0; i < LCD_WIDTH * LCD_HEIGHT; i++) {
        WriteData(color);
    }
}

void LCD_DrawChar_NoDMA(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg) {
    if (x >= LCD_WIDTH - 8 || y >= LCD_HEIGHT - 16) return;

    uint16_t offset = 0;
    if (c >= '0' && c <= '9') {
        offset = (c - '0') * 16;
    } else if (c >= 'A' && c <= 'Z') {
        offset = (10 + (c - 'A')) * 16;
    } else if (c >= 'a' && c <= 'z') {
        offset = (36 + (c - 'a')) * 16;
    } else {
        return;
    }

    const uint8_t *font = &ASCII_8x16[offset];
    LCD_SetWindow(x, y, x + 7, y + 15);

    for (uint8_t row = 0; row < 16; row++) {
        uint8_t line = font[row];
        for (uint8_t col = 0; col < 8; col++) {
            uint16_t px_color = (line & (0x80 >> col)) ? color : bg;
            WriteData(px_color);
        }
    }
}

void LCD_DrawString_NoDMA(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bg) {
    while (*str) {
        LCD_DrawChar_NoDMA(x, y, *str++, color, bg);
        x += 8;
        if (x >= LCD_WIDTH - 8) {
            x = 0;
            y += 16;
            if (y >= LCD_HEIGHT - 16) break;
        }
    }
}
