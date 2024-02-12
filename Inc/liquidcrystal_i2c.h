#ifndef LIQUIDCRYSTAL_I2C_H_
#define LIQUIDCRYSTAL_I2C_H_

#include "stm32f4xx_hal.h"

/* Command */
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

/* Entry Mode */
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

/* Display On/Off */
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

/* Cursor Shift */
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

/* Function Set */
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

/* Backlight */
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

/* Enable Bit */
#define ENABLE 0x04

/* Read Write Bit */
#define RW 0x0

/* Register Select Bit */
#define RS 0x01

/* Device I2C Address */
#define DEVICE_ADDR     (0x27 << 1)

void LCD_AutoScroll();
void LCD_Backlight();
void LCD_Blink();
void LCD_Clear();
void LCD_CreateSpecialChar(uint8_t, uint8_t[]);
void LCD_Cursor();
void LCD_Display();
void LCD_Home();
void LCD_Init(uint8_t rows);
void LCD_LeftToRight();
void LCD_LoadCustomCharacter(uint8_t char_num, uint8_t *rows);
void LCD_NoAutoScroll();
void LCD_NoBacklight();
void LCD_NoBlink();
void LCD_NoCursor();
void LCD_NoDisplay();
void LCD_PrintLeft();
void LCD_PrintRight();
void LCD_RightToLeft();
void LCD_ScrollDisplayLeft();
void LCD_ScrollDisplayRight();
void LCD_SetBacklight(uint8_t new_val);
void LCD_SetCursor(uint8_t, uint8_t);
void LCD_ShiftDecrement();
void LCD_ShiftIncrement();
void LCD_PrintSpecialChar(uint8_t);
void LCD_PrintStr(const char[]);

#endif /* LIQUIDCRYSTAL_I2C_H_ */
