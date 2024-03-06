#pragma once
#include <stdint.h>
#include <stdbool.h>


#define LCD_WIDTH	160
#define LCD_HEIGHT	128

#define BLACK			0x0000
#define RED				0x00f8
#define GREEN			0xe007
#define BLUE			0x1f00
#define YELLOW			0xe0ff
#define MAGENTA			0x1ff8
#define CYAN			0xff07
#define WHITE			0xffff

void lcd_init(void);
void lcd_fill_box(int x, int y, int width, int height, uint16_t color);
//void lcd_draw_line(int x1, int y1, int x2, int y2, uint16_t color);
void lcd_put_pixel(int x, int y, uint16_t color);
void LCD_DrawLine(int Xstart, int Ystart, int Xend, int Yend, uint16_t color);

void lcd_draw_image(int x, int y, int width, int height, const uint16_t* data);
void lcd_copy(void); // Przesłanie zawartości bufora
void fill_with(uint16_t color);
void LCD_DisplayChar(uint16_t Xpoint, uint16_t Ypoint, char Acsii_Char, uint16_t Color);
void LCD_DisplayString(uint16_t Xstart, uint16_t Ystart, char* pString, uint16_t Color);
void lcd_transfer_done(void);
bool lcd_is_busy(void);
void lcd_draw_horizontal_line(int x, uint16_t color);

#ifndef SRC_LCD_H_
#define SRC_LCD_H_



#endif /* SRC_LCD_H_ */
