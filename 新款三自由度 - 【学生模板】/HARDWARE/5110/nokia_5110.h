#ifndef __5110_H
#define __5110_H	 
#include "sys.h"
#include "system_stm32f10x.h"

#define LCD_RST PAout(1)// 	
#define LCD_CE PAout(2)// 	
#define LCD_DC PAout(3)// 
#define SDIN PAout(4)// 
#define SCLK PAout(5)// 

void LCD_init(void);
void LCD_clear(void);
void LCD_move_chinese_string(unsigned char X, unsigned char Y, unsigned char T); 
void LCD_write_english_string(unsigned char X,unsigned char Y,char *s);
void LCD_write_chinese_string(unsigned char X, unsigned char Y,
                   unsigned char ch_with,unsigned char num,
                   unsigned char line,unsigned char row);
void chinese_string(unsigned char X, unsigned char Y, unsigned char T);                 
void LCD_write_char(unsigned char c);
void LCD_draw_bmp_pixel(unsigned char X,unsigned char Y,unsigned char *map,
                  unsigned char Pix_x,unsigned char Pix_y);
void LCD_write_byte(unsigned char dat, unsigned char dc);
void LCD_set_XY(unsigned char X, unsigned char Y);                
void Lcd_Do(void); 
void LCD5110_Print_int(unsigned char X,unsigned char Y,float a,short integer,short decimal);
#endif
